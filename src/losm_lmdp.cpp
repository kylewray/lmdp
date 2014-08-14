/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Wray, University of Massachusetts
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include "../include/losm_lmdp.h"
#include "../include/losm_state.h"

#include "../../librbr/librbr/include/core/states/states_map.h"
#include "../../librbr/librbr/include/core/actions/actions_map.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transitions_array.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include "../../librbr/librbr/include/core/actions/indexed_action.h"

#include "../../librbr/librbr/include/core/states/state_exception.h"

#include "../../losm/losm/include/losm_exception.h"

#include <iostream> // TODO: Remove me after debug is complete.
#include <cmath>
#include <set>
#include <algorithm>

LOSMMDP::LOSMMDP(std::string nodesFilename, std::string edgesFilename, std::string landmarksFilename)
{
	LOSM *losm = new LOSM(nodesFilename, edgesFilename, landmarksFilename);

	create_edges_hash(losm);
	create_states(losm);
	create_actions(losm);
	create_state_transitions(losm);
	create_rewards(losm);
	create_misc(losm);

	delete losm;
}

LOSMMDP::~LOSMMDP()
{ }

void LOSMMDP::set_slack(float d1, float d2)
{
	delta.clear();
	delta.push_back(std::max(0.0f, d1));
	delta.push_back(std::max(0.0f, d2));
}

void LOSMMDP::set_uniform_conditional_preference()
{
	std::vector<const State *> p;
	for (auto s : *((StatesMap *)states)) {
		p.push_back(resolve(s));
	}

	partition.clear();
	partition.push_back(p);

	std::vector<unsigned int> r;
	r.push_back(0);
	r.push_back(1);

	ordering.clear();
	ordering.push_back(r);
}

void LOSMMDP::set_tiredness_conditional_preference()
{
	std::vector<const State *> p1;
	std::vector<const State *> p2;

	std::vector<const State *> p;
	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));
		if (s->get_tiredness() == 0) {
			p1.push_back(s);
		} else {
			p2.push_back(s);
		}
	}

	partition.clear();
	partition.push_back(p1);
	partition.push_back(p2);

	std::vector<unsigned int> r1;
	r1.push_back(0);
	r1.push_back(1);

	std::vector<unsigned int> r2;
	r2.push_back(1);
	r2.push_back(0);

	ordering.clear();
	ordering.push_back(r1);
	ordering.push_back(r2);
}

void LOSMMDP::create_edges_hash(LOSM *losm)
{
	for (const LOSMEdge *edge : losm->get_edges()) {
		edgeHash[edge->get_node_1()->get_uid()][edge->get_node_2()->get_uid()] = edge;
	}
	std::cout << "Done Create Edges Hash!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_states(LOSM *losm)
{
	LOSMState::reset_indexer();

	states = new StatesMap();

	std::cout << "Num Nodes: " << losm->get_nodes().size() << std::endl; std::cout.flush();
	std::cout << "Num Edges: " << losm->get_edges().size() << std::endl; std::cout.flush();
	std::cout << "Num Landmarks: " << losm->get_landmarks().size() << std::endl; std::cout.flush();

	// Create the set of states from the LOSM object's edges, making states for
	// both directions, as well as a tiredness level.
	for (const LOSMEdge *edge : losm->get_edges()) {
		const LOSMNode *current = nullptr;
		const LOSMNode *previous = nullptr;

		float distance = 0.0f;
		float speedLimit = 0.0f;
		bool isGoal = false;
		bool isAutonomyCapable = false;

		bool createBoth = false;

		if (edge->get_node_1()->get_degree() != 2 && edge->get_node_2()->get_degree() != 2) {
			// This computes the distance, speed limit, etc.
			map_directed_path(losm, edge->get_node_1(), edge->get_node_2(), distance, speedLimit, isGoal);
			current = edge->get_node_1();
			previous = edge->get_node_2();
			createBoth = true;
		} else if (edge->get_node_1()->get_degree() != 2) { // Node 1 is interesting, so find the other interesting one for Node 2.
			current = edge->get_node_1();
			previous = map_directed_path(losm, edge->get_node_2(), edge->get_node_1(), distance, speedLimit, isGoal);
		} else if (edge->get_node_2()->get_degree() != 2) { // Node 2 is interesting, so find the other interesting one for Node 1.
			current = edge->get_node_2();
			previous = map_directed_path(losm, edge->get_node_1(), edge->get_node_2(), distance, speedLimit, isGoal);
		} else {
			continue;
		}

		if (speedLimit >= AUTONOMY_SPEED_LIMIT_THRESHOLD && distance >= AUTONOMY_DISTANCE_THRESHOLD) {
			isAutonomyCapable = true;
		}

		// If the code made it here, then n1 and n2 are two intersections,
		// and 'distance' and 'time' store the respective distance and time.
		// Now, create the actual pair of LOSMStates.
		for (int i = 0; i < NUM_TIREDNESS_LEVELS; i++) {
			// Autonomy is not enabled.
			((StatesMap *)states)->add(new LOSMState(current, previous, i, false, distance, speedLimit, isGoal, isAutonomyCapable));
			if (createBoth) {
				((StatesMap *)states)->add(new LOSMState(previous, current, i, false, distance, speedLimit, isGoal, isAutonomyCapable));
			}

			// If possible, create the states in which autonomy is enabled.
			if (isAutonomyCapable) {
				((StatesMap *)states)->add(new LOSMState(current, previous, i, true, distance, speedLimit, isGoal, isAutonomyCapable));
				if (createBoth) {
					((StatesMap *)states)->add(new LOSMState(previous, current, i, true, distance, speedLimit, isGoal, isAutonomyCapable));
				}
			}
		}
	}

	std::cout << "Num States: " << ((StatesMap *)states)->get_num_states() << std::endl; std::cout.flush();

	std::cout << "Done States!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_actions(LOSM *losm)
{
	// Compute the maximum degree in the graph.
	int maxDegree = 0;
	for (const LOSMNode *node : losm->get_nodes()) {
		if ((int)node->get_degree() > maxDegree) {
			maxDegree = node->get_degree();
		}
	}

	IndexedAction::reset_indexer();

	// Create a number of indexed actions equal to the max degree times two. The first set of
	// actions assumes the agent does not wish to enable autonomy, and the second set of actions
	// assumes the agent wishes to enable autonomy.
	actions = new ActionsMap();
	for (int i = 0; i < maxDegree * 2; i++) {
		((ActionsMap *)actions)->add(new IndexedAction());
	}

	std::cout << "Done Actions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_state_transitions(LOSM *losm)
{
	stateTransitions = new StateTransitionsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());

	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		// TODO: Complete from here... You need probably do not need to double the action index, but you do need to
		// duplicate (copy-paste, or for-loop of 2) for states which keep the same autonomy, but only if the state
		// allows it, or swap autonomy for no-autonomy if the action indices are MODULOUS (|A| / 2) == 0.

		int actionIndex[NUM_TIREDNESS_LEVELS];
		for (int i = 0; i < NUM_TIREDNESS_LEVELS; i++) {
			actionIndex[i] = 0;
		}

		for (auto statePrime : *((StatesMap *)states)) {
			const LOSMState *sp = static_cast<const LOSMState *>(resolve(statePrime));

			// If the current intersection node for current state matches the previous node for the next state,
			// then this possibly a non-zero transition probability. It now depends on the tiredness level.
			if (s->get_current() != sp->get_previous()) {
				continue;
			}

//			std::cout << sp->get_tiredness() << ": " << actionIndex[sp->get_tiredness()] << std::endl; std::cout.flush();

			if (s->get_tiredness() == NUM_TIREDNESS_LEVELS - 1 && sp->get_tiredness() == NUM_TIREDNESS_LEVELS - 1) {
				// This is the same (max) level, so it is 1.0 probability.
				const Action *a = ((ActionsMap *)actions)->get(actionIndex[NUM_TIREDNESS_LEVELS - 1]);
				actionIndex[sp->get_tiredness()]++;

				stateTransitions->set(s, a, sp, 1.0);
				successors[s][a] = sp;
			} else if (s->get_tiredness() == sp->get_tiredness()) {
				// The same level has a probability of 0.9.
				const Action *a = ((ActionsMap *)actions)->get(actionIndex[s->get_tiredness()]);
				actionIndex[sp->get_tiredness()]++;

				stateTransitions->set(s, a, sp, 0.9);
				successors[s][a] = sp;
			} else if (s->get_tiredness() + 1 == sp->get_tiredness()) {
				// The next level has a probability of 0.1.
				const Action *a = ((ActionsMap *)actions)->get(actionIndex[s->get_tiredness() + 1]);
				actionIndex[sp->get_tiredness()]++;

				stateTransitions->set(s, a, sp, 0.1);
				successors[s][a] = sp;
			}
		}

		// Recall that the degree of the node corresponds to how many actions are available. Thus,
		// we need to fill in the remaining number of actions as a state transition to itself.
		// The reward for any self-transition will be defined to be the largest negative number
		// possible. This must be done for both enabled and disabled autonomy.
		for (int i = s->get_current()->get_degree(); i < (int)IndexedAction::get_num_actions() / 2; i++) {
			const Action *a = ((ActionsMap *)actions)->get(i);
			stateTransitions->set(s, a, s, 1.0);
			successors[s][a] = s;

			a = ((ActionsMap *)actions)->get(i + IndexedAction::get_num_actions() / 2);
			stateTransitions->set(s, a, s, 1.0);
			successors[s][a] = s;
		}
	}

	/*
	// CHECK!!!!!
	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		for (auto action : *((ActionsMap *)actions)) {
			const Action *a = resolve(action);

			double sum = 0.0;

//			std::cout << s->get_previous()->get_uid() << " " << s->get_current()->get_uid() << " ---- Sum is ";

			for (auto nextState : *((StatesMap *)states)) {
				const LOSMState *sp = static_cast<const LOSMState *>(resolve(nextState));

				sum += stateTransitions->get(s, a, sp);

//				if (stateTransitions->get(s, a, sp) > 0.0) {
//					std::cout << stateTransitions->get(s, a, sp);
//					std::cout << " + ";
//				}
			}

//			std::cout << " ==== " << sum << std::endl; std::cout.flush();

			if (sum > 1.00 || sum < 0.999999) {
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";std::cout.flush();
			}
		}
	}
	//*/

	std::cout << "Done State Transitions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_rewards(LOSM *losm)
{
	rewards = new FactoredRewards();

	SASRewardsArray *timeReward = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(timeReward);

	SASRewardsArray *autonomyReward = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(autonomyReward);

	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		for (auto action : *((ActionsMap *)actions)) {
			const Action *a = resolve(action);

			for (auto statePrime : *((StatesMap *)states)) {
				const LOSMState *sp = static_cast<const LOSMState *>(resolve(statePrime));

				// If this is a valid successor state, then we can set a non-trivial reward.
				if (((StateTransitionsArray *)stateTransitions)->get(s, a, sp) > 0.0) {
					// Check if this is a self-transition, which essentially yields a
					// large negative reward.
					if (s == sp) {
						float floatMaxCuda = 1e+35;

						timeReward->set(s, a, s, -floatMaxCuda);
						autonomyReward->set(s, a, s, -floatMaxCuda);

						continue;
					}

					// If this is autonomous capable...
					if (sp->get_speed_limit() >= AUTONOMY_SPEED_LIMIT_THRESHOLD) {
						if (sp->get_tiredness() == 0) {

						} else if (sp->get_tiredness() == 1) {

						}

						timeReward->set(s, a, sp, -sp->get_distance() / (sp->get_speed_limit() * AUTONOMY_SPEED_LIMIT_FACTOR));
						autonomyReward->set(s, a, sp, 1.0);
					} else {
						timeReward->set(s, a, sp, -sp->get_distance() / sp->get_speed_limit());
						autonomyReward->set(s, a, sp, 0.0);
					}
				}
			}
		}
	}

	std::cout << "Done Rewards!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_misc(LOSM *losm)
{
	// The initial state is arbitrary.
	initialState = new Initial(((StatesMap *)states)->get(0));

	// Infinite horizon with a discount factor of 0.9.
	horizon = new Horizon(0.9);

	std::cout << "Done Misc!" << std::endl; std::cout.flush();
}

const LOSMNode *LOSMMDP::map_directed_path(const LOSM *losm, const LOSMNode *current, const LOSMNode *previous,
		float &distance, float &speedLimit, bool &isGoal)
{
	// Update the distance and time.
	const LOSMEdge *edge = nullptr;
	try {
		edge = edgeHash.at(current->get_uid()).at(previous->get_uid());
	} catch (const std::out_of_range &err) {
		edge = edgeHash.at(previous->get_uid()).at(current->get_uid());
	}
	speedLimit += (speedLimit * distance + edge->get_speed_limit() * edge->get_distance()) / (distance + edge->get_distance());
	distance += edge->get_distance();

	// Check if this is a goal street.
	if (edge->get_name().compare(GOAL_STREET_NAME) == 0) {
		isGoal = true;
	}

//	// Check if this node is nearby any landmarks (secondary goals).
//	for (const LOSMLandmark *landmark : losm->get_landmarks()) {
//		if (point_to_line_distance(landmark->get_x(), landmark->get_y(),
//				current->get_x(), current->get_y(),
//				previous->get_x(), previous->get_y()) < LANDMARK_THRESHOLD) {
//			isSecondaryGoal = true;
//		}
//	}

	// Stop once an intersection or a dead end has been found.
	if (current->get_degree() != 2) {
		return current;
	}

	// Keep going by traversing the neighbor which is not 'previous'.
	std::vector<const LOSMNode *> neighbors;
	losm->get_neighbors(current, neighbors);

	if (neighbors[0] == previous) {
		return map_directed_path(losm, neighbors[1], current, distance, speedLimit, isGoal);
	} else {
		return map_directed_path(losm, neighbors[0], current, distance, speedLimit, isGoal);
	}
}

float LOSMMDP::point_to_line_distance(float x0, float y0, float x1, float y1, float x2, float y2)
{
	float Dx = x2 - x1;
	float Dy = y2 - y1;

	return fabs(Dy * x0 - Dx * y0 - x1 * y2 + x2 * y1) / sqrt(Dx * Dx + Dy * Dy);
}
