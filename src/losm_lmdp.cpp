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

#define GOAL_STREET "Gray Street"
#define LANDMARK_THRESHOLD 0.01f
#define SPEED_LIMIT_THRESHOLD 30.0f
#define DISTANCE_THRESHOLD 0.01f

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
		const LOSMNode *n1 = edge->get_node_1();
		const LOSMNode *n2 = edge->get_node_2();

		float distance = 0.0f;
		float speedLimit = 0.0f;
		bool isPrimaryGoal = false;
		bool isSecondaryGoal = false;

		bool createForwardDirection = false;
		bool createBackwardDirection = false;

		if (edge->get_node_1()->get_degree() != 2) { // Node 1 is interesting, so find the other interesting one for Node 2.
			createForwardDirection = true;
			n2 = map_directed_path(losm, edge->get_node_2(), edge->get_node_1(), distance, speedLimit, isPrimaryGoal, isSecondaryGoal);
		}

		if (edge->get_node_2()->get_degree() != 2) { // Node 2 is interesting, so find the other interesting one for Node 1.
			createBackwardDirection = true;

			// Handle the special case in which both nodes are 'interesting'. Don't redo this work, also
			// if you did this it would double 'distance'.
			if (!createForwardDirection) {
				n1 = map_directed_path(losm, edge->get_node_1(), edge->get_node_2(), distance, speedLimit, isPrimaryGoal, isSecondaryGoal);
			}
		}

		// If the code made it here, then n1 and n2 are two intersections,
		// and 'distance' and 'time' store the respective distance and time.
		// Now, create the actual pair of LOSMStates.
		for (int i = 0; i < NUM_TIREDNESS_LEVELS; i++) {
			if (createForwardDirection) {
				((StatesMap *)states)->add(new LOSMState(n1, n2, i, distance, speedLimit, isPrimaryGoal, isSecondaryGoal));
			}
			if (createBackwardDirection) {
				((StatesMap *)states)->add(new LOSMState(n2, n1, i, distance, speedLimit, isPrimaryGoal, isSecondaryGoal));
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

	// Create a number of indexed actions equal to the max degree.
	actions = new ActionsMap();
	for (int i = 0; i < maxDegree; i++) {
		((ActionsMap *)actions)->add(new IndexedAction());
	}

	std::cout << "Done Actions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_state_transitions(LOSM *losm)
{
	stateTransitions = new StateTransitionsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());

	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		int currentAction = 0;

		// With this LOSMState, we have two nodes in the original LOSM graph. These nodes had LOSM edges to
		// chains of other LOSMNodes, eventually reaching a terminal node (intersection or dead-end). Moving
		// from 'previous' to 'current', means that we need to find the other LOSMState objects which have
		// 'current' in *their* 'previous'.
		for (auto statePrime : *((StatesMap *)states)) {
			const LOSMState *sp = static_cast<const LOSMState *>(resolve(statePrime));

			// Each action corresponds to whatever the next state is that has a match for the current state's current UID,
			// with the next state's previous UID. This can happen at maximum a number of times equal to

			if (s->get_current()->get_uid() == sp->get_previous()->get_uid()) {

				currentAction++;
				continue;

				// Handle probability differently based on if this is the maximal level of tiredness.
				if (s->get_tiredness() == NUM_TIREDNESS_LEVELS - 1 && sp->get_tiredness() == NUM_TIREDNESS_LEVELS - 1) {
					// This is the same (max) level, so it is 1.0 probability.
					const Action *a = ((ActionsMap *)actions)->get(currentAction);
					currentAction++;

					stateTransitions->set(s, a, sp, 1.0);
					successors[s][a] = sp;
				} else if (s->get_tiredness() == sp->get_tiredness()) {
					// The same level has a probability of 0.9.
					const Action *a = ((ActionsMap *)actions)->get(currentAction);
					currentAction++;

					stateTransitions->set(s, a, sp, 0.9);
					successors[s][a] = sp;
				} else if (s->get_tiredness() + 1 == sp->get_tiredness()) {
					// The next level has a probability of 0.1.
					const Action *a = ((ActionsMap *)actions)->get(currentAction);
					currentAction++;

					stateTransitions->set(s, a, sp, 0.1);
					successors[s][a] = sp;
				}
			}
		}

		std::cout << currentAction << std::endl; std::cout.flush();
	}

	std::cout << "Done State Transitions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_rewards(LOSM *losm)
{
	rewards = new FactoredRewards();

	SASRewardsArray *primaryGoal = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(primaryGoal);

	SASRewardsArray *secondaryGoal = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(secondaryGoal);

	SASRewardsArray *fasterRoads = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(fasterRoads);

	SASRewardsArray *longerRoads = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	((FactoredRewards *)rewards)->add_factor(longerRoads);

	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		for (auto action : *((ActionsMap *)actions)) {
			const Action *a = resolve(action);

			for (auto statePrime : *((StatesMap *)states)) {
				const LOSMState *sp = static_cast<const LOSMState *>(resolve(statePrime));

//				std::cout << s << " " << a << " " << sp << std::endl; std::cout.flush();

				// If this is a valid successor state, then we can set a reward.
				try {
					if (successors.at(s).at(a)->hash_value() == sp->hash_value()) {
						// By default, the value functions always has a small penalty.
						primaryGoal->set(s, a, sp, -0.03);
						secondaryGoal->set(s, a, sp, -0.03);
						fasterRoads->set(s, a, sp, -0.03);
						longerRoads->set(s, a, sp, -0.03);

						// Check if this is a transition to a goal state.
						if (sp->is_primary_goal_state()) {
							primaryGoal->set(s, a, sp, 1.0);
						}

						// Check if this is a secondary goal state.
						if (sp->is_secondary_goal_state()) {
							secondaryGoal->set(s, a, sp, 1.0);
						}

						// Provide a positive reward if this is a 'fast' road.
						if (sp->get_speed_limit() >= SPEED_LIMIT_THRESHOLD) {
							fasterRoads->set(s, a, sp, 1.0);
						}

						// Provide a positive reward if this is a 'long' road.
						if (sp->get_distance() >= DISTANCE_THRESHOLD) {
							longerRoads->set(s, a, sp, 1.0);
						}
					}
				} catch (std::out_of_range &err) {
					// Do nothing, since it just means successors had an undefined value.
					// This is intentional, since successors[s][a] would work, but would
					// create useless items in the hash.
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
		float &distance, float &speedLimit, bool &isPrimaryGoal, bool &isSecondaryGoal)
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

	// Check if this is on the goal street (primary goal).
	if (edge->get_name().compare(GOAL_STREET) == 0) {
		isPrimaryGoal = true;
	}

	// Check if this node is nearby any landmarks (secondary goals).
	for (const LOSMLandmark *landmark : losm->get_landmarks()) {
		if (point_to_line_distance(landmark->get_x(), landmark->get_y(),
				current->get_x(), current->get_y(),
				previous->get_x(), previous->get_y()) < LANDMARK_THRESHOLD) {
			isSecondaryGoal = true;
		}
	}

	// Stop once an intersection or a dead end has been found.
	if (current->get_degree() != 2) {
		return current;
	}

	// Keep going by traversing the neighbor which is not 'previous'.
	std::vector<const LOSMNode *> neighbors;
	losm->get_neighbors(current, neighbors);

	if (neighbors[0] == previous) {
		return map_directed_path(losm, neighbors[1], current, distance, speedLimit, isPrimaryGoal, isSecondaryGoal);
	} else {
		return map_directed_path(losm, neighbors[0], current, distance, speedLimit, isPrimaryGoal, isSecondaryGoal);
	}
}

float LOSMMDP::point_to_line_distance(float x0, float y0, float x1, float y1, float x2, float y2)
{
	float Dx = x2 - x1;
	float Dy = y2 - y1;

	return fabs(Dy * x0 - Dx * y0 - x1 * y2 + x2 * y1) / sqrt(Dx * Dx + Dy * Dy);
}
