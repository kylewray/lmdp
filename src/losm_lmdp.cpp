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
#include "../../librbr/librbr/include/core/rewards/factored_weighted_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include "../../librbr/librbr/include/core/actions/indexed_action.h"

#include "../../librbr/librbr/include/core/core_exception.h"
#include "../../librbr/librbr/include/core/states/state_exception.h"

#include "../../losm/losm/include/losm_exception.h"

#include <iostream> // TODO: Remove me after debug is complete.
#include <fstream>
#include <cmath>
#include <set>
#include <algorithm>

LOSMMDP::LOSMMDP(std::string nodesFilename, std::string edgesFilename, std::string landmarksFilename,
		std::string goal1, std::string goal2)
{
	try {
		goalNodeUID1 = std::stol(goal1);
		goalNodeUID2 = std::stol(goal2);
	} catch (std::exception &err) {
		throw CoreException();
	}

	losm = new LOSM(nodesFilename, edgesFilename, landmarksFilename);

	create_edges_hash(losm);
	create_states(losm);
	create_actions(losm);
	create_state_transitions(losm);
	create_rewards(losm);
	create_misc(losm);
}

LOSMMDP::~LOSMMDP()
{
	delete losm;
}

void LOSMMDP::set_slack(float d1, float d2)
{
	delta.clear();
	delta.push_back(std::max(0.0f, d1));
	delta.push_back(std::max(0.0f, d2));
}

void LOSMMDP::set_uniform_conditional_preference()
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::vector<const State *> p;
	for (auto state : *S) {
		const LOSMState *s = dynamic_cast<const LOSMState *>(resolve(state));
		p.push_back(s);
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
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::vector<const State *> p1;
	std::vector<const State *> p2;

	for (auto state : *S) {
		const LOSMState *s = dynamic_cast<const LOSMState *>(resolve(state));

		if (s->get_tiredness() == 0) {
			p1.push_back(s);
		} else if (s->get_tiredness() == 1){
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

bool LOSMMDP::save_policy(const PolicyMap *policy, std::string filename,
		const std::vector<std::unordered_map<const State *, double> > &V) const
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::ofstream file(filename);
	if (!file.is_open()) {
		return true;
	}

	for (auto state : *S) {
		const State *s = resolve(state);
		const LOSMState *ls = dynamic_cast<const LOSMState *>(s);

		const Action *a = policy->get(s);
		const IndexedAction *ia = dynamic_cast<const IndexedAction *>(a);

		file << ls->get_current_step()->get_uid() << ",";
		file << ls->get_current()->get_uid() << ",";
		file << ls->get_tiredness() << ",";
		file << ls->get_autonomy() << ",";
		file << successors.at(ls).at(ia->get_index())->get_previous_step()->get_uid() << ",";
		file << successors.at(ls).at(ia->get_index())->get_autonomy() << ",";
		for (int i = 0; i < (int)V.size(); i++) {
			file << V.at(i).at(s);
			if (i != (int)V.size() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();

	return false;
}

const LOSMState *LOSMMDP::get_initial_state(std::string initial1, std::string initial2) const
{
	unsigned long initialNodeUID1 = 0;
	unsigned long initialNodeUID2 = 0;

	try {
		initialNodeUID1 = std::stol(initial1);
		initialNodeUID2 = std::stol(initial2);
	} catch (std::exception &err) {
		throw CoreException();
	}

	const StatesMap *S = dynamic_cast<const StatesMap *>(states);

	for (auto state : *S) {
		const LOSMState *s = dynamic_cast<const LOSMState *>(resolve(state));

		if ((s->get_current()->get_uid() == initialNodeUID1 && s->get_previous()->get_uid() == initialNodeUID2) ||
				(s->get_current()->get_uid() == initialNodeUID2 && s->get_previous()->get_uid() == initialNodeUID1)) {
			return s;
		}
	}

	throw CoreException();
}

void LOSMMDP::set_rewards_weights(const std::vector<double> &weights)
{
	FactoredWeightedRewards *R = dynamic_cast<FactoredWeightedRewards *>(rewards);
	if (R == nullptr) {
		throw CoreException();
	}

	R->set_weights(weights);
}

const std::vector<double> &LOSMMDP::get_rewards_weights() const
{
	FactoredWeightedRewards *R = dynamic_cast<FactoredWeightedRewards *>(rewards);
	if (R == nullptr) {
		throw CoreException();
	}

	return R->get_weights();
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
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::cout << "Num Nodes: " << losm->get_nodes().size() << std::endl; std::cout.flush();
	std::cout << "Num Edges: " << losm->get_edges().size() << std::endl; std::cout.flush();
	std::cout << "Num Landmarks: " << losm->get_landmarks().size() << std::endl; std::cout.flush();

	// Create the set of states from the LOSM object's edges, making states for
	// both directions, as well as a tiredness level.
	for (const LOSMEdge *edge : losm->get_edges()) {
		const LOSMNode *current = nullptr;
		const LOSMNode *previous = nullptr;

		const LOSMNode *currentStepNode = nullptr;
		const LOSMNode *previousStepNode = nullptr;

		float distance = 0.0f;
		float speedLimit = 0.0f;
		bool isGoal = false;
		bool isAutonomyCapable = false;

		// We must create both if they are both 'interesting' nodes, because there would be no other edge,
		// that it would iterate over.
		bool createBoth = false;

		if (edge->get_node_1()->get_degree() != 2 && edge->get_node_2()->get_degree() != 2) {
			// This computes the distance, speed limit, etc.
			const LOSMNode *nothing = nullptr;
			const LOSMNode *nothingStep = nullptr;
			map_directed_path(losm, edge->get_node_1(), edge->get_node_2(), distance, speedLimit, nothing, nothingStep);

			current = edge->get_node_1();
			previous = edge->get_node_2();

			currentStepNode = edge->get_node_2();
			previousStepNode = edge->get_node_1();

			createBoth = true;

		} else if (edge->get_node_1()->get_degree() != 2 && edge->get_node_2()->get_degree() == 2) {
			// Node 1 is interesting, so find the other interesting one for Node 2.
			current = edge->get_node_1();
			currentStepNode = edge->get_node_2();
			map_directed_path(losm, edge->get_node_2(), edge->get_node_1(), distance, speedLimit, previous, previousStepNode);

		} else if (edge->get_node_1()->get_degree() == 2 && edge->get_node_2()->get_degree() != 2) {
			// Node 2 is interesting, so find the other interesting one for Node 1.
			current = edge->get_node_2();
			currentStepNode = edge->get_node_1();
			map_directed_path(losm, edge->get_node_1(), edge->get_node_2(), distance, speedLimit, previous, previousStepNode);

		} else {
			continue;
		}

		if ((current->get_uid() == goalNodeUID1 && previous->get_uid() == goalNodeUID2) ||
				(current->get_uid() == goalNodeUID2 && previous->get_uid() == goalNodeUID1)) {
			isGoal = true;
			std::cout << "Added Goal State!" << std::endl; std::cout.flush();
		}

		if (speedLimit >= AUTONOMY_SPEED_LIMIT_THRESHOLD) {
			isAutonomyCapable = true;
		}

		// If the code made it here, then n1 and n2 are two intersections,
		// and 'distance' and 'time' store the respective distance and time.
		// Now, create the actual pair of LOSMStates.
		for (int i = 0; i < NUM_TIREDNESS_LEVELS; i++) {
			// Autonomy is not enabled. This always exists.
			S->add(new LOSMState(current, previous, i, false,
													distance, speedLimit, isGoal, isAutonomyCapable,
													currentStepNode, previousStepNode));
			if (createBoth) {
				S->add(new LOSMState(previous, current, i, false,
														distance, speedLimit, isGoal, isAutonomyCapable,
														previousStepNode, currentStepNode));
			}

			// If possible, create the states in which autonomy is enabled. This may or may not exist.
			if (isAutonomyCapable) {
				S->add(new LOSMState(current, previous, i, true,
														distance, speedLimit, isGoal, isAutonomyCapable,
														currentStepNode, previousStepNode));
				if (createBoth) {
					S->add(new LOSMState(previous, current, i, true,
														distance, speedLimit, isGoal, isAutonomyCapable,
														previousStepNode, currentStepNode));
				}
			}
		}
	}

	/* Check!
	for (auto state : *((StatesMap *)states)) {
		const LOSMState *s = static_cast<const LOSMState *>(resolve(state));

		int count = 0;
		for (auto nextState : *((StatesMap *)states)) {
			const LOSMState *sp = static_cast<const LOSMState *>(resolve(nextState));

//			if (s == sp) {
//				continue;
//			}

			if (s->get_previous() == sp->get_previous() && s->get_current() == sp->get_current() &&
					s->get_tiredness() == sp->get_tiredness() && s->get_autonomy() == sp->get_autonomy() &&
					s->get_uniqueness_index() == sp->get_uniqueness_index())
			{
				count++;
//				std::cout << "BADNESS!\n"; std::cout.flush();
			}
		}

		if (count != 1) {
			std::cout << s->get_previous()->get_uid() << " " << s->get_current()->get_uid() << " BADNESS!!!!!\n"; std::cout.flush();
		}
	}
	//*/

	std::cout << "Num States: " << S->get_num_states() << std::endl; std::cout.flush();

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
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);

	for (int i = 0; i < maxDegree * 2; i++) {
		A->add(new IndexedAction());
	}

	std::cout << "Num Actions: " << A->get_num_actions() << std::endl; std::cout.flush();

	std::cout << "Done Actions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_state_transitions(LOSM *losm)
{
	stateTransitions = new StateTransitionsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
//	StateTransitionsArray *T = dynamic_cast<StateTransitionsArray *>(stateTransitions);

	StatesMap *S = dynamic_cast<StatesMap *>(states);
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);

	for (auto state : *S) {
		const LOSMState *s = dynamic_cast<const LOSMState *>(resolve(state));

		// Must store the mapping from a next state (prev, cur, auto) to action taken.
		std::unordered_map<const LOSMNode *,
			std::unordered_map<const LOSMNode *,
				std::unordered_map<bool,
					std::unordered_map<unsigned int, const Action *> > > > map;
		int index = 0;

		// Only set transitions if this is not a goal state. Goal states will always loop to themselves (handled at the end).
		if (!s->is_goal()) {
			for (auto nextState : *S) {
				const LOSMState *sp = dynamic_cast<const LOSMState *>(resolve(nextState));

				// If the current intersection node for current state matches the previous node for the next state,
				// then this possibly a non-zero transition probability. It now depends on the tiredness level.
				if (s->get_current() != sp->get_previous()) {
					continue;
				}

				// This is a valid node. First check if a mapping already exists for taking an action at this next state.
				const Action *a = nullptr;
				try {
					a = map.at(sp->get_previous()).at(sp->get_current()).at(sp->get_autonomy()).at(sp->get_uniqueness_index());
				} catch (const std::out_of_range &err) {
					a = A->get(index);
					map[sp->get_previous()][sp->get_current()][sp->get_autonomy()][sp->get_uniqueness_index()] = a;
					index++;
				}

				// Determine the probability, while verifying the state transition makes sense in terms of tiredness level.
				double p = -1.0;
				if (s->get_tiredness() == NUM_TIREDNESS_LEVELS - 1 && sp->get_tiredness() == NUM_TIREDNESS_LEVELS - 1) {
					p = 1.0;
				} else if (s->get_tiredness() == sp->get_tiredness()) {
					p = 0.9;
				} else if (s->get_tiredness() + 1 == sp->get_tiredness()) {
					p = 0.1;
				}

				// If no probability was assigned, it means that while there is an action, it is impossible to transition
				// from s's level of tiredness to sp's level of tiredness. Otherwise, we can assign a state transition.
				if (p >= 0.0) {
					stateTransitions->set(s, a, sp, p);

					const IndexedAction *ia = dynamic_cast<const IndexedAction *>(a);
					successors[s][ia->get_index()] = sp;
				}
			}
		}

		// Recall that the degree of the node corresponds to how many actions are available. Thus,
		// we need to fill in the remaining number of actions as a state transition to itself.
		// The reward for any self-transition will be defined to be the largest negative number
		// possible. This must be done for both enabled and disabled autonomy.
		for (int i = index; i < (int)IndexedAction::get_num_actions(); i++) {
			const Action *a = A->get(i);
			stateTransitions->set(s, a, s, 1.0);
			successors[s][i] = s;
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

			std::vector<const LOSMState *> asdf;

			for (auto nextState : *((StatesMap *)states)) {
				const LOSMState *sp = static_cast<const LOSMState *>(resolve(nextState));

				sum += stateTransitions->get(s, a, sp);

				if (stateTransitions->get(s, a, sp) > 0.0) {
					asdf.push_back(sp);

//					std::cout << stateTransitions->get(s, a, sp);
//					std::cout << " + ";
				}
			}

//			std::cout << " ==== " << sum << std::endl; std::cout.flush();

			if (sum > 1.00 || sum < 0.999999) {
				std::cout << "Sum is: " << sum <<  " ... Bad State " << s->get_previous()->get_uid() << "_" << s->get_current()->get_uid() << " Action " << a->to_string();
				std::cout << " Next States: "; std::cout.flush();
				for (const LOSMState *sp : asdf) {
					std::cout << sp << "**" << sp->get_previous()->get_uid() << "_" << sp->get_current()->get_uid();
					std::cout << "(" << sp->get_tiredness() << ", " << sp->get_autonomy() << "::" << stateTransitions->get(s, a, sp) << ") ";
				}
				std::cout << std::endl; std::cout.flush();
			}
		}
	}
	//*/

	std::cout << "Done State Transitions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_rewards(LOSM *losm)
{
	rewards = new FactoredWeightedRewards();
	FactoredWeightedRewards *R = dynamic_cast<FactoredWeightedRewards *>(rewards);

	StatesMap *S = dynamic_cast<StatesMap *>(states);
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);
	StateTransitionsArray *T = dynamic_cast<StateTransitionsArray *>(stateTransitions);

	SASRewardsArray *timeReward = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	R->add_factor(timeReward);

	SASRewardsArray *autonomyReward = new SASRewardsArray(LOSMState::get_num_states(), IndexedAction::get_num_actions());
	R->add_factor(autonomyReward);

	float floatMaxCuda = -1e+35;

	for (auto state : *S) {
		const LOSMState *s = dynamic_cast<const LOSMState *>(resolve(state));

		for (auto action : *A) {
			const Action *a = resolve(action);

			for (auto statePrime : *S) {
				const LOSMState *sp = dynamic_cast<const LOSMState *>(resolve(statePrime));

//				std::cout << s->get_index() << " " << ((IndexedAction *)a)->get_index() << " " << sp->get_index() << std::endl; std::cout.flush();

//				timeReward->set(s, a, sp, floatMaxCuda);
//				autonomyReward->set(s, a, sp, floatMaxCuda);

				// If this is a valid successor state, then we can set a non-trivial reward.
				if (T->get(s, a, sp) > 0.0) {
					// Check if this is a self-transition, which is fine if the agent is in a goal
					// state, but otherwise yields a large negative reward. This is how I am able to
					// handle having the same number of actions for each state, even if the degree of
					// the node is less than the number of actions.
					if (s == sp && !sp->is_goal()) {
						// Goal states always transition to themselves (absorbing), with zero reward.
						timeReward->set(s, a, s, floatMaxCuda);
						autonomyReward->set(s, a, s, floatMaxCuda);

						continue;
					}

					// If you got here, then s != sp, so any transition to a goal state is cost of 0 for the time reward.
					if (sp->is_goal()) {
						timeReward->set(s, a, sp, 0.0);
						autonomyReward->set(s, a, sp, 0.0);

						continue;
					}

					// Enabling or disabling autonomy changes the speed of the car, but provides
					// a positive reward for safely driving autonomously, regardless of the
					// tiredness of the driver.
//					if (sp->get_autonomy()) {
//						timeReward->set(s, a, sp, -sp->get_distance() / (sp->get_speed_limit() * AUTONOMY_SPEED_LIMIT_FACTOR) * TO_SECONDS);
//					} else {
						timeReward->set(s, a, sp, -sp->get_distance() / sp->get_speed_limit() * TO_SECONDS - INTERSECTION_WAIT_TIME_IN_SECONDS);
//					}

					//*
					if (!sp->get_autonomy() && sp->get_tiredness() > 0) {
//					if (sp->is_autonomy_capable() && !sp->get_autonomy() && sp->get_tiredness() > 0) {
						autonomyReward->set(s, a, sp, -sp->get_distance() / sp->get_speed_limit() * TO_SECONDS - INTERSECTION_WAIT_TIME_IN_SECONDS);
					} else {
						autonomyReward->set(s, a, sp, -INTERSECTION_WAIT_TIME_IN_SECONDS);
					}
					//*/

					/*
					// If the road is autonomy capable, you are not autonomous, and you are tired, then take a penalty.
					// Otherwise, no penalty is given. In other words, you are penalized for every second spent driving
					// manually when you are tired.
					if (sp->is_autonomy_capable()) {
						if (sp->get_autonomy()) {
							if (sp->get_tiredness() == 0) {
								autonomyReward->set(s, a, sp, 0.0); //-sp->get_distance() / sp->get_speed_limit() * 0.5); // Autonomy Possible + Autonomy Enabled + Awake = Alright
							} else {
								autonomyReward->set(s, a, sp, 0.0); //-sp->get_distance() / sp->get_speed_limit() * 0.1); // Autonomy Possible + Autonomy Enabled + Tired = Good!!!
							}
						} else {
							if (sp->get_tiredness() == 0) {
								autonomyReward->set(s, a, sp, 0.0); //-sp->get_distance() / sp->get_speed_limit() * 0.5); // Autonomy Possible + Autonomy Disabled + Awake = Alright
							} else {
								autonomyReward->set(s, a, sp, -sp->get_distance() / sp->get_speed_limit()); // Autonomy Possible + Autonomy Disabled + Tired = Bad!!!
							}
						}
					} else {
						if (sp->get_tiredness() == 0) {
							autonomyReward->set(s, a, sp, 0.0); //-sp->get_distance() / sp->get_speed_limit() * 0.1); // Autonomy Impossible + Awake = Good!!!
						} else {
							autonomyReward->set(s, a, sp, 0.0); //-sp->get_distance() / sp->get_speed_limit() * 0.5); // Autonomy Impossible + Tired = Alright
						}
					}
					//*/
				}
			}
		}
	}

	std::cout << "Done Rewards!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_misc(LOSM *losm)
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	// The initial state is arbitrary.
	initialState = new Initial(S->get(0));

	// Infinite horizon with a discount factor of 0.9.
	horizon = new Horizon(0.99);

	std::cout << "Done Misc!" << std::endl; std::cout.flush();
}

void LOSMMDP::map_directed_path(const LOSM *losm, const LOSMNode *current, const LOSMNode *previous,
		float &distance, float &speedLimit,
		const LOSMNode *&result, const LOSMNode *&resultStep)
{
	// Update the distance and time.
	const LOSMEdge *edge = nullptr;
	try {
		edge = edgeHash.at(current->get_uid()).at(previous->get_uid());
	} catch (const std::out_of_range &err) {
		edge = edgeHash.at(previous->get_uid()).at(current->get_uid());
	}
	speedLimit = (speedLimit * distance + edge->get_speed_limit() * edge->get_distance()) / (distance + edge->get_distance());
	distance += edge->get_distance();

	// Stop once an intersection or a dead end has been found.
	if (current->get_degree() != 2) {
		result = current;
		resultStep = previous;
		return;
	}

	// Keep going by traversing the neighbor which is not 'previous'.
	std::vector<const LOSMNode *> neighbors;
	losm->get_neighbors(current, neighbors);

	if (neighbors[0] == previous) {
		return map_directed_path(losm, neighbors[1], current, distance, speedLimit, result, resultStep);
	} else {
		return map_directed_path(losm, neighbors[0], current, distance, speedLimit, result, resultStep);
	}
}

float LOSMMDP::point_to_line_distance(float x0, float y0, float x1, float y1, float x2, float y2)
{
	float Dx = x2 - x1;
	float Dy = y2 - y1;

	return fabs(Dy * x0 - Dx * y0 - x1 * y2 + x2 * y1) / sqrt(Dx * Dx + Dy * Dy);
}
