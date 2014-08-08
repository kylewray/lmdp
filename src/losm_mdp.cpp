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


#include "../include/losm_mdp.h"
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

	// Create the set of states from the LOSM object's edges, making states for
	// both directions, as well as a tiredness level.
	for (const LOSMEdge *edge : losm->get_edges()) {
		const LOSMNode *n1 = edge->get_node_1();
		const LOSMNode *n2 = edge->get_node_2();

		float distance = 0.0f;
		float time = 0.0f;

		// Check if one of these is an intersection.
		if (n1->get_degree() != 2) {
			n2 = map_directed_path(losm, n2, n1, distance, time);
		} else if (n2->get_degree() != 2) {
			n1 = map_directed_path(losm, n1, n2, distance, time);
		} else {
			continue;
		}

		// If the code made it here, then n1 and n2 are two intersections,
		// and 'distance' and 'time' store the respective distance and time.
		// Now, create the actual pair of LOSMStates.
		for (int i = 0; i < NUM_TIREDNESS_LEVELS; i++) {
			((StatesMap *)states)->add(new LOSMState(n1, n2, i, distance, time));
			((StatesMap *)states)->add(new LOSMState(n2, n1, i, distance, time));
		}
	}

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
			if (s->get_current() == sp->get_previous()) {
				// Handle probability differently based on if this is the maximal level of tiredness.
				if (s->get_tiredness() == NUM_TIREDNESS_LEVELS - 1) {
					// This is the same (max) level, so it is 1.0 probability.
					if (s->get_tiredness() == sp->get_tiredness()) {
						const Action *a = ((ActionsMap *)actions)->get(currentAction);
						currentAction++;

						stateTransitions->set(s, a, sp, 1.0);
						successors[s][a] = sp;
					}
				} else {
					// The same level has a probability of 0.9. The next level has a probability of 0.1.
					if (s->get_tiredness() == sp->get_tiredness()) {
						const Action *a = ((ActionsMap *)actions)->get(currentAction);
						currentAction++;

						stateTransitions->set(s, a, sp, 0.9);
						successors[s][a] = sp;
					} else if (s->get_tiredness() + 1 == sp->get_tiredness()) {
						const Action *a = ((ActionsMap *)actions)->get(currentAction);
						currentAction++;

						stateTransitions->set(s, a, sp, 0.1);
						successors[s][a] = sp;
					}
				}
			}
		}
	}

	std::cout << "Done State Transitions!" << std::endl; std::cout.flush();
}

void LOSMMDP::create_rewards(LOSM *losm)
{
	// TODO: Start here.

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
		float &distance, float &time)
{
	// Once we have found an intersection, we can stop.
	if (current->get_degree() != 2) {
		return current;
	}

	// Update the distance and time.
	try {
		const LOSMEdge *edge = edgeHash.at(current->get_uid()).at(previous->get_uid());
		distance += edge->get_distance();
		time += edge->get_distance() / (float)edge->get_speed_limit();
	} catch (const std::out_of_range &err) {
		const LOSMEdge *edge = edgeHash.at(previous->get_uid()).at(current->get_uid());
		distance += edge->get_distance();
		time += edge->get_distance() / (float)edge->get_speed_limit();
	}

	// Keep going by traversing the neighbor which is not 'previous'.
	std::vector<const LOSMNode *> neighbors;
	losm->get_neighbors(current, neighbors);

	// If this actually has one neighbor, then it must be a dead end. (Degree is not fully correct on nodes.)
	if (neighbors.size() == 1) {
		return current;
	}

	if (neighbors[0] == previous) {
		return map_directed_path(losm, neighbors[1], current, distance, time);
	} else {
		return map_directed_path(losm, neighbors[0], current, distance, time);
	}
}
