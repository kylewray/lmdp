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


#include "../include/grid_lmdp.h"

#include "../../librbr/librbr/include/core/states/states_map.h"
#include "../../librbr/librbr/include/core/actions/actions_map.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transitions_array.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include "../../librbr/librbr/include/core/states/indexed_state.h"
#include "../../librbr/librbr/include/core/actions/indexed_action.h"

#include <iostream>
#include <random>
#include <algorithm>

GridLMDP::GridLMDP(unsigned int seed, unsigned int gridSize, unsigned int numBlockedStates, double tertiaryPenalty)
{
	std::srand(seed);

	// First, randomly select states to be blocked.
	size = gridSize;
	for (int i = 0; i < numBlockedStates; i++) {
		int x = (int)((float)(size - 2) * (float)rand() / (float)RAND_MAX + 1.0f);
		int y = (int)((float)(size - 2) * (float)rand() / (float)RAND_MAX + 1.0f);
		blocked.push_back(0 + y * size + x);
		blocked.push_back(1 * size * size + y * size + x);
	}
	penalty = tertiaryPenalty;

	// Now setup the LMDP according to these parameters.
	create_states();
	create_actions();
	create_state_transitions();
	create_rewards();
	create_misc();
}

GridLMDP::~GridLMDP()
{ }

void GridLMDP::set_slack(float d1, float d2, float d3)
{
	delta.clear();
	delta.push_back(std::max(0.0f, d1));
	delta.push_back(std::max(0.0f, d2));
	delta.push_back(std::max(0.0f, d3));
}

void GridLMDP::set_default_conditional_preference()
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::vector<const State *> p;
	for (auto s : *S) {
		p.push_back(resolve(s));
	}

	partition.clear();
	partition.push_back(p);

	std::vector<unsigned int> r;
	r.push_back(0);
	r.push_back(1);
	r.push_back(2);

	ordering.clear();
	ordering.push_back(r);
}

void GridLMDP::set_split_conditional_preference()
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	std::vector<const State *> p1;
	std::vector<const State *> p2;

	for (int c = 0; c < 2; c++) {
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				if (x < size / 2) {
					p1.push_back(S->get(c * size * size + y * size + x));
				} else {
					p2.push_back(S->get(c * size * size + y * size + x));
				}
			}
		}
	}

	partition.clear();
	partition.push_back(p1);
	partition.push_back(p2);

	std::vector<unsigned int> r1;
	r1.push_back(0);
	r1.push_back(2);
	r1.push_back(1);

	std::vector<unsigned int> r2;
	r2.push_back(0);
	r2.push_back(1);
	r2.push_back(2);

	ordering.clear();
	ordering.push_back(r1);
	ordering.push_back(r2);
}


void GridLMDP::print(const PolicyMap *policy)
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	unsigned int actionNorth = 0; // North
	unsigned int actionSouth = 1; // South
	unsigned int actionEast = 2; // East
	unsigned int actionWest = 3; // West

	for (int c = 0; c < 2; c++) {
		std::cout << "c = " << c << std::endl;

        for (int x = 0; x < size + 2; x++) {
            std::cout << ". ";
        }
        std::cout << std::endl;

        for (int y = 0; y < size; y++) {
            std::cout << ". ";

            for (int x = 0; x < size; x++) {
//                unsigned int stateHashValue = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c));
                unsigned int stateHashValue = c * size * size + y * size + x;

                /* if (x == 0 && y == 0) {
                    std::cout << "s";
                } else */
                if (x == 0 && y == size - 1 && c == 0) {
                    std::cout << "c";
                } else if (x == size - 1 && y == 0) {
                    std::cout << "-";
                } else if (x == size - 1 && y == size - 1) {
                    std::cout << "+";
                } else if (std::find(blocked.begin(), blocked.end(), stateHashValue) != blocked.end()) {
                    std::cout << "x";
                } else {
                    const Action *action = policy->get(S->get(stateHashValue));

                    if (action->hash_value() == actionNorth) {
                        std::cout << "^";
                    } else if (action->hash_value() == actionSouth) {
                        std::cout << "v";
                    } else if (action->hash_value() == actionEast) {
                        std::cout << ">";
                    } else if (action->hash_value() == actionWest) {
                        std::cout << "<";
                    }
                }

                std::cout << " ";
            }

            std::cout << "." << std::endl;
        }

        for (int x = 0; x < size + 2; x++) {
            std::cout << ". ";
        }
        std::cout << std::endl;
	}
}

void GridLMDP::create_states()
{
	states = new StatesMap();
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	// The grid world's size is determined by the size variable.
	IndexedState::reset_indexer();

	for (int c = 0; c < 2; c++) {
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
//            	std::string name = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c);
//                unsigned int current = NamedState::hash_value(name);

//            	unsigned int current = IndexedState::get_num_states();
//                if (std::find(blocked.begin(), blocked.end(), current) == blocked.end()) {
                    S->add(new IndexedState());
//                }
            }
        }
	}
}

void GridLMDP::create_actions()
{
	actions = new ActionsMap();
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);

	// There will only be four actions to move around the grid world.
	IndexedAction::reset_indexer();
	A->add(new IndexedAction()); // North
	A->add(new IndexedAction()); // South
	A->add(new IndexedAction()); // East
	A->add(new IndexedAction()); // West
}

void GridLMDP::create_state_transitions()
{
	stateTransitions = new StateTransitionsArray(IndexedState::get_num_states(), IndexedAction::get_num_actions());
	StateTransitionsArray *T = dynamic_cast<StateTransitionsArray *>(stateTransitions);

	StatesMap *S = dynamic_cast<StatesMap *>(states);
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);

	unsigned int actionNorth = 0; // North
	unsigned int actionSouth = 1; // South
	unsigned int actionEast = 2; // East
	unsigned int actionWest = 3; // West

	// Loop over all starting states.
	for (int c = 0; c < 2; c++) {
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
                // Create the hash values for the four directions.
                unsigned int current = c * size * size + y * size + x;
                unsigned int north = c * size * size + (y - 1) * size + x;
                unsigned int south = c * size * size + (y + 1) * size + x;
                unsigned int east = c * size * size + y * size + (x + 1);
                unsigned int west = c * size * size + y * size + (x - 1);

                // If this is a blocked cell, then the state does not exist. Skip it.
                if (std::find(blocked.begin(), blocked.end(), current) != blocked.end()) {
                    continue;
                }

                // Only do all the work below if this is not one of the two absorbing state corners (top and bottom right).
                if ((x == size - 1 && y == 0) || (x == size - 1 && y == size - 1)) {
                	for (auto action : *A) {
                		const Action *a = resolve(action);

						T->set(
							S->get(current),
							a,
							S->get(current),
							1.0);
                	}

                    continue;
                }

                // Also, if you 'eat the cookie' then you must transition to the other 'level' of states, following the
                // normal transition probabilities.
                if (x == 0 && y == size - 1 && c == 0) {
                	unsigned int currentNoCookie = 1 * size * size + y * size + x;
                	unsigned int northNoCookie = 1 * size * size + (y - 1) * size + x;
                	unsigned int eastNoCookie = 1 * size * size + y * size + (x + 1);

                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(currentNoCookie),
                        0.8);
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(eastNoCookie),
                        0.1);
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(currentNoCookie),
                        0.1);

                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(eastNoCookie),
                        0.1);
                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(currentNoCookie),
                        0.9);

                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(northNoCookie),
                        0.1);
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(eastNoCookie),
                        0.8);
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(currentNoCookie),
                        0.1);

                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(northNoCookie),
                        0.1);
                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(currentNoCookie),
                        0.9);

                    continue;
                }

                // Handle moving north, taking into account both the edges and the blocked cells.
                double forward = 0.8;
                double left = 0.1;
                double right = 0.1;
                double stuck = 0.0;

                if (y == 0 || std::find(blocked.begin(), blocked.end(), north) != blocked.end()) {
                    stuck += forward;
                    forward = 0.0;
                }
                if (x == 0 || std::find(blocked.begin(), blocked.end(), west) != blocked.end()) {
                    stuck += left;
                    left = 0.0;
                }
                if (x == size - 1 || std::find(blocked.begin(), blocked.end(), east) != blocked.end()) {
                    stuck += right;
                    right = 0.0;
                }

                if (forward > 0.0) { // The hash 'north' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(north),
                        forward);
                }
                if (left > 0.0) { // The hash 'west' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(west),
                        left);
                }
                if (right > 0.0) { // The hash 'east' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(east),
                        right);
                }
                if (stuck > 0.0) {
                    T->set(
                        S->get(current),
                        A->get(actionNorth),
                        S->get(current),
                        stuck);
                }

                // Handle moving south, taking into account both the edges and the blocked cells.
                forward = 0.8;
                left = 0.1;
                right = 0.1;
                stuck = 0.0;

                if (y == size - 1 || std::find(blocked.begin(), blocked.end(), south) != blocked.end()) {
                    stuck += forward;
                    forward = 0.0;
                }
                if (x == 0 || std::find(blocked.begin(), blocked.end(), west) != blocked.end()) {
                    stuck += right;
                    right = 0.0;
                }
                if (x == size - 1 || std::find(blocked.begin(), blocked.end(), east) != blocked.end()) {
                    stuck += left;
                    left = 0.0;
                }

                if (forward > 0.0) { // The hash 'south' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(south),
                        forward);
                }
                if (right > 0.0) { // The hash 'west' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(west),
                        right);
                }
                if (left > 0.0) { // The hash 'east' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(east),
                        left);
                }
                if (stuck > 0.0) {
                    T->set(
                        S->get(current),
                        A->get(actionSouth),
                        S->get(current),
                        stuck);
                }

                // Handle moving east, taking into account both the edges and the blocked cells.
                forward = 0.8;
                left = 0.1;
                right = 0.1;
                stuck = 0.0;

                if (x == size - 1 || std::find(blocked.begin(), blocked.end(), east) != blocked.end()) {
                    stuck += forward;
                    forward = 0.0;
                }
                if (y == 0 || std::find(blocked.begin(), blocked.end(), north) != blocked.end()) {
                    stuck += left;
                    left = 0.0;
                }
                if (y == size - 1 || std::find(blocked.begin(), blocked.end(), south) != blocked.end()) {
                    stuck += right;
                    right = 0.0;
                }

                if (forward > 0.0) { // The hash 'east' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(east),
                        forward);
                }
                if (left > 0.0) { // The hash 'north' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(north),
                        left);
                }
                if (right > 0.0) { // The hash 'south' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(south),
                        right);
                }
                if (stuck > 0.0) {
                    T->set(
                        S->get(current),
                        A->get(actionEast),
                        S->get(current),
                        stuck);
                }

                // Handle moving west, taking into account both the edges and the blocked cells.
                forward = 0.8;
                left = 0.1;
                right = 0.1;
                stuck = 0.0;

                if (x == 0 || std::find(blocked.begin(), blocked.end(), west) != blocked.end()) {
                    stuck += forward;
                    forward = 0.0;
                }
                if (y == 0 || std::find(blocked.begin(), blocked.end(), north) != blocked.end()) {
                    stuck += right;
                    right = 0.0;
                }
                if (y == size - 1 || std::find(blocked.begin(), blocked.end(), south) != blocked.end()) {
                    stuck += left;
                    left = 0.0;
                }

                if (forward > 0.0) { // The hash 'west' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(west),
                        forward);
                }
                if (right > 0.0) { // The hash 'north' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(north),
                        right);
                }
                if (left > 0.0) { // The hash 'south' must be a valid state.
                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(south),
                        left);
                }
                if (stuck > 0.0) {
                    T->set(
                        S->get(current),
                        A->get(actionWest),
                        S->get(current),
                        stuck);
                }
            }
        }
	}
}

void GridLMDP::create_rewards()
{
	rewards = new FactoredRewards();
	FactoredRewards *R = dynamic_cast<FactoredRewards *>(rewards);

	StatesMap *S = dynamic_cast<StatesMap *>(states);
	ActionsMap *A = dynamic_cast<ActionsMap *>(actions);
	StateTransitionsArray *T = dynamic_cast<StateTransitionsArray *>(stateTransitions);

	// Create the primary penalty in the top right corner, an absorbing state.
	SASRewardsArray *primary = new SASRewardsArray(IndexedState::get_num_states(), IndexedAction::get_num_actions());
	R->add_factor(primary);

	// Top right penalty.
	for (auto state : *S) {
		const State *s = resolve(state);

		if (std::find(blocked.begin(), blocked.end(), s->hash_value()) != blocked.end()) {
			continue;
		}

		for (auto action : *A) {
			const Action *a = resolve(action);

			primary->set(s, a,
					S->get(0 + 0 + (size - 1)),
					-1.0);
			primary->set(s, a,
					S->get(1 * size * size + 0 + (size - 1)),
					-1.0);
		}
	}

	// Small penalty for travel. Not for the dead end, since it seems to want to avoid the dead end with all
	// states, meaning that it only gives West and South as actions. Since there's both rewards to the south,
	// all actions are to move south. This only is observed in the case with no obstacles.
//	primary->set(nullptr, nullptr, nullptr, penalty);

	// Zero for absorbing states.
	/*
	for (int c = 0; c < 2; c++) {
		for (auto action : *((ActionsMap *)actions)) {
			const Action *a = resolve(action);

			primary->set(((StatesMap *)states)->get(c * size * size + (size - 1) * size + (size - 1)),
							a,
							((StatesMap *)states)->get(c * size * size + (size - 1) * size + (size - 1)),
							0.0);
			primary->set(((StatesMap *)states)->get(c * size * size + 0 + (size - 1)),
							a,
							((StatesMap *)states)->get(c * size * size + 0 + (size - 1)),
							0.0);
		}
	}
	//*/

	// Create the secondary reward in the bottom right corner, an absorbing state.
	SASRewardsArray *secondary = new SASRewardsArray(IndexedState::get_num_states(), IndexedAction::get_num_actions());
	R->add_factor(secondary);

	// Small penalty for travel.
	for (auto state : *S) {
		const State *s = resolve(state);

		if (std::find(blocked.begin(), blocked.end(), s->hash_value()) != blocked.end()) {
			continue;
		}

		for (auto action : *A) {
			const Action *a = resolve(action);

			for (auto statePrime : *S) {
				const State *sp = resolve(statePrime);

				if (std::find(blocked.begin(), blocked.end(), sp->hash_value()) != blocked.end()) {
					continue;
				}

				secondary->set(s, a, sp, penalty);
			}
		}
	}

	// Bottom right reward.
	for (int c = 0; c < 2; c++) {
		for (auto state : *S) {
			const State *s = resolve(state);

			if (std::find(blocked.begin(), blocked.end(), s->hash_value()) != blocked.end()) {
				continue;
			}

			for (auto action : *A) {
				const Action *a = resolve(action);

				secondary->set(s, a,
						S->get(c * size * size + (size - 1) * size + (size - 1)),
						1.0);
			}
		}
	}

	// Zero for absorbing states.
	for (int c = 0; c < 2; c++) {
		for (auto action : *A) {
			const Action *a = resolve(action);

			secondary->set(S->get(c * size * size + (size - 1) * size + (size - 1)),
							a,
							S->get(c * size * size + (size - 1) * size + (size - 1)),
							0.0);
			secondary->set(S->get(c * size * size + 0 + (size - 1)),
							a,
							S->get(c * size * size + 0 + (size - 1)),
							0.0);
		}
	}

	// Create the tertiary reward in the bottom left corner.
	SASRewardsArray *tertiary = new SASRewardsArray(IndexedState::get_num_states(), IndexedAction::get_num_actions());
	R->add_factor(tertiary);

	// Small penalty for travel.
	for (auto state : *S) {
		const State *s = resolve(state);

		if (std::find(blocked.begin(), blocked.end(), s->hash_value()) != blocked.end()) {
			continue;
		}

		for (auto action : *A) {
			const Action *a = resolve(action);

			for (auto statePrime : *S) {
				const State *sp = resolve(statePrime);

				if (std::find(blocked.begin(), blocked.end(), sp->hash_value()) != blocked.end()) {
					continue;
				}

				tertiary->set(s, a, sp, penalty);
			}
		}
	}

	// Bottom left reward.
	for (auto state : *S) {
		const State *s = resolve(state);

		if (std::find(blocked.begin(), blocked.end(), s->hash_value()) != blocked.end()) {
			continue;
		}

		for (auto action : *A) {
			const Action *a = resolve(action);

			tertiary->set(s, a,
					S->get(0 + (size - 1) * size + 0),
					1.0);
//			tertiary->set(nullptr, nullptr,
//					((FiniteStates *)states)->get(NamedState::hash_value("0 " + std::to_string(size - 1) + " 1")),
//					1.0); // Important, not 0.0 and not -1.0, it should be the default... which is set already below.
		}
	}

	// Zero for absorbing states.
	for (int c = 0; c < 2; c++) {
		for (auto action : *A) {
			const Action *a = resolve(action);

			tertiary->set(S->get(c * size * size + (size - 1) * size + (size - 1)),
							a,
							S->get(c * size * size + (size - 1) * size + (size - 1)),
							0.0);
			tertiary->set(S->get(c * size * size + 0 + (size - 1)),
							a,
							S->get(c * size * size + 0 + (size - 1)),
							0.0);
		}
	}
}

void GridLMDP::create_misc()
{
	StatesMap *S = dynamic_cast<StatesMap *>(states);

	// The initial state is the top left cell.
	initialState = new Initial(S->get(0));

	// Infinite horizon with a discount factor of 0.9.
	horizon = new Horizon(0.9);
}
