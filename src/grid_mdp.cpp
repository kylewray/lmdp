/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Wray
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

#include "../include/grid_mdp.h"

#include "../../librbr/librbr/include/core/states/finite_states.h"
#include "../../librbr/librbr/include/core/actions/finite_actions.h"
#include "../../librbr/librbr/include/core/state_transitions/finite_state_transitions.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards_map.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include "../../librbr/librbr/include/core/states/named_state.h"
#include "../../librbr/librbr/include/core/actions/named_action.h"

#include <iostream>
#include <random>

GridMDP::GridMDP(unsigned int seed, unsigned int gridSize, unsigned int numBlockedStates, double tertiaryPenalty)
{
	std::srand(seed);

	size = gridSize;
	for (int i = 0; i < numBlockedStates; i++) {
		std::string blockedStateName = std::to_string((int)((float)rand() / (float)RAND_MAX * (float)(size - 2)) + 1)
										+ " "
										+ std::to_string((int)((float)rand() / (float)RAND_MAX * (float)(size - 2)) + 1);
		blocked.push_back(NamedState::hash_value(blockedStateName + " 0"));
		blocked.push_back(NamedState::hash_value(blockedStateName + " 1"));
	}
	penalty = tertiaryPenalty;

	create_states();
	create_actions();
	create_state_transitions();
	create_rewards();
	create_misc();
}

GridMDP::~GridMDP()
{ }

void GridMDP::print(const PolicyMap *policy)
{
	unsigned int actionNorth = NamedAction::hash_value("North");
	unsigned int actionSouth = NamedAction::hash_value("South");
	unsigned int actionEast = NamedAction::hash_value("East");
	unsigned int actionWest = NamedAction::hash_value("West");

	for (int c = 0; c <= 1; c++) {
		std::cout << "c = " << c << std::endl;

        for (int x = 0; x < size + 2; x++) {
            std::cout << ". ";
        }
        std::cout << std::endl;

        for (int y = 0; y < size; y++) {
            std::cout << ". ";

            for (int x = 0; x < size; x++) {
                unsigned int stateHashValue = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c));

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
                    const Action *action = policy->get(((FiniteStates *)states)->get(stateHashValue));

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

void GridMDP::create_states()
{
	states = new FiniteStates();

	// The grid world's size is determined by the size variable.
	for (int c = 0; c <= 1; c++) {
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
            	std::string name = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c);
                unsigned int current = NamedState::hash_value(name);

                if (std::find(blocked.begin(), blocked.end(), current) == blocked.end()) {
                    ((FiniteStates *)states)->add(new NamedState(name));
                }
            }
        }
	}
}

void GridMDP::create_actions()
{
	actions = new FiniteActions();

	// There will only be four actions to move around the grid world.
	((FiniteActions *)actions)->add(new NamedAction("North"));
	((FiniteActions *)actions)->add(new NamedAction("South"));
	((FiniteActions *)actions)->add(new NamedAction("East"));
	((FiniteActions *)actions)->add(new NamedAction("West"));
}

void GridMDP::create_state_transitions()
{
	stateTransitions = new FiniteStateTransitions();

	// Loop over all starting states.
	for (int c = 0; c <= 1; c++) {
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                // Create the hash values for the four directions.
                unsigned int current = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c));
                unsigned int north = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y - 1) + " " + std::to_string(c));
                unsigned int south = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y + 1) + " " + std::to_string(c));
                unsigned int east = NamedState::hash_value(std::to_string(x + 1) + " " + std::to_string(y) + " " + std::to_string(c));
                unsigned int west = NamedState::hash_value(std::to_string(x - 1) + " " + std::to_string(y) + " " + std::to_string(c));

                // If this is a blocked cell, then the state does not exist. Skip it.
                if (std::find(blocked.begin(), blocked.end(), current) != blocked.end()) {
                    continue;
                }

                // Only do all the work below if this is not one of the two absorbing state corners (top and bottom right).
                if ((x == size - 1 && y == 0) || (x == size - 1 && y == size - 1)) {
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        nullptr,
                        ((FiniteStates *)states)->get(current),
                        1.0);
                    continue;
                }

                // Also, if you 'eat the cookie' then you must transition to the other 'level' of states, following the
                // normal transition probabilities.
                if (x == 0 && y == size - 1 && c == 0) {
                	unsigned int currentNoCookie = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " 1");
                	unsigned int northNoCookie = NamedState::hash_value(std::to_string(x) + " " + std::to_string(y - 1) + " 1");
                	unsigned int eastNoCookie= NamedState::hash_value(std::to_string(x + 1) + " " + std::to_string(y) + " 1");

                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(currentNoCookie),
                        0.8);

                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(eastNoCookie),
                        0.1);
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(currentNoCookie),
                        0.1);

                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(eastNoCookie),
                        0.1);
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(currentNoCookie),
                        0.9);

                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(currentNoCookie),
                        0.1);
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(eastNoCookie),
                        0.8);
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(currentNoCookie),
                        0.1);

                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(northNoCookie),
                        0.1);
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(currentNoCookie),
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
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(north),
                        forward);
                }
                if (left > 0.0) { // The hash 'west' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(west),
                        left);
                }
                if (right > 0.0) { // The hash 'east' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(east),
                        right);
                }
                if (stuck > 0.0) {
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("North")),
                        ((FiniteStates *)states)->get(current),
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
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(south),
                        forward);
                }
                if (right > 0.0) { // The hash 'west' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(west),
                        right);
                }
                if (left > 0.0) { // The hash 'east' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(east),
                        left);
                }
                if (stuck > 0.0) {
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("South")),
                        ((FiniteStates *)states)->get(current),
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
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(east),
                        forward);
                }
                if (left > 0.0) { // The hash 'north' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(north),
                        left);
                }
                if (right > 0.0) { // The hash 'south' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(south),
                        right);
                }
                if (stuck > 0.0) {
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("East")),
                        ((FiniteStates *)states)->get(current),
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
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(west),
                        forward);
                }
                if (right > 0.0) { // The hash 'north' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(north),
                        right);
                }
                if (left > 0.0) { // The hash 'south' must be a valid state.
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(south),
                        left);
                }
                if (stuck > 0.0) {
                    ((FiniteStateTransitions *)stateTransitions)->set(
                        ((FiniteStates *)states)->get(current),
                        ((FiniteActions *)actions)->get(NamedAction::hash_value("West")),
                        ((FiniteStates *)states)->get(current),
                        stuck);
                }
            }
        }
	}
}

void GridMDP::create_rewards()
{
	rewards = new FactoredRewards();

	// Create the primary  penalty in the top right corner, an absorbing state.
	SASRewardsMap *primary = new SASRewardsMap();
	((FactoredRewards *)rewards)->add_factor(primary);

	// Top right penalty.
	primary->set(nullptr, nullptr,
			((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
			-1.0);
	primary->set(nullptr, nullptr,
			((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
			-1.0);

	// Small penalty for travel. Not for the dead end, since it seems to want to avoid the dead end with all
	// states, meaning that it only gives West and South as actions. Since there's both rewards to the south,
	// all actions are to move south. This only is observed in the case with no obstacles.
//	primary->set(nullptr, nullptr, nullptr, penalty);

	// Zero for absorbing states.
	primary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					0.0);
	primary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					0.0);
	primary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					0.0);
	primary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					0.0);

	// Create the secondary reward in the bottom left corner, an absorbing state.
	SASRewardsMap *secondary = new SASRewardsMap();
	((FactoredRewards *)rewards)->add_factor(secondary);

	// Bottom right reward.
	secondary->set(nullptr, nullptr,
			((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
			1.0);
	secondary->set(nullptr, nullptr,
			((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
			1.0);

	// Small penalty for travel.
	secondary->set(nullptr, nullptr, nullptr, penalty);

	// Zero for absorbing states.
	secondary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					0.0);
	secondary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					0.0);
	secondary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					0.0);
	secondary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					0.0);

	//*
	// Create the tertiary reward in the bottom left corner.
	SASRewardsMap *tertiary = new SASRewardsMap();
	((FactoredRewards *)rewards)->add_factor(tertiary);

	// Bottom left reward.
	tertiary->set(nullptr, nullptr,
			((FiniteStates *)states)->get(NamedState::hash_value("0 " + std::to_string(size - 1) + " 0")),
			1.0);
//	tertiary->set(nullptr, nullptr,
//			((FiniteStates *)states)->get(NamedState::hash_value("0 " + std::to_string(size - 1) + " 1")),
//			1.0); // Important, not 0.0 and not -1.0, it should be the default... which is set already below.

	// Small penalty for travel.
	tertiary->set(nullptr, nullptr, nullptr, penalty);

	// Zero for absorbing states.
	tertiary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 0")),
					0.0);
	tertiary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 0")),
					0.0);
	tertiary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " " + std::to_string(size - 1) + " 1")),
					0.0);
	tertiary->set(((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					nullptr,
					((FiniteStates *)states)->get(NamedState::hash_value(std::to_string(size - 1) + " 0 1")),
					0.0);
	//*/
}

void GridMDP::create_misc()
{
	// The initial state is the top left cell.
	initialState = new Initial(((FiniteStates *)states)->get(NamedState::hash_value("0 0 0")));

	// Infinite horizon with a discount factor of 0.9.
	horizon = new Horizon(0.9);
}
