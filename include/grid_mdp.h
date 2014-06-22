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


#ifndef GRID_MDP_H
#define GRID_MDP_H


#include "../../librbr/librbr/include/mdp/mdp.h"
#include "../../librbr/librbr/include/core/policy/policy_map.h"

#include "../../librbr/librbr/include/core/states/state.h"
#include "../../librbr/librbr/include/core/actions/action.h"

#include <vector>
#include <string>

/**
 * A simple grid world Multi-Objective Markov Decision Process (MOMDP) with lexicographic reward preferences.
 */
class GridMDP : public MDP {
public:
	/**
	 * The default constructor for the GridMDP class.
	 * @param	seed				The random seed to use.
	 * @param	gridSize			The size of the grid world.
	 * @param	numBlockedStates	The number of blocked states.
	 * @param	tertiaryPenalty		The small penalty for the tertiary reward.
	 */
	GridMDP(unsigned int seed, unsigned int gridSize, unsigned int numBlockedStates, double tertiaryPenalty);

	/**
	 * The default deconstructor of the LOSMState class.
	 */
	virtual ~GridMDP();

	/**
	 * Given a policy map, this outputs the pretty picture of the grid world and the policy.
	 * @param	policy		The policy to print along with the grid world.
	 */
	void print(const PolicyMap *policy);

private:
	/**
	 * Create the MDP's states.
	 */
	void create_states();

	/**
	 * Create the MDP's actions.
	 */
	void create_actions();

	/**
	 * Create the MDP's state transitions.
	 */
	void create_state_transitions();

	/**
	 * Create the MDP's rewards.
	 */
	void create_rewards();

	/**
	 * Create the MDP's initial and horizon objects.
	 */
	void create_misc();

	/**
	 * The size of the grid world.
	 */
	unsigned int size;

	/**
	 * The blocked states in their hash value form.
	 */
	std::vector<unsigned int> blocked;

	/**
	 * The small penalty for the tertiary reward.
	 */
	double penalty;

};


#endif // GRID_MDP_H
