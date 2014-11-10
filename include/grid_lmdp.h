/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Hollins Wray, University of Massachusetts
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


#ifndef GRID_LMDP_H
#define GRID_LMDP_H


#include "lmdp.h"

#include "../../librbr/librbr/include/core/policy/policy_map.h"

#include "../../librbr/librbr/include/core/states/state.h"
#include "../../librbr/librbr/include/core/actions/action.h"

#include <vector>
#include <string>

/**
 * A simple grid world LMDP.
 */
class GridLMDP : public LMDP {
public:
	/**
	 * The default constructor for the GridLMDP class.
	 * @param	seed				The random seed to use.
	 * @param	gridSize			The size of the grid world.
	 * @param	numBlockedStates	The number of blocked states.
	 * @param	tertiaryPenalty		The small penalty for the tertiary reward.
	 */
	GridLMDP(unsigned int seed, unsigned int gridSize, unsigned int numBlockedStates,
			double tertiaryPenalty);

	/**
	 * The default deconstructor of the GridLMDP class.
	 */
	virtual ~GridLMDP();

	/**
	 * Set the delta values.
	 * @param	d1	The first delta.
	 * @param	d2	The second delta.
	 * @param	d3	The third delta.
	 */
	void set_slack(float d1, float d2, float d3);

	/**
	 * Define a 1-partition with the ordering (1, 2, 3) for all states.
	 */
	void set_default_conditional_preference();

	/**
	 * Define a 2-partition with the ordering (1, 2, 3) for all northern states,
	 * and the ordering (1, 3, 2) for all southern states.
	 */
	void set_split_conditional_preference();

	/**
	 * Given a policy map, this outputs the pretty picture of the grid world and the policy.
	 * @param	policy		The policy to print along with the grid world.
	 */
	void print(PolicyMap *policy);

	/**
	 * Set the weights for the factored weighted rewards.
	 * @param	weights		The new weight vector.
	 */
	void set_rewards_weights(const std::vector<double> &weights);

	/**
	 * Get the weights for the factored weighted rewards.
	 * @return	The weight vector.
	 */
	const std::vector<double> &get_rewards_weights() const;

private:
	/**
	 * Create the LMDP's states.
	 */
	void create_states();

	/**
	 * Create the LMDP's actions.
	 */
	void create_actions();

	/**
	 * Create the LMDP's state transitions.
	 */
	void create_state_transitions();

	/**
	 * Create the LMDP's rewards.
	 */
	void create_rewards();

	/**
	 * Create the LMDP's initial and horizon objects.
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


#endif // GRID_LMDP_H
