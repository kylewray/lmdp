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


#ifndef L_MDP_H
#define L_MDP_H


#include "../../librbr/librbr/include/mdp/mdp.h"

#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"

/**
 * A MOMDP with conditional lexicographic reward preferences which allows for slack.
 */
class LMDP : public MDP {
public:
	/**
	 * The default constructor for the LMDP class.
	 */
	LMDP();

	/**
	 * A constructor for the LMDP class.
	 * @param	S		The states.
	 * @param	A		The actions.
	 * @param	T		The state transitions, which uses the states and actions parameters.
	 * @param	R		The k-factored rewards, which uses the states and actions parameters.
	 * @param	s		The initial state, which uses the states parameter.
	 * @param	h		The horizon.
	 * @param	d		The slack as a k-array; each element must be non-negative.
	 * @param	P		The z-partition over states.
	 * @param	o		The z-array of orderings over each of the k rewards.
	 */
	LMDP(States *S, Actions *A, StateTransitions *T, FactoredRewards *R, Initial *s, Horizon *h,
			std::vector<float> *d, std::vector<std::vector<const State *> > *P,
			std::vector<std::vector<unsigned int> > *o);

	/**
	 * The default deconstructor for the LDMP class.
	 */
	virtual ~LMDP();

	/**
	 * Get the factored rewards. This is an overloaded method, allowing for explicit
	 * return of a FactoredRewards object.
	 * @return	The factored rewards.
	 */
	const FactoredRewards *get_rewards() const;

	/**
	 * Set the slack.
	 * @param	d	The new slack.
	 */
	void set_slack(const std::vector<float> &d);

	/**
	 * Get the slack.
	 * @return	The slack vector.
	 */
	const std::vector<float> &get_slack() const;

	/**
	 * Set the partitions.
	 * @param	P	The new partitions.
	 */
	void set_partitions(const std::vector<std::vector<const State *> > &P);

	/**
	 * Get the partition over states.
	 * @return	The partition vector.
	 */
	const std::vector<std::vector<const State *> > &get_partitions() const;

	/**
	 * Set the orderings.
	 * @param	o	The new orderings.
	 */
	void set_orderings(const std::vector<std::vector<unsigned int> > &o);

	/**
	 * Get the orderings for each partition.
	 * @return	The orderings for each partition.
	 */
	const std::vector<std::vector<unsigned int> > &get_orderings() const;

protected:
	/**
	 * The slack as a k-array; each element must be non-negative.
	 */
	std::vector<float> delta;

	/**
	 * The z-partition over states. This is a parallel vector to ordering.
	 */
	std::vector<std::vector<const State *> > partition;

	/**
	 * The z-array of orderings over each of the k rewards. This is a parallel
	 * vector to partition.
	 */
	std::vector<std::vector<unsigned int> > ordering;

};


#endif // L_MDP_H
