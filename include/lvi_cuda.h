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


#ifndef LVI_CUDA_H
#define LVI_CUDA_H


#include "lvi.h"

/**
 * Solve a Lexicographic Markov Decision Process (LMDP) with Cuda. This is always the
 * so-called 'loopingVersion'.
 */
class LVICuda : public LVI {
public:
	/**
	 * The default constructor for the LVINova class. The default tolerance is 0.001.
	 */
	LVICuda();

	/**
	 * A constructor for the LVINova class which allows for the specification
	 * of the convergence criterion (tolerance).
	 * @param	tolerance		The tolerance which determines convergence of value iteration.
	 */
	LVICuda(double tolerance);

	/**
	 * The deconstructor for the LVINova class.
	 */
	virtual ~LVICuda();

protected:
	/**
	 * Solve the infinite horizon MDP for a particular partition of the state space.
	 * @param	S					The finite states.
	 * @param	A					The finite actions.
	 * @param	T					The finite state transition function.
	 * @param	R					The factored state-action-state rewards.
	 * @param	h					The horizon.
	 * @param	delta				The slack vector.
	 * @param	Pj					The z-partition over states.
	 * @param	oj					The z-array of orderings over each of the k rewards.
	 * @param	VFixed				The fixed set of value functions from the previous outer step.
	 * @param	V					The resultant value of the states. This is updated.
	 * @param	policy				The policy for the states in the partition. This is updated.
	 * @param	maxDifference		The maximal difference for convergence checking. This is updated.
	 * @throw	PolicyException		An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	virtual void compute_partition(const StatesMap *S, const ActionsMap *A, const StateTransitions *T,
			const FactoredRewards *R, const Initial *s0, const Horizon *h, const std::vector<float> &delta,
			const std::vector<const State *> &Pj, const std::vector<unsigned int> &oj,
			const std::vector<std::unordered_map<const State *, double> > &VFixed,
			std::vector<std::unordered_map<const State *, double> > &V,
			PolicyMap *policy, std::vector<double> &maxDifference);

};


#endif // LVI_CUDA_H
