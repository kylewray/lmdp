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


#ifndef LVI_H
#define LVI_H


#include "lmdp.h"

#include "../../librbr/librbr/include/core/policy/policy_map.h"

#include "../../librbr/librbr/include/core/states/states_map.h"
#include "../../librbr/librbr/include/core/actions/actions_map.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transitions.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include <unordered_map>

/**
 * Solve a Lexicographic Markov Decision Process (LMDP).
 */
class LVI {
public:
	/**
	 * The default constructor for the LVI class. The default tolerance is 0.001.
	 */
	LVI();

	/**
	 * A constructor for the LVI class which allows for the specification
	 * of the convergence criterion (tolerance).
	 * @param	tolerance		The tolerance which determines convergence of value iteration.
	 * @param	enableLooping	If this should be the looping version of LVI.
	 */
	LVI(double tolerance, bool loopingVersion);

	/**
	 * The deconstructor for the LVI class.
	 */
	virtual ~LVI();

	/**
	 * Solve the LMDP provided using lexicographic value iteration.
	 * @param	lmdp						The LMDP to solve.
	 * @throw	StateException				The LMDP did not have a StatesMap states object.
	 * @throw	ActionException				The LMDP did not have a ActionsMap actions object.
	 * @throw	StateTransitionsException	The LMDP did not have a StateTransitions state transitions object.
	 * @throw	RewardException				The LMDP did not have a FactoredRewards (elements SASRewards) rewards object.
	 * @throw	CoreException				The LMDP was not infinite horizon.
	 * @throw	PolicyException				An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	PolicyMap *solve(const LMDP *lmdp);

	/**
	 * Get the values of the states.
	 * @return	The values of all the states.
	 */
	const std::vector<std::unordered_map<const State *, double> > &get_V() const;

protected:
	/**
	 * Solve an infinite horizon LMDP using value iteration.
	 * @param	S					The finite states.
	 * @param	A					The finite actions.
	 * @param	T					The finite state transition function.
	 * @param	R					The factored state-action-state rewards.
	 * @param	h					The horizon.
	 * @param	delta				The slack vector.
	 * @param	P					The vector of partitions.
	 * @param	o					The vector of orderings.
	 * @throw	PolicyException		An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	virtual PolicyMap *solve_infinite_horizon(const StatesMap *S, const ActionsMap *A,
			const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
			const std::vector<float> &delta,
			const std::vector<std::vector<const State *> > &P,
			const std::vector<std::vector<unsigned int> > &o);

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

	/**
	 * Compute A_{i+1}^t given that the value function for i, V_i^t, has NOT yet converged.
	 * @param	S 		The finite states.
	 * @param	Ai		The set of actions, which are likely pruned.
	 * @param	T 		The finite state transition function.
	 * @param	Ri 		The state-action-state rewards.
	 * @param	h 		The horizon.
	 * @param	s 		The current state being examined, i.e., V_i(s).
	 * @param	Vi		The i-th value function.
	 * @param	AiPlus1	The new set of actions for i + 1. This will be updated.
	 */
	void compute_A_argmax(const StatesMap *S, const std::vector<const Action *> &Ai,
			const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
			const State *s, const std::unordered_map<const State *, double> &Vi,
			std::vector<const Action *> &AiPlus1);

	/**
	 * Compute A_{i+1}^t given that the value function for i, V_i^*, has already converged.
	 * @param	S 		The finite states.
	 * @param	Ai		The set of actions, which are likely pruned.
	 * @param	T 		The finite state transition function.
	 * @param	Ri 		The state-action-state rewards.
	 * @param	h 		The horizon.
	 * @param	s 		The current state being examined, i.e., V_i(s).
	 * @param	Vi		The i-th value function.
	 * @param	deltai	The slack value for i in K.
	 * @param	AiPlus1	The new set of actions for i + 1. This will be updated.
	 */
	void compute_A_delta(const StatesMap *S, const std::vector<const Action *> &Ai,
			const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
			const State *s, const std::unordered_map<const State *, double> &Vi,
			float deltai,
			std::vector<const Action *> &AiPlus1);

	/**
	 * Compute V_i^{t+1} given that the value function for i, V_i^t.
	 * @param	S 		The finite states.
	 * @param	Ai		The set of actions, which are likely pruned.
	 * @param	T 		The finite state transition function.
	 * @param	Ri 		The state-action-state rewards.
	 * @param	h 		The horizon.
	 * @param	s 		The current state being examined, i.e., V_i(s).
	 * @param	Vi		The i-th value function at time t.
	 * @param	ViNext	The i-th value function at time t+1. This will be updated.
	 * @param	a		The action taken to obtain the max value. This will be updated.
	 */
	void compute_V(const StatesMap *S, const std::vector<const Action *> &Ai,
			const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
			const State *s, const std::unordered_map<const State *, double> &Vi,
			std::unordered_map<const State *, double> &ViNext, const Action *&a);

	/**
	 * Compute the value of Q_i(s, a) for some state and action.
	 * @param	S					The finite set of states.
	 * @param	T					The finite state transition function.
	 * @param	Ri					The i-th state-action-state reward function.
	 * @param	h					The horizon.
	 * @param	s					The current state.
	 * @param	a					The action taken at the current state.
	 * @param	Vi					The i-th value function.
	 * @throw	PolicyException		There was an undefined value of a state.
	 * @return	Returns the Q_i(s, a) value.
	 */
	double compute_Q(const StatesMap *S, const StateTransitions *T, const SASRewards *Ri,
			const Horizon *h, const State *s, const Action *a,
			const std::unordered_map<const State *, double> &Vi);

	/**
	 * The value of the states, one for each reward.
	 */
	std::vector<std::unordered_map<const State *, double> > V;

	/**
	 * The tolerance convergence criterion.
	 */
	double epsilon;

	/**
	 * If this is the looping version or not.
	 */
	bool loopingVersion;

};


#endif // LVI_H
