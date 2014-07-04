/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Wray
 *  Copyright (c) 2013-2014 Kyle Wray and Luis Pineda
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


#ifndef LVMAX_VALUE_ITERATION_H
#define LVMAX_VALUE_ITERATION_H


#include "../../librbr/librbr/include/mdp/mdp.h"

#include "../../librbr/librbr/include/core/policy/policy_map.h"

#include "../../librbr/librbr/include/core/states/states_map.h"
#include "../../librbr/librbr/include/core/actions/actions_map.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transitions.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

/**
 * Solve an Multi-Objective MDP (MOMDP) via lvmax value iteration (finite or infinite horizon).
 * This solver has the following requirements:
 * - MDP states must be of type FiniteStates.
 * - MDP actions must be of type FiniteActions.
 * - MDP state transitions must be of type FiniteStateTransitions.
 * - MDP rewards must be of type FactoredRewards with SASRewards elements.
 */
class LVMaxValueIteration {
public:
	/**
	 * The default constructor for the LVMaxValueIteration class. The default tolerance is 0.001.
	 */
	LVMaxValueIteration();

	/**
	 * A constructor for the LVMaxValueIteration class which allows for the specification
	 * of the convergence criterion (tolerance).
	 * @param	tolerance		The tolerance which determines convergence of value iteration.
	 */
	LVMaxValueIteration(double tolerance);

	/**
	 * The deconstructor for the LVMaxValueIteration class.
	 */
	virtual ~LVMaxValueIteration();

	/**
	 * Solve the MOMDP provided using lvmax value iteration.
	 * @param	mdp							The Markov decision process to solve.
	 * @param	delta						The tuple of slack variables, one for each reward function.
	 * @param	cuda						If you want to use CUDA.
	 * @throw	StateException				The MDP did not have a FiniteStates states object.
	 * @throw	ActionException				The MDP did not have a FiniteActions actions object.
	 * @throw	StateTransitionsException	The MDP did not have a FiniteStateTransitions state transitions object.
	 * @throw	RewardException				The MDP did not have a FactoredRewards (elements SASRewards) rewards object.
	 * @throw	PolicyException				An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	PolicyMap *solve(const MDP *mdp, const std::vector<double> &delta, bool cuda);

private:
	/**
	 * Solve a finite horizon MOMDP using value iteration.
	 * @param	S					The finite states.
	 * @param	A					The finite actions.
	 * @param	T					The finite state transition function.
	 * @param	R					The factored state-action-state rewards.
	 * @param	h					The horizon.
	 * @param	delta				The slack vector.
	 * @throw	PolicyException		An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	PolicyMap *solve_finite_horizon(const StatesMap *S, const ActionsMap *A,
			const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
			const std::vector<double> &delta);

	/**
	 * Solve an infinite horizon MOMDP using value iteration and leveraging CUDA.
	 * @param	S					The finite states.
	 * @param	A					The finite actions.
	 * @param	T					The finite state transition function, as an array.
	 * @param	R					The factored state-action-state rewards, with each as an array.
	 * @param	h					The horizon.
	 * @param	delta				The slack vector.
	 * @throw	PolicyException		An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	PolicyMap *solve_infinite_horizon_cuda(const StatesMap *S, const ActionsMap *A,
			const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
			const std::vector<double> &delta);

	/**
	 * Solve an infinite horizon MOMDP using value iteration.
	 * @param	S					The finite states.
	 * @param	A					The finite actions.
	 * @param	T					The finite state transition function.
	 * @param	R					The factored state-action-state rewards.
	 * @param	h					The horizon.
	 * @param	delta				The slack vector.
	 * @throw	PolicyException		An error occurred computing the policy.
	 * @return	Return the optimal policy.
	 */
	PolicyMap *solve_infinite_horizon(const StatesMap *S, const ActionsMap *A,
			const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
			const std::vector<double> &delta);

	/**
	 * Compute A_{i+1}^t given that the value function for i, V_i^t, has NOT yet converged.
	 * @param	S 		The finite states.
	 * @param	Ai		The set of actions, which are likely pruned. This will be updated for A_{i+1}.
	 * @param	T 		The finite state transition function.
	 * @param	Ri 		The state-action-state rewards.
	 * @param	h 		The horizon.
	 * @param	s 		The current state being examined, i.e., V_i(s).
	 * @param	Vi		The i-th value function.
	 * @param	AiPlus1	The new set of actions for i + 1.
	 */
	void compute_A_argmax(const StatesMap *S, const std::vector<const Action *> &Ai,
			const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
			const State *s, const std::unordered_map<const State *, double> &Vi,
			std::vector<const Action *> &AiPlus1);

	/**
	 * Compute A_{i+1}^t given that the value function for i, V_i^*, has already converged.
	 * @param	S 		The finite states.
	 * @param	Ai		The set of actions, which are likely pruned. This will be updated for A_{i+1}.
	 * @param	T 		The finite state transition function.
	 * @param	Ri 		The state-action-state rewards.
	 * @param	h 		The horizon.
	 * @param	s 		The current state being examined, i.e., V_i(s).
	 * @param	Vi		The i-th value function.
	 * @param	deltai	The slack value for i in K.
	 * @param	AiPlus1	The new set of actions for i + 1.
	 */
	void compute_A_delta(const StatesMap *S, const std::vector<const Action *> &Ai,
			const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
			const State *s, const std::unordered_map<const State *, double> &Vi,
			double deltai, std::vector<const Action *> &AiPlus1);

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
	 * @param	S		The finite set of states.
	 * @param	T		The finite state transition function.
	 * @param	Ri		The i-th state-action-state reward function.
	 * @param	h		The horizon.
	 * @param	s		The current state.
	 * @param	a		The action taken at the current state.
	 * @param	Vi		The i-th value function.
	 * @return	Returns the Q_i(s, a) value.
	 */
	double compute_Q(const StatesMap *S, const StateTransitions *T, const SASRewards *Ri,
			const Horizon *h, const State *s, const Action *a,
			const std::unordered_map<const State *, double> &Vi);

	/**
	 * The tolerance convergence criterion.
	 */
	double epsilon;

};


#endif // LVMAX_VALUE_ITERATION_H
