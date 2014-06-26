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


#include "../include/lvmax_value_iteration.h"
#include "../../librbr/librbr/include/mdp/mdp_utilities.h"

#include "../../librbr/librbr/include/core/states/state_exception.h"
#include "../../librbr/librbr/include/core/actions/action_exception.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transition_exception.h"
#include "../../librbr/librbr/include/core/rewards/reward_exception.h"
#include "../../librbr/librbr/include/core/policy/policy_exception.h"

#include <unordered_map>
#include <math.h>

#include <iostream> // NOTE: REMOVE ME AT SOME POINT!
#include "../../librbr/librbr/include/core/states/named_state.h" // NOTE: REMOVE ME AT SOME POINT!

LVMaxValueIteration::LVMaxValueIteration()
{
	epsilon = 0.001;
}

LVMaxValueIteration::LVMaxValueIteration(double tolerance)
{
	epsilon = tolerance;
}

LVMaxValueIteration::~LVMaxValueIteration()
{ }

PolicyMap *LVMaxValueIteration::solve(const MDP *mdp, const std::vector<double> &delta)
{
	// Handle the trivial case.
	if (mdp == nullptr) {
		return nullptr;
	}

	// Attempt to convert the states object into FiniteStates.
	const FiniteStates *S = dynamic_cast<const FiniteStates *>(mdp->get_states());
	if (S == nullptr) {
		throw StateException();
	}

	// Attempt to convert the actions object into FiniteActions.
	const FiniteActions *A = dynamic_cast<const FiniteActions *>(mdp->get_actions());
	if (A == nullptr) {
		throw ActionException();
	}

	// Attempt to convert the state transitions object into FiniteStateTransitions.
	const FiniteStateTransitions *T =
			dynamic_cast<const FiniteStateTransitions *>(mdp->get_state_transitions());
	if (T == nullptr) {
		throw StateTransitionException();
	}

	// Attempt to convert the rewards object into FactoredRewards. Also, ensure that the
	// type of each element is SASRewards.
	const FactoredRewards *R = dynamic_cast<const FactoredRewards *>(mdp->get_rewards());
	if (R == nullptr) {
		throw RewardException();
	}
	for (int i = 0; i < R->get_num_rewards(); i++) {
		const SASRewards *Ri = dynamic_cast<const SASRewards *>(R->get(i));
		if (Ri == nullptr) {
			throw RewardException();
		}
	}

	// Handle the other trivial case in which the slack variables were incorrectly defined.
	if (delta.size() != R->get_num_rewards()) {
		throw RewardException();
	}
	for (int i = 0; i < delta.size(); i++) {
		if (delta[i] < 0.0) {
			throw RewardException();
		}
	}

	// Obtain the horizon and return the correct value iteration.
	const Horizon *h = mdp->get_horizon();
	if (h->is_finite()) {
		return solve_finite_horizon(S, A, T, R, h, delta);
	} else {
		return solve_infinite_horizon(S, A, T, R, h, delta);
	}
}

PolicyMap *LVMaxValueIteration::solve_finite_horizon(const FiniteStates *S, const FiniteActions *A,
		const FiniteStateTransitions *T, const FactoredRewards *R, const Horizon *h,
		const std::vector<double> &delta)
{
	return nullptr;
}

PolicyMap *LVMaxValueIteration::solve_infinite_horizon(const FiniteStates *S, const FiniteActions *A,
		const FiniteStateTransitions *T, const FactoredRewards *R, const Horizon *h,
		const std::vector<double> &delta)
{
	// Create the policy based on the horizon.
	PolicyMap *policy = new PolicyMap(h);

	// The value of the states, one for each reward.
	std::vector<std::unordered_map<const State *, double> > V;
	V.resize(R->get_num_rewards());

	// We will want to remember the optimal possible value of a state, too.
	std::vector<std::unordered_map<const State *, double> > VStar;
	VStar.resize(R->get_num_rewards());

	// Compute the convergence criterion which follows from the proof of convergence.
	double convergenceCriterion = epsilon * (1.0 - h->get_discount_factor()) / h->get_discount_factor();

	// Remember the set of actions available to each of the value functions. This will be computed at the end of each step.
	std::vector<std::unordered_map<const State *, std::vector<const Action *> > > AStar;
	AStar.resize(R->get_num_rewards() + 1);

	// Note: We will store the final value function's reduced set of actions, too.

	// Setup the initial set of actions for i = 1.
	for (auto state : *S) {
		const State *s = resolve(state);
		for (auto action : *A) {
			const Action *a = resolve(action);
			AStar[0][s].push_back(a);
		}
	}

	// For each of the value functions, we will compute the actions set.
	for (int i = 0; i < R->get_num_rewards(); i++) {
		const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));

		// Setup V[i] with initial values of 0.0.
		for (auto state : *S) {
			const State *s = resolve(state);
			V[i][s] = 0.0;
		}

		double difference = convergenceCriterion + 1.0;

		// For this V_i, converge until you reach within epsilon of V_i^*.
		while (difference > convergenceCriterion) {
			std::cout << "Value Iteration: Convergence Check: " << difference << " vs " << convergenceCriterion << std::endl;

			difference = 0.0;

			// Copy the value of all states for this i. The constant type casting forces it to use the copy
			// assignment instead of the move one.
			std::unordered_map<const State *, double> Vi;
			for (auto state : *S) {
				const State *s = resolve(state);
				Vi[s] = V[i][s];
			}

			// For all the states, compute V_i(s).
			for (auto state : *S) {
				const State *s = resolve(state);
				const Action *a = nullptr;

				// Update V according to the previously converged subset of actions.
				compute_V(S, AStar[i][s], T, Ri, h, s, V[i], Vi, a);

				// Store the action taken as part of the policy. This will change all the time, especially over i, but whatever.
				policy->set(s, a);

				// Continue to compute the infinity normed difference between value functions for convergence checking.
				if (std::fabs(V[i][s] - Vi[s]) > difference) {
					difference = std::fabs(V[i][s] - Vi[s]);
				}
			}

			// After iterating over states, update the real V[i] for all s.
			for (auto state : *S) {
				const State *s = resolve(state);
				V[i][s] = Vi[s];
			}
		}

		// After everything, we can finally compute the set of actions ***for i + 1*** with the delta slack.
		for (auto state : *S) {
			const State *s = resolve(state);

			// Use the delta function to compute the final set of AStar[i + 1].
			compute_A_delta(S, AStar[i][s], T, Ri, h, s, V[i], delta[i], AStar[i + 1][s]);
		}
	}

	// At the very end, compute the final values of V following the policy.
	for (int i = 0; i < R->get_num_rewards(); i++) {
		const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));

		for (auto state : *S) {
			const State *s = resolve(state);
			V[i][s] = 0.0;
		}

		double difference = convergenceCriterion + 1.0;

		// For this V_i, converge until you reach within epsilon of V_i^*.
		while (difference > convergenceCriterion) {
			std::cout << "Computing Final Values: Convergence Check: " << difference << " vs " << convergenceCriterion << std::endl;

			difference = 0.0;

			// For all the states, compute V_i(s).
			for (auto state : *S) {
				const State *s = resolve(state);

				double Vis = compute_Q(S, T, Ri, h, s, policy->get(s), V[i]);

				if (std::fabs(V[i][s] - Vis) > difference) {
					difference = std::fabs(V[i][s] - Vis);
				}

				V[i][s] = Vis;
			}
		}
	}

	// Output the pretty values in a table format for the GridMDP object!
    int size = 8;

	for (int i = 0; i < R->get_num_rewards(); i++) {
        std::cout << "V[" << i << "]:" << std::endl;

        for (int c = 0; c <= 1; c++) {
//            std::cout << "c = " << c << std::endl;

            for (int y = 0; y < size; y++) {
                for (int x = 0; x < size; x++) {
                    try {
                        const State *s = S->get(NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c)));
                        std::cout << V[i][s];
                        if (x < size - 1) {
                        	std::cout << ",";
                        }
                    } catch (const StateException &err) {
                        std::cout << "0.0,";
                    }
                }
                std::cout << std::endl;
            }
        }

        std::cout << "\n\n";
	}

	return policy;
}

void LVMaxValueIteration::compute_A_argmax(const FiniteStates *S, const std::vector<const Action *> &Ai,
		const FiniteStateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		std::vector<const Action *> &AiPlus1)
{
	compute_A_delta(S, Ai, T, Ri, h, s, Vi, 0.0, AiPlus1);
}

void LVMaxValueIteration::compute_A_delta(const FiniteStates *S, const std::vector<const Action *> &Ai,
		const FiniteStateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		double deltai, std::vector<const Action *> &AiPlus1)
{
	double maxQisa = -std::numeric_limits<double>::max();
	std::vector<double> Qis;

	// For all the actions, compute max Q_i(s, a) over the current set of actions. Also, record the
	// value of each Q_i(s, a) for all actions a in A.
	for (int i = 0; i < Ai.size(); i++) {
		double Qisa = compute_Q(S, T, Ri, h, s, Ai[i], Vi);
		Qis.push_back(Qisa);

		// Determine the maximum Q_i(s, a) value.
		if (Qisa > maxQisa) {
			maxQisa = Qisa;
		}
	}

	// Compute eta_i.
	double etai = (1.0 - h->get_discount_factor()) * deltai;

	// Compute the new A_{i+1} using the Q-values and current A_i.
	AiPlus1.clear();
	for (int i = 0; i < Ai.size(); i++) {
		// Check if this is difference within eta_i, but account for machine precision issues
		// within 1 order of magnitude.
		if (maxQisa - Qis[i] < etai + std::numeric_limits<double>::epsilon() * 10.0) {
			AiPlus1.push_back(Ai[i]);
		}
	}
}

void LVMaxValueIteration::compute_V(const FiniteStates *S, const std::vector<const Action *> &Ai,
		const FiniteStateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		std::unordered_map<const State *, double> &ViNext, const Action *&a)
{
	// Compute the maximal Q_i(s, a) given the reduced set of actions.
	ViNext[s] = -std::numeric_limits<double>::max();
	a = nullptr;

	// For all the actions, compute max Q_i(s, a) over the current set of actions.
	for (int i = 0; i < Ai.size(); i++) {
		const Action *action = Ai[i];
		double Qisa = compute_Q(S, T, Ri, h, s, action, Vi);
		if (Qisa > ViNext[s]) {
			ViNext[s] = Qisa;
			a = action;
		}
	}
}

double LVMaxValueIteration::compute_Q(const FiniteStates *S, const FiniteStateTransitions *T, const SASRewards *Ri,
		const Horizon *h, const State *s, const Action *a, const std::unordered_map<const State *, double> &Vi)
{
	// Compute the Q_i(s, a) estimate.
	double Qisa = 0.0;

	// Get the list of successor states.
	std::vector<const State *> successors;
	T->successors(S, s, a, successors);

	// For each of the successors, compute the bellman update equation.
	for (const State *sPrime : successors) {
		std::unordered_map<const State *, double>::const_iterator VisIterator = Vi.find(sPrime);
		if (VisIterator == Vi.end()) {
			std::cout << "An error has occurred." << std::endl;
			return 0.0;
		}

		double Vis = VisIterator->second;
		Qisa += T->get(s, a, sPrime) * (Ri->get(s, a, sPrime) + h->get_discount_factor() * Vis);
	}

	return Qisa;
}
