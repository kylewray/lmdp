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
	/*
	// Create the policy based on the horizon.
	PolicyMap *policy = new PolicyMap(h);

	// The value of a states and state's actions.
	std::unordered_map<const State *, double> V;

	// Continue to iterate until the maximum difference between two V[s]'s is less than the tolerance.
	for (int t = h->get_horizon() - 1; t >= 0; t--){
		// For all the states, compute V(s).
		for (auto state : *S) {
			const State *s = resolve(state);
			const Action *aBest = nullptr;

			//bellman_update(S, A, T, R, h, s, V, aBest);

			// Set the policy's action, which will yield the optimal policy at the end.
			policy->set(t, s, aBest);
		}
	}

	return policy;
	*/
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

	/* Method #2
	// The current actions which are locked in place. Initially, this is all actions, but every time
	// one of the value functions converges, it locks the actions to a subset for the next one.
	std::vector<const Action *> currentA;
	for (auto a : *A) {
		currentA.push_back(resolve(a));
	}

	// This will hold the current A_i (without star) for the current index. This is then copied to currentA
	// once that value function has converged.
	std::vector<const Action *> currentIndexActions;
	//*/

	// Compute the convergence criterion which follows from the proof of convergence.
	//* Method #1
	double nu = std::numeric_limits<double>::lowest();
	//*/
	/* Method #2
	std::vector<double> nu;
	for (int i = 0; i < R->get_num_rewards(); i++) {
		nu.push_back(std::numeric_limits<double>::lowest());
	}
	//*/

	for (int i = 0; i < R->get_num_rewards(); i++) {
		const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));
		double tmpNu = h->get_discount_factor() + (1.0 - h->get_discount_factor()) * delta[i] / (Ri->get_max() - Ri->get_min());
		//* Method #1
		if (tmpNu > nu) {
			nu = tmpNu;
		}
		//*/
		/* Method #2
		if (tmpNu > nu[i]) {
			nu[i] = tmpNu;
		}
		//*/
	}

	// Continue to iterate until the maximum difference between two V[s]'s is less than the tolerance.
	//* Method #1
	double convergenceCriterion = epsilon * (1.0 - nu) / nu;
	double infinityNormedDifference = convergenceCriterion + 1.0;
	//*/
	/* Method #2
	double convergenceCriterion = epsilon * (1.0 - nu[0]) / nu[0];
	double infinityNormedDifference = convergenceCriterion + 1.0;
	int currentIndex = 0;
	//*/

	/* Delta Prime Test
	std::vector<double> infinity;
	for (int i = 0; i < R->get_num_rewards(); i++) {
		infinity.push_back(0.0);
	}
	//*/

	//* Method #1
	while (infinityNormedDifference > convergenceCriterion) {
	//*/
	/* Method #2
	while (currentIndex < R->get_num_rewards()) {
	//*/
		std::cout << "lvmax Value Iteration: " << infinityNormedDifference << " vs " << convergenceCriterion << std::endl;

		/* Method #2
		if (infinityNormedDifference < convergenceCriterion) {
			std::cout << "lvmax Value Iteration: Converged for V_" << (currentIndex + 1) << "!!!" << std::endl;

			// Lock the new set of actions.
			currentA = currentIndexActions;

			// Update the index, and possibly the convergence criterion. If we are done, then break.
			currentIndex++;
			if (currentIndex == R->get_num_rewards()) {
				break;
			} else {
				convergenceCriterion = epsilon * (1.0 - nu[currentIndex]) / nu[currentIndex];
			}
		}
		//*/

		infinityNormedDifference = 0.0;

		/* Delta Prime Test
		for (int i = 0; i < R->get_num_rewards(); i++) {
			infinity[i] = 0.0;
		}
		//*/

		// For all the states, compute V(s).
		for (auto state : *S) {
			const State *s = resolve(state);

			// Setup the temporary value of this state for each i in K.
			std::vector<std::unordered_map<const State *, double> > Vtmp;
			for (int i = 0; i < R->get_num_rewards(); i++) {
				std::unordered_map<const State *, double> Vi;
				Vi = V[i];
				Vtmp.push_back(Vi);
			}

			// Create the set of actions, which is all actions initially.
			std::vector<const Action *> AStar;
			//* Method #1
			for (auto a : *A) {
				AStar.push_back(resolve(a));
			}
			//*/
			/* Method #2
			for (const Action *a : currentA) {
				AStar.push_back(a);
			}
			//*/

			/* Delta Prime Test
			std::vector<double> deltaPrime;
			for (int i = 0; i < R->get_num_rewards(); i++) {
				double val = std::fmax(0.0,
						std::fmin((1.0 - h->get_discount_factor()) * infinity[i] // infinityNormedDifference
								- std::numeric_limits<double>::epsilon() * 10.0,
								delta[i]));
				deltaPrime.push_back(val);
			}
			//*/

			// Starting with a set of all actions, prune each following the definition of lvmax.
			for (int i = 0; i < R->get_num_rewards(); i++) {
				const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));
				lvmax(S, AStar, T, Ri, h, s, Vtmp[i], delta[i], false); // Note: Do not update V, use delta.
				/* Delta Prime Test
				lvmax(S, AStar, T, Ri, h, s, Vtmp[i], deltaPrime[i], false); // Note: Do not update V, use delta.
				//*/

				/* Method #2
				// If this is the current index, we want to copy A_i (no star) to remember it for locking later.
				if (i == currentIndex) {
					currentIndexActions = AStar;
				}
				//*/
			}

			// Break the final ties using strict argmax.
			for (int i = 0; i < R->get_num_rewards(); i++) {
				const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));
				lvmax(S, AStar, T, Ri, h, s, Vtmp[i], delta[i], true); // Note: Update V, don't use delta.
				/* Delta Prime Test
//				lvmax(S, AStar, T, Ri, h, s, Vtmp[i], deltaPrime[i], true); // Note: Update V, don't use delta.
				//*/
			}

			// Find the maximum difference, as part of our convergence criterion check.
			//* Method #1
			for (int i = 0; i < R->get_num_rewards(); i++) {
				if (fabs(V[i][s] - Vtmp[i][s]) > infinityNormedDifference) {
					infinityNormedDifference = fabs(V[i][s] - Vtmp[i][s]);
				}
			}
			//*/
			/* Method #2
			if (fabs(V[currentIndex][s] - Vtmp[currentIndex][s]) > infinityNormedDifference) {
				infinityNormedDifference = fabs(V[currentIndex][s] - Vtmp[currentIndex][s]);
			}
			//*/

			/* Delta Prime Test
			for (int i = 0; i < R->get_num_rewards(); i++) {
				if (fabs(V[i][s] - Vtmp[i][s]) > infinity[i]) {
					infinity[i] = fabs(V[i][s] - Vtmp[i][s]);
				}
			}
			//*/

			// ----- DEBUG -----
			// NOTE: For some reason,
			for (int i = 0; i < R->get_num_rewards(); i++) {
				if (fabs(V[i][s] - Vtmp[i][s]) > 0.01) {
					std::cout << "For V_" << i << ", state '" << s->to_string() << "' has diff = " << fabs(V[i][s] - Vtmp[i][s]) << std::endl;
				}
			}

			// Update the value of the state for each i in K.
			for (int i = 0; i < R->get_num_rewards(); i++) {
				V[i][s] = Vtmp[i][s];
			}

			// Set the policy's action, which will yield the optimal policy at the end.
			policy->set(s, AStar[0]);
		}

		// ----- DEBUG -----
		//*
		int size = 8;
		for (int i = 0; i < V.size(); i++) {
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					try {
						const State *s = S->get(NamedState::hash_value(std::to_string(x) + " " + std::to_string(y)));
						std::cout << V[i][s] << "\t";
					} catch (const StateException &err) {
						std::cout << "xxxxxxxx\t";
					}
				}
				std::cout << std::endl;
			}
			std::cout << "\n\n";
		}
		//*/
		// ----- DEBUG -----
	}

	return policy;
}

void LVMaxValueIteration::lvmax(const FiniteStates *S, std::vector<const Action *> &AiStar,
		const FiniteStateTransitions *T, const SASRewards *Ri, const Horizon *h, const State *s,
		std::unordered_map<const State *, double> &Vi, double deltai, bool star)
{
	double maxQisa = std::numeric_limits<double>::lowest();

	std::vector<double> Qis;

	// For all the actions, compute max Q_i(s, a) over the current set of actions.
	for (const Action *a : AiStar) {
		// Compute the Q_i(s, a) estimate.
		double Qisa = compute_Q(S, T, Ri, h, s, a, Vi);
		Qis.push_back(Qisa);

		// Determine the maximum Q_i(s, a) value.
		if (Qisa > maxQisa) {
			maxQisa = Qisa;
		}
	}

	// Based on if this is a star pass (the second pass through) or not, update the actions differently. This
	// variable will hold the new A_i^* set.
	std::vector<const Action *> newAiStar;

	// In the case of a first pass, we need to use delta_i; however, in the second pass we use 0.0. Both will be
	// shifted by 1 order of magnitude above machine precision.
	double offset = 0.0; //std::numeric_limits<double>::epsilon() * 10.0;
	if (!star) {
		/* Normal Delta_i.
		offset += deltai;
		//*/

		//* Experiment: Delta_i * (1 - gamma) / gamma
		offset += deltai * (1.0 - h->get_discount_factor()) / h->get_discount_factor();
		//*/
	}

	// Keep all of the actions which are within 1 order of magnitude from machine precision of the max Q value.
	for (int i = 0; i < AiStar.size(); i++) {
		if (std::fabs(maxQisa - Qis[i]) <= offset) {
			newAiStar.push_back(AiStar[i]);
		}
	}

	// This actually changes the AiStar variable since it is passed by reference.
	AiStar = newAiStar;

	// We will only update V_i(s) in the second pass, since we are taking the argmax and thus all Q values will
	// be the same regardless of the action selected.
	if (star) {
		Vi[s] = maxQisa;
	}
}

double LVMaxValueIteration::compute_Q(const FiniteStates *S, const FiniteStateTransitions *T, const SASRewards *Ri,
		const Horizon *h, const State *s, const Action *a, std::unordered_map<const State *, double> &Vi)
{
	// Compute the Q_i(s, a) estimate.
	double Qisa = 0.0;

	// Get the list of successor states.
	std::vector<const State *> successors;
	T->successors(S, s, a, successors);

	// For each of the successors, compute the bellman update equation.
	for (const State *sPrime : successors) {
		Qisa += T->get(s, a, sPrime) * (Ri->get(s, a, sPrime) + h->get_discount_factor() * Vi[sPrime]);
	}

	return Qisa;
}
