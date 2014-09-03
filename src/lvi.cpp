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


#include <unistd.h>

#include "../include/lvi.h"

#include "../../nova/nova/include/lvi.h"

#include "../../librbr/librbr/include/management/conversion.h"

#include "../../librbr/librbr/include/core/state_transitions/state_transitions_array.h"
#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"

#include "../../librbr/librbr/include/core/core_exception.h"
#include "../../librbr/librbr/include/core/states/state_exception.h"
#include "../../librbr/librbr/include/core/actions/action_exception.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transition_exception.h"
#include "../../librbr/librbr/include/core/rewards/reward_exception.h"
#include "../../librbr/librbr/include/core/policy/policy_exception.h"

#include "../../librbr/librbr/include/core/actions/action_utilities.h"

#include <iostream> // TODO: Remove this after debug statements have been removed.

#include <math.h>
#include <vector>
#include <unordered_map>
#include <algorithm>

LVI::LVI()
{
	epsilon = 0.001;
	loopingVersion = false;
}

LVI::LVI(double tolerance, bool enableLooping)
{
	epsilon = tolerance;
	loopingVersion = enableLooping;
}

LVI::~LVI()
{ }

PolicyMap *LVI::solve(const LMDP *lmdp)
{
	// Handle the trivial case.
	if (lmdp == nullptr) {
		return nullptr;
	}

	// Attempt to convert the states object into FiniteStates.
	const StatesMap *S = dynamic_cast<const StatesMap *>(lmdp->get_states());
	if (S == nullptr) {
		throw StateException();
	}

	// Attempt to convert the actions object into FiniteActions.
	const ActionsMap *A = dynamic_cast<const ActionsMap *>(lmdp->get_actions());
	if (A == nullptr) {
		throw ActionException();
	}

	// Note: All forms of StateTransitions objects are valid here, since they all require get.
	const StateTransitions *T = lmdp->get_state_transitions();

	// Attempt to convert the rewards object into FactoredRewards. Also, ensure that the
	// type of each element is SASRewards.
	const FactoredRewards *R = dynamic_cast<const FactoredRewards *>(lmdp->get_rewards());
	if (R == nullptr) {
		throw RewardException();
	}
	/*
	for (int i = 0; i < R->get_num_rewards(); i++) {
		const SASRewards *Ri = dynamic_cast<const SASRewards *>(R->get(oj[i]));
		if (Ri == nullptr) {
			throw RewardException();
		}
	}
	*/

	// Handle the other trivial case in which the slack variables were incorrectly defined.
	if (lmdp->get_slack().size() != R->get_num_rewards()) {
		throw RewardException();
	}
	for (int i = 0; i < (int)lmdp->get_slack().size(); i++) {
		if (lmdp->get_slack().at(i) < 0.0) {
			throw RewardException();
		}
	}

	// Obtain the horizon and return the correct value iteration.
	const Initial *s0 = lmdp->get_initial_state();
	const Horizon *h = lmdp->get_horizon();
	if (h->is_finite()) {
		throw CoreException();
	}

	return solve_infinite_horizon(S, A, T, R, s0, h,
			lmdp->get_slack(), lmdp->get_partitions(), lmdp->get_orderings());
}

const std::vector<std::unordered_map<const State *, double> > &LVI::get_V() const
{
	return V;
}

PolicyMap *LVI::solve_infinite_horizon(const StatesMap *S, const ActionsMap *A,
		const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
		const std::vector<float> &delta,
		const std::vector<std::vector<const State *> > &P,
		const std::vector<std::vector<unsigned int> > &o)
{
	// Create the policy based on the horizon.
	PolicyMap *policy = new PolicyMap(h);

	// The value of the states, one for each reward.
	V.clear();
	V.resize(R->get_num_rewards());

	// We will want to remember the previous fixed values of states, too.
	std::vector<std::unordered_map<const State *, double> > VFixed;
	VFixed.resize(R->get_num_rewards());

	// Default all values.
	for (auto state : *S) {
		const State *s = resolve(state);
		for (int i = 0; i < (int)R->get_num_rewards(); i++) {
			V[i][s] = 0.0;
			VFixed[i][s] = 0.0;
		}
	}

	// Compute the convergence criterion.
	double convergenceCriterion = epsilon * std::max(0.1, (1.0 - h->get_discount_factor()) / h->get_discount_factor());
	bool converged = false;

	std::vector<std::vector<double> > difference;
	difference.resize(P.size());
	for (int j = 0; j < (int)P.size(); j++) {
		difference[j].resize(R->get_num_rewards());
	}

	std::cout << "Starting...\n"; std::cout.flush();

	//*
	// ------------------------------------------------------------------------------
	// ------------------------------------------------------------------------------
	// ------------------------------------------------------------------------------

	// Print out the iteration's convergence table result.
	printf("Iterations      ");
	for (int j = 0; j < (int)P.size(); j++) {
		for (int i = 0; i < (int)R->get_num_rewards(); i++) {
			std::cout << o[j][i] << " ";
		}
		if (j != (int)P.size() - 1) {
			std::cout << "    ";
		}
	}
	std::cout << "    ";
	for (int j = 0; j < (int)P.size(); j++) {
		for (int i = 0; i < (int)R->get_num_rewards(); i++) {
			printf("o(%i) = %-3i ", i, o[j][i]);
		}
	}
	std::cout << std::endl; std::cout.flush();

	// ------------------------------------------------------------------------------
	// ------------------------------------------------------------------------------
	// ------------------------------------------------------------------------------
	//*/

	// Iterate the outer loop until the convergence criterion is satisfied.
	int counter = 1;
	while (!converged) {
		// Update VFixed to the previous value of V.
		for (auto state : *S) {
			const State *s = resolve(state);
			for (int i = 0; i < (int)R->get_num_rewards(); i++) {
				VFixed[i][s] = V[i][s];
			}
		}

		converged = true;

		// For each of the partitions, run value iteration. Each time, copy the resulting value functions.
		for (int j = 0; j < (int)P.size(); j++) {
			// Reset the difference for *all* of the variables.
			for (int i = 0; i < (int)R->get_num_rewards(); i++) {
				difference[j][i] = 0.0;
			}

			compute_partition(S, A, T, R, s0, h, delta, P[j], o[j], VFixed, V, policy, difference[j]);
		}

		// Check for convergence.
		for (int j = 0; j < (int)P.size(); j++) {
			for (int i = 0; i < (int)R->get_num_rewards(); i++) {
				if (difference[j][i] > convergenceCriterion) {
					converged = false;
				}
			}
		}

		//*
		// ------------------------------------------------------------------------------
		// ------------------------------------------------------------------------------
		// ------------------------------------------------------------------------------

		// Print out the iteration's convergence table result.
		printf("Iteration %-3i [ ", counter);

		for (int j = 0; j < (int)P.size(); j++) {
			for (int i = 0; i < (int)R->get_num_rewards(); i++) {
				// NOTE: Some value functions in the ordering may converge before the ones before them, but this is
				// not guaranteed. The only guarantee is that once a 'parent' has converged, its 'child' will converge.
				// Eventually, this must include all value functions over all partitions.
				if (difference[j][o[j][i]] > convergenceCriterion) {
					std::cout << "x ";
				} else {
					std::cout << "o ";
				}
			}

			if (j != (int)P.size() - 1) {
				std::cout << "| ";
			}
		}

		std::cout << "]   ";

		for (int j = 0; j < (int)P.size(); j++) {
			for (int i = 0; i < (int)R->get_num_rewards(); i++) {
				// NOTE: Some value functions in the ordering may converge before the ones before them, but this is
				// not guaranteed. The only guarantee is that once a 'parent' has converged, its 'child' will converge.
				// Eventually, this must include all value functions over all partitions.
//				std::cout << difference[j][i] << "\t";
				printf("%10.6f ", difference[j][o[j][i]]);
			}
		}

		std::cout << std::endl; std::cout.flush();

		counter++;

		// ------------------------------------------------------------------------------
		// ------------------------------------------------------------------------------
		// ------------------------------------------------------------------------------
		//*/
	}

	std::cout << "Complete LVI." << std::endl; std::cout.flush();

	return policy;
}

void LVI::compute_partition(const StatesMap *S, const ActionsMap *A, const StateTransitions *T,
		const FactoredRewards *R, const Initial *s0, const Horizon *h, const std::vector<float> &delta,
		const std::vector<const State *> &Pj, const std::vector<unsigned int> &oj,
		const std::vector<std::unordered_map<const State *, double> > &VFixed,
		std::vector<std::unordered_map<const State *, double> > &V,
		PolicyMap *policy, std::vector<double> &maxDifference)
{
	std::cout << "LVI Version\n"; std::cout.flush();

	// The value of the states, one for each reward.
	std::vector<std::unordered_map<const State *, double> > VPrime;
	VPrime.resize(R->get_num_rewards());

	// Compute the convergence criterion which follows from the proof of convergence.
	double convergenceCriterion = epsilon * std::max(0.1, (1.0 - h->get_discount_factor()) / h->get_discount_factor());

	// Remember the set of actions available to each of the value functions. This will be computed at the end of each step.
	std::vector<std::unordered_map<const State *, std::vector<const Action *> > > AStar;
	AStar.resize(R->get_num_rewards());

	// Setup the initial set of actions for i = 1.
	for (auto s : Pj) {
		for (auto action : *A) {
			const Action *a = resolve(action);
			AStar[oj[0]][s].push_back(a);
		}
	}

	// For each of the value functions, we will compute the actions set.
	for (int i = 0; i < (int)R->get_num_rewards(); i++) {
//std::cout << "Starting VI for Reward " << oj[i] << std::endl; std::cout.flush();

		const SASRewards *Ri = dynamic_cast<const SASRewards *>(R->get(oj[i]));

		// Setup V[i] with initial values of 0.0.
		for (auto state : *S) {
			const State *s = resolve(state);

			// NOTE: You can remove the if statement and just use VStar.at(oj[i]).at(s)!!! It is faster, since it
			// does not matter which V[.] you initialize it to be.

			// If it is in the partition, then it will be updated, so initialize to 0.0.
//			if (std::find(Pj.begin(), Pj.end(), s) != Pj.end()) {
//				V[oj[i]][s] = 0.0;
//			} else {
				VPrime[oj[i]][s] = VFixed.at(oj[i]).at(s);
//			}
		}

		double difference = convergenceCriterion + 1.0;

		// For this V_i, converge until you reach within epsilon of V_i^*.
		do {
//std::cout << "Value Iteration: Convergence Check: " << difference << " vs " << convergenceCriterion << std::endl;

			difference = 0.0;

			// Copy the value of all states for this i. The constant type casting forces it to use the copy
			// assignment instead of the move one.
			std::unordered_map<const State *, double> Vi;
			for (auto s : Pj) {
				Vi[s] = VPrime.at(oj[i]).at(s);
			}

			// For all the states, compute V_i(s).
			for (auto s : Pj) {
				const Action *a = nullptr;

				// Update V according to the previously converged subset of actions.
				compute_V(S, AStar.at(oj[i]).at(s), T, Ri, h, s, VPrime.at(oj[i]), Vi, a);

				// Store the action taken as part of the policy. This will change all the time, especially over i, but whatever.
				policy->set(s, a);

				// Continue to compute the infinity normed difference between value functions for convergence checking.
				if (std::fabs(VPrime[oj[i]][s] - Vi[s]) > difference) {
					difference = std::fabs(VPrime.at(oj[i]).at(s) - Vi.at(s));
				}
			}

			// After iterating over states, update the real V[i] for all s.
			for (auto s : Pj) {
				VPrime.at(oj[i]).at(s) = Vi.at(s);
			}
		} while (loopingVersion && difference > convergenceCriterion);

//std::cout << "Value Iteration: Convergence Check: " << difference << " vs " << convergenceCriterion << std::endl;

		// After everything, we can finally compute the set of actions ***for i + 1*** with the delta slack.
		if (i != (int)R->get_num_rewards() - 1) {
			for (auto s : Pj) {
				// Use the delta function to compute the final set of AStar[i + 1].
				// NOTE: Intentionally, we use [s] for AStar, because this has not yet been defined for i + 1. This
				// creates the vector for state s in place.
				compute_A_delta(S, AStar.at(oj[i]).at(s), T, Ri, h, s, VPrime.at(oj[i]), delta[oj[i]], AStar.at(oj[i + 1])[s]);
			}

//// DEBUG!!!! NOTE: ALL ACTIONS ARE POSSIBLE FOR AStar.at(oj[0])!!!!!!!!!!!!!!!!!
//for (auto s : Pj) {
//std::cout << AStar.at(oj[i + 1]).at(s).size() << " ";
//}
//std::cout << std::endl; std::cout.flush();
		}

		// Copy the final results for these states.
		for (auto s : Pj) {
//			// Also, update the maximum difference found over all partitions after the subset
//			// of states have had VI executed.
//			if (fabs(V.at(oj[i]).at(s) - VPrime.at(oj[i]).at(s)) > maxDifference[oj[i]]) {
//				maxDifference[oj[i]] = fabs(V.at(oj[i]).at(s) - VPrime.at(oj[i]).at(s));
//			}

			V[oj[i]][s] = VPrime.at(oj[i]).at(s);
		}
	}

	/*
	std::cout << "Completed policy for partition. Now computing actual V." << std::endl; std::cout.flush();

	// At the very end, compute the final values of V following the policy.
	for (int i = 0; i < R->get_num_rewards(); i++) {
		std::cout << "Starting VI for Reward " << oj[i] << std::endl; std::cout.flush();

		const SASRewards *Ri = dynamic_cast<const SASRewards *>(R->get(oj[i]));

		for (auto state : *S) {
			const State *s = resolve(state);

//			if (std::find(Pj.begin(), Pj.end(), s) != Pj.end()) {
//				V[oj[i]][s] = 0.0;
//			} else {
				V[oj[i]][s] = VPrime.at(oj[i]).at(s);
//			}
		}

		double difference = convergenceCriterion + 1.0;

		// For this V_i, converge until you reach within epsilon of V_i^*.
		while (difference > convergenceCriterion) {
//			std::cout << "Computing Final Values: Convergence Check: " << difference << " vs " << convergenceCriterion << std::endl;

			difference = 0.0;

			// For all the states in the partition, compute V_i(s).
			for (auto s : Pj) {
				double Vis = compute_Q(S, T, Ri, h, s, policy->get(s), V[oj[i]]);

				if (std::fabs(V[oj[i]][s] - Vis) > difference) {
					difference = std::fabs(V[oj[i]][s] - Vis);
				}

				V[oj[i]][s] = Vis;
			}
		}
	}
	//*/

	// Update the maximum difference found over all partitions after the subset
	// of states have had VI executed. This does not follow the ordering, meaning
	// that maxDifference stores the differences in order of 1, 2, 3, etc, not
	// the ordering, e.g., 3, 1, 2, etc. Also, this is equivalent to above.. so remove this.
	for (int i = 0; i < (int)R->get_num_rewards(); i++) {
		for (auto s : Pj) {
			if (fabs(VPrime.at(i).at(s) - VFixed.at(i).at(s)) > maxDifference[i]) {
				maxDifference[i] = fabs(VPrime.at(i).at(s) - VFixed.at(i).at(s));
			}
		}
	}

//	std::cout << "Completed partition." << std::endl; std::cout.flush();
}

void LVI::compute_A_argmax(const StatesMap *S, const std::vector<const Action *> &Ai,
		const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		std::vector<const Action *> &AiPlus1)
{
	compute_A_delta(S, Ai, T, Ri, h, s, Vi, 0.0, AiPlus1);
}

void LVI::compute_A_delta(const StatesMap *S, const std::vector<const Action *> &Ai,
		const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		float deltai,
		std::vector<const Action *> &AiPlus1)
{
	double maxQisa = -std::numeric_limits<double>::max();
	std::vector<double> Qis;

	// For all the actions, compute max Q_i(s, a) over the current set of actions. Also, record the
	// value of each Q_i(s, a) for all actions a in A.
	for (int i = 0; i < (int)Ai.size(); i++) {
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
	for (int i = 0; i < (int)Ai.size(); i++) {
		// Check if this is difference within eta_i, but account for machine precision issues
		// within 1 order of magnitude.
//std::cout << fabs(maxQisa - Qis[i]) << " vs (1 - " << h->get_discount_factor() << ") * " << deltai << " = " << etai << std::endl;
		if (fabs(maxQisa - Qis[i]) < etai + std::numeric_limits<double>::epsilon() * 10.0) {
			AiPlus1.push_back(Ai[i]);
		}
	}
}

void LVI::compute_V(const StatesMap *S, const std::vector<const Action *> &Ai,
		const StateTransitions *T, const SASRewards *Ri, const Horizon *h,
		const State *s, const std::unordered_map<const State *, double> &Vi,
		std::unordered_map<const State *, double> &ViNext, const Action *&a)
{
	// Compute the maximal Q_i(s, a) given the reduced set of actions.
	ViNext[s] = -std::numeric_limits<double>::max();
	a = nullptr;

	// For all the actions, compute max Q_i(s, a) over the current set of actions.
	for (int i = 0; i < (int)Ai.size(); i++) {
		const Action *action = Ai[i];

		double Qisa = compute_Q(S, T, Ri, h, s, action, Vi);
		if (Qisa > ViNext[s]) {
			ViNext[s] = Qisa;
			a = action;
		}
	}
}

double LVI::compute_Q(const StatesMap *S, const StateTransitions *T, const SASRewards *Ri,
		const Horizon *h, const State *s, const Action *a,
		const std::unordered_map<const State *, double> &Vi)
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
			throw PolicyException();
		}

		double Vis = VisIterator->second;
		Qisa += T->get(s, a, sPrime) * (Ri->get(s, a, sPrime) + h->get_discount_factor() * Vis);
	}

	return Qisa;
}
