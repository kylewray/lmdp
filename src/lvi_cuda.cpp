
#include "../include/lvi_cuda.h"

#include "../../librbr/librbr/include/core/states/indexed_state.h"

#include "../../librbr/librbr/include/core/state_transitions/state_transitions_array.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transition_exception.h"

#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"
#include "../../librbr/librbr/include/core/rewards/reward_exception.h"

#include "../../librbr/librbr/include/core/policy/policy_exception.h"

// Include Cuda-optimized LVI.
#include "../lvi_cuda/lvi_cuda.h"

#include <iostream>
#include <algorithm>

LVICuda::LVICuda()
{
	epsilon = 0.001;

	cudaP = nullptr;
	cudaPI = nullptr;

	d_T = nullptr;
	d_R = nullptr;
	d_P = nullptr;
	d_pi = nullptr;
}

LVICuda::LVICuda(double tolerance)
{
	epsilon = tolerance;

	cudaP = nullptr;
	cudaPI = nullptr;

	d_T = nullptr;
	d_R = nullptr;
	d_P = nullptr;
	d_pi = nullptr;
}

LVICuda::~LVICuda()
{ }

PolicyMap *LVICuda::solve_infinite_horizon(const StatesMap *S, const ActionsMap *A,
		const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
		const std::vector<float> &delta,
		const std::vector<std::vector<const State *> > &P,
		const std::vector<std::vector<unsigned int> > &o)
{
	initialize_variables(S, A, T, R, P);

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

			compute_partition(S, A, T, R, s0, h, delta, j, P[j], o[j], VFixed, V, policy, difference[j]);
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

	uninitialize_variables(R->get_num_rewards(), P.size());

	return policy;
}

void LVICuda::compute_partition(const StatesMap *S, const ActionsMap *A, const StateTransitions *T,
		const FactoredRewards *R, const Initial *s0, const Horizon *h, const std::vector<float> &delta,
		int j,
		const std::vector<const State *> &Pj, const std::vector<unsigned int> &oj,
		const std::vector<std::unordered_map<const State *, double> > &VFixed,
		std::vector<std::unordered_map<const State *, double> > &V,
		PolicyMap *policy, std::vector<double> &maxDifference)
{
	const StateTransitionsArray *Tarray = dynamic_cast<const StateTransitionsArray *>(T);
	if (Tarray == nullptr) {
		throw PolicyException();
	}

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
		const SASRewardsArray *Ri = dynamic_cast<const SASRewardsArray *>(R->get(oj[i]));
		if (Ri == nullptr) {
			throw PolicyException();
		}

		// In order of cudaPj (above), e.g., 5, 8, 1, 2, ...
		float *cudaVi = new float[S->get_num_states()];
		for (int state = 0; state < (int)S->get_num_states(); state++) {
			const State *s = S->get(state);
			cudaVi[state] = VFixed.at(oj[i]).at(s);
		}

		// Create the array of available actions, represented as a boolean.
		bool *cudaAStar = new bool[Pj.size() * A->get_num_actions()];
		for (int state = 0; state < (int)Pj.size(); state++) {
			const State *s = Pj.at(state);

			for (int action = 0; action < (int)A->get_num_actions(); action++) {
				const Action *a = A->get(action);

				if (std::find(AStar[oj[i]][s].begin(), AStar[oj[i]][s].end(), a) != AStar[oj[i]][s].end()) {
					cudaAStar[state * A->get_num_actions() + action] = true;
				} else {
					cudaAStar[state * A->get_num_actions() + action] = false;
				}
			}
		}

		// Run value iteration optimized with CUDA!
		int result = lvi_cuda(S->get_num_states(),
							Pj.size(),
							A->get_num_actions(),
							(const bool *)cudaAStar,
							d_T,
							d_R[oj[i]],
							d_P[j],
							d_pi[j],
							(float)Ri->get_min(),
							(float)Ri->get_max(),
							(float)h->get_discount_factor(),
							(float)epsilon,
							(unsigned int)std::ceil((double)Pj.size() / 1024.0),
							(unsigned int)1024,
							cudaVi);

		if (result == 0) {
			for (int state = 0; state < (int)Pj.size(); state++) {
				const State *s = Pj.at(state);

				// Set the value of the state.
				V[oj[i]][s] = cudaVi[cudaP[j][state]];
			}
		} else {
			std::cout << "Error[compute_partition]: Failed to copy CUDA data." << std::endl;
			std::cout.flush();

			throw PolicyException();
		}

		// After everything, we can finally compute the set of actions ***for i + 1*** with the delta slack.
		if (i != (int)R->get_num_rewards() - 1) {
			for (auto s : Pj) {
				// Use the delta function to compute the final set of AStar[i + 1].
				// NOTE: Intentionally, we use [s] for AStar, because this has not yet been defined for i + 1. This
				// creates the vector for state s in place.
				compute_A_delta(S, AStar.at(oj[i]).at(s), T, Ri, h, s, V.at(oj[i]), delta[oj[i]], AStar.at(oj[i + 1])[s]);
			}
		} else {
			// This must be the final reward, so instead of computing AStar[oj[i + 1]], we compute the action selected!
			lvi_get_policy(Pj.size(), d_P[j], cudaP[j]);

			if (result == 0) {
				for (int state = 0; state < (int)Pj.size(); state++) {
					const State *s = Pj.at(state);

					// Set the policy.
					for (int action = 0; action < (int)A->get_num_actions(); action++) {
						if (action == (int)cudaPI[j][state]) {
							const Action *a = A->get(action);
							policy->set(s, a);
							break;
						}
					}
				}
			} else {
				std::cout << "Error[compute_partition]: Failed to copy CUDA data." << std::endl;
				std::cout.flush();

				throw PolicyException();
			}
		}

		// Free the memory at each loop, since the MDP has been solved now.
		delete cudaVi;
		delete cudaAStar;
	}

	// Update the maximum difference found over all partitions after the subset
	// of states have had VI executed. This does not follow the ordering, meaning
	// that maxDifference stores the differences in order of 1, 2, 3, etc, not
	// the ordering, e.g., 3, 1, 2, etc. Also, this is equivalent to above.. so remove this.
	for (int i = 0; i < (int)R->get_num_rewards(); i++) {
		for (auto s : Pj) {
			if (fabs(V.at(i).at(s) - VFixed.at(i).at(s)) > maxDifference[i]) {
				maxDifference[i] = fabs(V.at(i).at(s) - VFixed.at(i).at(s));
			}
		}
	}

	std::cout << "Completed partition." << std::endl; std::cout.flush();
}

void LVICuda::initialize_variables(const StatesMap *S, const ActionsMap *A, const StateTransitions *T,
		const FactoredRewards *R, const std::vector<std::vector<const State *> > &P)
{
	const StateTransitionsArray *Tarray = dynamic_cast<const StateTransitionsArray *>(T);
	if (Tarray == nullptr) {
		throw PolicyException();
	}

	int k = (int)R->get_num_rewards();
	int ell = (int)P.size();

	cudaP = new unsigned int *[ell];
	cudaPI = new unsigned int *[ell];

	for (int j = 0; j < ell; j++) {
		// Reserve the memory for the value functions and strategy.
		cudaP[j] = new unsigned int[P[j].size()];
		cudaPI[j] = new unsigned int[P[j].size()];

		for (int state = 0; state < (int)P[j].size(); state++) {
			const IndexedState *s = dynamic_cast<const IndexedState *>(P[j][state]);
			cudaP[j][state] = s->get_index();
			cudaPI[j][state] = 0;
		}
	}

	int result = lvi_initialize_state_transitions(S->get_num_states(),
												A->get_num_actions(),
												Tarray->get_state_transitions(),
												d_T);
	if (result != 0) {
		throw PolicyException();
	}

	d_R = new float *[k];

	for (int i = 0; i < k; i++) {
		const SASRewardsArray *Ri = dynamic_cast<const SASRewardsArray *>(R->get(i));
		if (Ri == nullptr) {
			throw PolicyException();
		}

		result = lvi_initialize_rewards(S->get_num_states(),
									A->get_num_actions(),
									Ri->get_rewards(),
									d_R[i]);
		if (result != 0) {
			throw PolicyException();
		}
	}

	d_P = new unsigned int *[ell];
	d_pi = new unsigned int *[ell];

	for (int j = 0; j < ell; j++) {
		result = lvi_initialize_partition(P[j].size(),
										cudaP[j],
										cudaPI[j],
										d_P[j],
										d_pi[j]);
		if (result != 0) {
			throw PolicyException();
		}
	}
}

void LVICuda::uninitialize_variables(unsigned int k, unsigned int ell)
{
	if (cudaP != nullptr) {
		for (int j = 0; j < ell; j++) {
			delete [] cudaP[j];
		}
		delete [] cudaP;
	}
	cudaP = nullptr;

	if (cudaPI != nullptr) {
		for (int j = 0; j < ell; j++) {
			delete [] cudaPI[j];
		}
		delete [] cudaPI;
	}
	cudaPI = nullptr;

	lvi_uninitialize(d_T, k, d_R, ell, d_P, d_pi);

	d_T = nullptr;

	delete [] d_R;
	d_R = nullptr;

	delete [] d_P;
	d_P = nullptr;

	delete [] d_pi;
	d_pi = nullptr;
}
