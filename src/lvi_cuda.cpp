
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
}

LVICuda::LVICuda(double tolerance)
{
	epsilon = tolerance;
}

LVICuda::~LVICuda()
{ }

void LVICuda::compute_partition(const StatesMap *S, const ActionsMap *A, const StateTransitions *T,
		const FactoredRewards *R, const Initial *s0, const Horizon *h, const std::vector<float> &delta,
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

		// Reserve the memory for the value functions and strategy.
		unsigned int *cudaPj = new unsigned int[Pj.size()];
		unsigned int *cudaPI = new unsigned int[Pj.size()];

		for (int state = 0; state < (int)Pj.size(); state++) {
			const IndexedState *s = dynamic_cast<const IndexedState *>(Pj[state]);
			cudaPj[state] = s->get_index();
			cudaPI[state] = 0;
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
		int result = lvi_cuda(
				S->get_num_states(),
				Pj.size(),
				cudaPj,
				A->get_num_actions(),
				(const bool *)cudaAStar,
				Tarray->get_state_transitions(),
				Ri->get_rewards(),
				(float)Ri->get_min(),
				(float)Ri->get_max(),
				(float)h->get_discount_factor(),
				(float)epsilon,
				cudaVi,
				cudaPI,
				(int)std::ceil((double)Pj.size() / 512.0),
				512);

		if (result == 0) {
			for (int state = 0; state < (int)Pj.size(); state++) {
				const State *s = Pj.at(state);

				// Set the value of the state.
				V[oj[i]][s] = cudaVi[cudaPj[state]];

				// Set the policy.
				for (int action = 0; action < (int)A->get_num_actions(); action++) {
					if (action == (int)cudaPI[state]) {
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

		// After everything, we can finally compute the set of actions ***for i + 1*** with the delta slack.
		if (i != (int)R->get_num_rewards() - 1) {
			for (auto s : Pj) {
				// Use the delta function to compute the final set of AStar[i + 1].
				// NOTE: Intentionally, we use [s] for AStar, because this has not yet been defined for i + 1. This
				// creates the vector for state s in place.
				compute_A_delta(S, AStar.at(oj[i]).at(s), T, Ri, h, s, V.at(oj[i]), delta[oj[i]], AStar.at(oj[i + 1])[s]);
			}
		}

		// Free the memory at each loop, since the MDP has been solved now.
		delete cudaPj;
		delete cudaVi;
		delete cudaPI;
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
