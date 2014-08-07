
#include "../include/lvi_nova.h"

#include "../../librbr/librbr/include/core/state_transitions/state_transitions_array.h"
#include "../../librbr/librbr/include/core/state_transitions/state_transition_exception.h"

#include "../../librbr/librbr/include/core/rewards/sas_rewards_array.h"
#include "../../librbr/librbr/include/core/rewards/reward_exception.h"

#include "../../nova/nova/include/lvi.h"

#include <iostream>
#include <algorithm>

LVINova::LVINova()
{
	epsilon = 0.001;
}

LVINova::LVINova(double tolerance)
{
	epsilon = tolerance;
}

LVINova::~LVINova()
{ }

PolicyMap *LVINova::solve_infinite_horizon(const StatesMap *S, const ActionsMap *A,
		const StateTransitions *T, const FactoredRewards *R, const Initial *s0, const Horizon *h,
		const std::vector<float> &delta)
{
//	std::cout << "Initializing...";
//	std::cout.flush();
//
//	// The Nova version of this requires array versions of state transitions and each reward.
//	if (dynamic_cast<const StateTransitionsArray *>(T) == nullptr) {
//		throw StateTransitionException();
//	}
//
//	for (int i = 0; i < R->get_num_rewards(); i++) {
//		if (dynamic_cast<const SASRewardsArray *>(R->get(i)) == nullptr) {
//			throw RewardException();
//		}
//	}
//
//	// Create the policy based on the horizon.
//	PolicyMap *policy = new PolicyMap(h);
//
//	// The value of the states, one for each reward.
//	std::vector<std::unordered_map<const State *, double> > V;
//	V.resize(R->get_num_rewards());
//
//	// We will want to remember the optimal possible value of a state, too.
//	std::vector<std::unordered_map<const State *, double> > VStar;
//	VStar.resize(R->get_num_rewards());
//
//	// Remember the set of actions available to each of the value functions.
//	// This will be computed at the end of each step.
//	std::vector<std::unordered_map<const State *, std::vector<const Action *> > > AStar;
//	AStar.resize(R->get_num_rewards() + 1);
//
//	// Note: We will store the final value function's reduced set of actions, too.
//
//	// Setup the initial set of actions for i = 1.
//	for (auto state : *S) {
//		const State *s = resolve(state);
//		for (auto action : *A) {
//			const Action *a = resolve(action);
//			AStar[0][s].push_back(a);
//		}
//	}
//
//	std::cout << "Complete." << std::endl;
//	std::cout.flush();
//
//	/* Assume an array-absed MDP is given.
//	// Create the array-based MDPs first.
//	std::vector<const MDP *> mdp;
//	for (int i = 0; i < R->get_num_rewards(); i++) {
//		std::cout << "Constructing Array-Based MDP for Reward " << (i + 1) << "... ";
//		std::cout.flush();
//
//		const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));
//		MDP *newMDP = convert_map_to_array(S, A, T, Ri, s0, h);
//		mdp.push_back(newMDP);
//
//		std::cout << "Complete." << std::endl;
//		std::cout.flush();
//	}
//	//*/
//
//	// For each of the value functions, we will solve the MDP and then compute the next actions set.
//	for (int i = 0; i < R->get_num_rewards(); i++) {
//		std::cout << "Initializing Data for Reward " << (i + 1) << "... ";
//		std::cout.flush();
//
//		const SASRewards *Ri = static_cast<const SASRewards *>(R->get(i));
//
//		// Setup V[i] with initial values of 0.0.
//		for (auto state : *S) {
//			const State *s = resolve(state);
//			V[i][s] = 0.0;
//		}
//
//		// Reserve the memory for the value functions and strategy.
//		float *cudaV = new float[S->get_num_states()];
//		unsigned int *cudaPI = new unsigned int[S->get_num_states()];
//
//		for (int s = 0; s < S->get_num_states(); s++) {
//			cudaV[s] = 0.0f;
//			cudaPI[s] = 0;
//		}
//
//		// Create the array of available actions, represented as a boolean.
//		bool *cudaAStar = new bool[S->get_num_states() * A->get_num_actions()];
//		for (int j = 0; j < S->get_num_states(); j++) {
//			const State *s = S->get(j);
//
//			for (int k = 0; k < A->get_num_actions(); k++) {
//				const Action *a = A->get(k);
//
//				if (std::find(AStar[i][s].begin(), AStar[i][s].end(), a) != AStar[i][s].end()) {
//					cudaAStar[j * A->get_num_actions() + k] = true;
//				} else {
//					cudaAStar[j * A->get_num_actions() + k] = false;
//				}
//			}
//		}
//
//		std::cout << "Complete." << std::endl;
//		std::cout.flush();
//
////		std::cout << "Waiting... " << std::endl;
////		std::cout.flush();
////		usleep(500000);
////		std::cout << "Complete." << std::endl;
////		std::cout.flush();
//
//		std::cout << "Executing Value Iteration with CUDA... ";
//		std::cout.flush();
//
//		// Run value iteration optimized with CUDA!
//		int result = lvi(
//				S->get_num_states(),
//				A->get_num_actions(),
//				(const bool *)cudaAStar,
//				((const StateTransitionsArray *)T)->get_state_transitions(),
//				((const SASRewardsArray *)R->get(i))->get_rewards(),
//				(float)((const SASRewardsArray *)R->get(i))->get_max(),
//				(float)h->get_discount_factor(),
//				(float)epsilon,
//				cudaV,
//				cudaPI,
//				(int)std::ceil((double)S->get_num_states() / 128.0),
//				128);
//
//		std::cout << "Complete." << std::endl;
//		std::cout.flush();
//
////		std::cout << "Waiting... " << std::endl;
////		std::cout.flush();
////		usleep(500000);
////		std::cout << "Complete." << std::endl;
////		std::cout.flush();
//
//		// Check the result. If successful, then copy the resultant value function and policy.
//		if (result == 0) {
//			std::cout << "Verifying and copying data... ";
//			std::cout.flush();
//
//			for (int j = 0; j < S->get_num_states(); j++) {
//				const State *s = S->get(j);
//
//				// Set the value of the state.
//				V[i][s] = cudaV[j];
//
//				// Set the policy.
//				for (int k = 0; k < A->get_num_actions(); k++) {
//					if (k == cudaPI[j]) {
//						const Action *a = A->get(k);
//						policy->set(s, a);
//						break;
//					}
//				}
//			}
//
//			std::cout << "Complete." << std::endl;
//			std::cout.flush();
//		}
//
////		std::cout << "Waiting... " << std::endl;
////		std::cout.flush();
////		usleep(500000);
////		std::cout << "Complete." << std::endl;
////		std::cout.flush();
//
//		// After everything, we can finally compute the set of actions ***for i + 1*** with the delta slack.
//		std::cout << "Updating A_{i+1}^*... ";
//		std::cout.flush();
//
//		for (auto state : *S) {
//			const State *s = resolve(state);
//
//			// Use the delta function to compute the final set of AStar[i + 1].
//			compute_A_delta(S, AStar[i][s], T, Ri, h, s, V[i], delta[i], AStar[i + 1][s]);
//		}
//
//		std::cout << "Complete." << std::endl;
//		std::cout.flush();
//
////		std::cout << "Waiting... " << std::endl;
////		std::cout.flush();
////		usleep(500000);
////		std::cout << "Complete." << std::endl;
////		std::cout.flush();
//
//		// Free the memory at each loop, since the MDP has been solved now.
////		delete mdp[i];
//		delete cudaV;
//		delete cudaPI;
//		delete cudaAStar;
//	}
////	mdp.clear();
//
//	/*
//	// Output the pretty values in a table format for the GridMDP object!
//    int size = 8;
//
//	for (int i = 0; i < R->get_num_rewards(); i++) {
//        std::cout << "V[" << i << "]:" << std::endl;
//
//        for (int c = 0; c <= 1; c++) {
////            std::cout << "c = " << c << std::endl;
//
//            for (int y = 0; y < size; y++) {
//                for (int x = 0; x < size; x++) {
//                    try {
//                        const State *s = S->get(NamedState::hash_value(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(c)));
//                        std::cout << V[i][s];
//                        if (x < size - 1) {
//                        	std::cout << ",";
//                        }
//                    } catch (const StateException &err) {
//                        std::cout << "0.0,";
//                    }
//                }
//                std::cout << std::endl;
//            }
//        }
//
//        std::cout << "\n\n";
//	}
//	//*/
//
//	return policy;

	return nullptr;
}
