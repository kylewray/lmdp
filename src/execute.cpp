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


#include "../include/losm_lmdp.h"
#include "../include/grid_lmdp.h"

#include "../include/lvi.h"
#include "../include/lvi_cuda.h"

#include "../../librbr/librbr/include/mdp/mdp_value_iteration.h"
#include "../../librbr/librbr/include/mdp/mdp_utilities.h"

#include "../../losm/losm/include/losm_exception.h"

#include "../../librbr/librbr/include/management/raw_file.h"

#include "../../librbr/librbr/include/core/core_exception.h"

#include "../../librbr/librbr/include/core/states/state_utilities.h"

#include "../../librbr/librbr/include/core/actions/action_utilities.h"
#include "../../librbr/librbr/include/core/actions/action_exception.h"

#include <iostream>
#include <unordered_map>

int main(int argc, char *argv[]) {

	bool losmVersion = true;
	bool viWeightCheck = false;
	bool cudaVersion = true;

	if (losmVersion) {
		// Ensure the correct number of arguments.
		if (argc != 9) {
			std::cerr << "Please specify nodes, edges, and landmarks data files, as well as the initial and goal nodes' UIDs, plus the policy output file." << std::endl;
			return -1;
		}

		// Load the LOSM MDP.
		LOSMMDP *losmMDP = nullptr;
		try {
			losmMDP = new LOSMMDP(argv[1], argv[2], argv[3], argv[6], argv[7]);
		} catch (LOSMException &err) {
			std::cerr << "Failed to load the files provided." << std::endl;
			return -1;
		}

		losmMDP->set_slack(0.0f, 0.0f);

//		losmMDP->set_uniform_conditional_preference();
		losmMDP->set_tiredness_conditional_preference();

		// Solve the LOSM MDP using LVI.
		PolicyMap *policy = nullptr;

		if (cudaVersion) {
			LVICuda solver(0.0001);
			policy = solver.solve(losmMDP);
			losmMDP->save_policy(policy, argv[8], solver.get_V());
		} else {
			LVI solver(0.0001, true); // false);
			policy = solver.solve(losmMDP);
			losmMDP->save_policy(policy, argv[8], solver.get_V());
		}

		std::vector<std::unordered_map<const State *, double> > V;

		compute_V_pi(dynamic_cast<const StatesMap *>(losmMDP->get_states()),
						dynamic_cast<const ActionsMap *>(losmMDP->get_actions()),
						losmMDP->get_state_transitions(),
						dynamic_cast<const FactoredRewards *>(losmMDP->get_rewards()),
						losmMDP->get_horizon(),
						0.0001,
						policy,
						V);

		// Saving it with this V means the actual value of the policy: V^\pi, versus V^\eta ('solver.get_V()').
		losmMDP->save_policy(policy, argv[8], V);

		// Before we start this VI loop, get the initial state.
		const LOSMState *initialState = losmMDP->get_initial_state(argv[4], argv[5]);

		// Output the initial state's value for this policy.
		std::cout << "Initial State Value for LVI: ";
		std::cout << V.at(0).at(initialState) << ", ";
		std::cout << V.at(1).at(initialState) << std::endl;

		delete policy;

		// Solve the LOSM MDP using VI with various weights, and save the values of the initial state each time.
		if (viWeightCheck) {
			std::cout << "Initial State Values for VI with Weights:" << std::endl;

			for (double weight = 0.0; weight <= 1.0; weight += 0.1) {
				losmMDP->set_rewards_weights({weight, 1.0 - weight});

				MDPValueIteration viSolver(0.0001);
				PolicyMap *viPolicy = viSolver.solve(losmMDP);

				compute_V_pi(dynamic_cast<const StatesMap *>(losmMDP->get_states()),
							dynamic_cast<const ActionsMap *>(losmMDP->get_actions()),
							losmMDP->get_state_transitions(),
							dynamic_cast<const FactoredRewards *>(losmMDP->get_rewards()),
							losmMDP->get_horizon(),
							0.0001,
							viPolicy,
							V);

				std::cout << "Weight: [" << weight << ", " << (1.0 - weight) << "]: ";
				std::cout << V.at(0).at(initialState) << ", ";
				std::cout << V.at(1).at(initialState) << std::endl;

				delete viPolicy;
			}
		}

		delete losmMDP;

	} else {
		RawFile rawFile;

//		GridLMDP *gridLMDP = new GridLMDP(0, 5, 0, -0.03);
		GridLMDP *gridLMDP = new GridLMDP(0, 10, 0, -0.03);
//		GridLMDP *gridLMDP = new GridLMDP(1, 8, 10, -0.03);
//		GridLMDP *gridLMDP = new GridLMDP(3, 15, 30, -0.03);
//		GridLMDP *gridLMDP = new GridLMDP(1, 20, 30, -0.03);
//		GridLMDP *gridLMDP = new GridLMDP(1, 25, 0, -0.03);

		gridLMDP->set_slack(0.0f, 0.0f, 0.0f);
//		gridLMDP->set_default_conditional_preference();
		gridLMDP->set_split_conditional_preference();

		PolicyMap *policy = nullptr;

		if (cudaVersion) {
			LVICuda solver(0.0001);
			policy = solver.solve(gridLMDP);
		} else {
//			LVI solver(0.0001, false);
			LVI solver(0.0001, true);
			policy = solver.solve(gridLMDP);
		}

		gridLMDP->print(policy);

		delete policy;

		std::vector<std::unordered_map<const State *, double> > V;

		// Solve the Grid MDP using VI with various weights, and save the values of the initial state each time.
		if (viWeightCheck) {
			for (double weight = 0.0; weight <= 0.8; weight += 0.1) {
				gridLMDP->set_rewards_weights({0.2, weight, 0.8 - weight});

				MDPValueIteration viSolver(0.0001);
				PolicyMap *viPolicy = viSolver.solve(gridLMDP);

				compute_V_pi(dynamic_cast<const StatesMap *>(gridLMDP->get_states()),
							dynamic_cast<const ActionsMap *>(gridLMDP->get_actions()),
							gridLMDP->get_state_transitions(),
							dynamic_cast<const FactoredRewards *>(gridLMDP->get_rewards()),
							gridLMDP->get_horizon(),
							0.0001,
							viPolicy,
							V);

				std::cout << "Weight: [" << weight << ", " << (1.0 - weight) << "]:" << std::endl;
				gridLMDP->print(viPolicy);

				delete viPolicy;
			}
		}

		delete gridLMDP;

	}

	return 0;
}
