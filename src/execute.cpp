/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Wray
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


#include "../include/losm_mdp.h"
#include "../include/grid_mdp.h"

#include "../include/lvmax_value_iteration.h"

#include "../../losm/losm/include/losm_exception.h"

#include "../../librbr/librbr/include/management/raw_file.h"

#include "../../librbr/librbr/include/core/core_exception.h"

#include "../../librbr/librbr/include/core/states/state_utilities.h"

#include "../../librbr/librbr/include/core/actions/action_utilities.h"
#include "../../librbr/librbr/include/core/actions/action_exception.h"

#include <iostream>
#include <unordered_map>

PolicyMap *convert_policy(PolicyMap *policy, const MDP *src, const MDP *dest)
{
	const StatesMap *srcS = dynamic_cast<const StatesMap *>(src->get_states());
	const ActionsMap *srcA = dynamic_cast<const ActionsMap *>(src->get_actions());

	const StatesMap *destS = dynamic_cast<const StatesMap *>(dest->get_states());
	const ActionsMap *destA = dynamic_cast<const ActionsMap *>(dest->get_actions());

	if (srcS == nullptr || srcA == nullptr || destS == nullptr || destA == nullptr) {
		throw CoreException();
	}

	// Maps src to dest.
	std::unordered_map<const Action *, const Action *> actionsMap;

	// First figure out the mapping of actions: src to dest.
	int i = 0;

	for (auto action : *destA) {
		const Action *destAction = resolve(action);
		const Action *srcAction = srcA->get(i);

		actionsMap[srcAction] = destAction;

		i++;
	}

	// Now, iterate over all the states. For each one, we will redefine the policy provided.
	PolicyMap *p = new PolicyMap();

	i = 0;

	for (auto state : *destS) {
		const State *destState = resolve(state);
		const State *srcState = srcS->get(i);

		p->set(destState, actionsMap[policy->get(srcState)]);

		i++;
	}

	return p;
}

int main(int argc, char *argv[]) {
	/* LOSM MDP Version.

	if (argc != 4) {
		std::cerr << "Please specify nodes, edges, and landmarks data files." << std::endl;
		return -1;
	}

	LOSMMDP *losmMDP = nullptr;
	try {
		losmMDP = new LOSMMDP(argv[1], argv[2], argv[3]);
	} catch (LOSMException &err) {
		std::cerr << "Failed to load the files provided." << std::endl;
		return -1;
	}

	LVMaxMDPSolver solver;
	// TODO: Finish solver...

	delete losmMDP;

	//*/



	//* Grid World Version.

	RawFile rawFile;

//	GridMDP *gridMDPOriginal = new GridMDP(0, 5, 0, -0.03);
//	GridMDP *gridMDPOriginal = new GridMDP(1, 8, 10, -0.03);
	GridMDP *gridMDPOriginal = new GridMDP(1, 20, 30, -0.03);
//	GridMDP *gridMDPOriginal = new GridMDP(1, 25, 0, -0.03);

//	rawFile.save_raw_mdp(gridMDPOriginal, "grid_0_5_0.raw_mdp");
//	rawFile.save_raw_mdp(gridMDPOriginal, "grid_1_8_10.raw_mdp");
//	rawFile.save_raw_mdp(gridMDPOriginal, "grid_1_20_30.raw_mdp");
//	rawFile.save_raw_mdp(gridMDPOriginal, "grid_1_25_0.raw_mdp");

//	GridMDP *gridMDPFast = (GridMDP *)rawFile.load_raw_mdp("grid_0_5_0.raw_mdp");
//	GridMDP *gridMDPFast = (GridMDP *)rawFile.load_raw_mdp("grid_1_8_10.raw_mdp");
	GridMDP *gridMDPFast = (GridMDP *)rawFile.load_raw_mdp("grid_1_20_30.raw_mdp");
//	GridMDP *gridMDPFast = (GridMDP *)rawFile.load_raw_mdp("grid_1_25_0.raw_mdp");

	LVMaxValueIteration solver(0.0001);

	std::vector<double> delta;
	delta.push_back(0.0);
	delta.push_back(0.5);
	delta.push_back(0.0);

	PolicyMap *policyFast = solver.solve(gridMDPFast, delta, true);
	PolicyMap *policyOriginal = convert_policy(policyFast, gridMDPFast, gridMDPOriginal);

	gridMDPOriginal->print(policyOriginal);

	delete policyOriginal;
	delete policyFast;
	delete gridMDPOriginal;
	delete gridMDPFast;

	//*/

	return 0;
}
