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
#include "../include/lvi_nova.h"

#include "../../losm/losm/include/losm_exception.h"

#include "../../librbr/librbr/include/management/raw_file.h"

#include "../../librbr/librbr/include/core/core_exception.h"

#include "../../librbr/librbr/include/core/states/state_utilities.h"

#include "../../librbr/librbr/include/core/actions/action_utilities.h"
#include "../../librbr/librbr/include/core/actions/action_exception.h"

#include <iostream>
#include <unordered_map>

int main(int argc, char *argv[]) {
	//* LOSM MDP Version.

	if (argc != 9) {
		std::cerr << "Please specify nodes, edges, and landmarks data files, as well as the initial and goal nodes' UIDs, plus the policy output file." << std::endl;
		return -1;
	}

	LOSMMDP *losmMDP = nullptr;
	try {
		losmMDP = new LOSMMDP(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
	} catch (LOSMException &err) {
		std::cerr << "Failed to load the files provided." << std::endl;
		return -1;
	}

	losmMDP->set_slack(0.0f, 0.0f);

//	losmMDP->set_uniform_conditional_preference();
	losmMDP->set_tiredness_conditional_preference();

	PolicyMap *policy = nullptr;

//	LVI solver(0.0001, false);
	LVI solver(0.0001, true);

	policy = solver.solve(losmMDP);

	losmMDP->save_policy(policy, argv[8], solver.get_V());
//	policy->save(argv[8]);

	delete policy;
	delete losmMDP;

	//*/



	/* Grid World Version.

	RawFile rawFile;

//	GridLMDP *gridLMDP = new GridLMDP(0, 5, 0, -0.03);
	GridLMDP *gridLMDP = new GridLMDP(0, 10, 0, -0.03);
//	GridLMDP *gridLMDP = new GridLMDP(1, 8, 10, -0.03);
//	GridLMDP *gridLMDP = new GridLMDP(3, 15, 30, -0.03);
//	GridLMDP *gridLMDP = new GridLMDP(1, 20, 30, -0.03);
//	GridLMDP *gridLMDP = new GridLMDP(1, 25, 0, -0.03);

	gridLMDP->set_slack(0.0f, 0.0f, 0.0f);
//	gridLMDP->set_default_conditional_preference();
	gridLMDP->set_split_conditional_preference();

	PolicyMap *policy = nullptr;

//	LVI solver(0.0001, false);
	LVI solver(0.0001, true);

	policy = solver.solve(gridLMDP);
	gridLMDP->print(policy);

//	delete policy;

//	LVINova novaSolver(0.0001);
//	policy = novaSolver.solve(gridLMDP);
//	gridLMDP->print(policy);

	delete policy;
	delete gridLMDP;

	//*/

	return 0;
}
