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

#include <iostream>

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

//	GridMDP *gridMDP = new GridMDP(0, 5, 0, -0.03);
	GridMDP *gridMDP = new GridMDP(1, 8, 10, -0.03);

	LVMaxValueIteration solver(0.0001);

	std::vector<double> delta;
	delta.push_back(0.0);
	delta.push_back(0.2);
	delta.push_back(0.0);

	PolicyMap *policy = solver.solve(gridMDP, delta);

	gridMDP->print(policy);

	delete gridMDP;

	//*/

	return 0;
}
