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


#ifndef LOSMMDP_H
#define LOSMMDP_H

#include "../../librbr/librbr/include/mdp/mdp.h"

#include "../../librbr/librbr/include/core/states/state.h"
#include "../../librbr/librbr/include/core/actions/action.h"

#include "../../losm/losm/include/losm.h"

#include "losm_state.h"

#define FRICTION 0.1
#define D_0 3.0
#define S_0 25.0
#define L_0 2.0

/**
 * A Multi-Objective Markov Decision Process (MOMDP) with lexicographic reward preferences.
 */
class LOSMMDP : public MDP {
public:
	/**
	 * The default constructor for the LOSMMDP class. It requires the specification of the three
	 * LOSM files to use.
	 * @param	nodesFilename 		The name of the LOSM nodes file to load.
	 * @param	edgesFilename 		The name of the LOSM edges file to load.
	 * @param	landmarksFilename	The name of the LOSM landmarks file to load.
	 */
	LOSMMDP(std::string nodesFilename, std::string edgesFilename, std::string landmarksFilename);

	/**
	 * A deconstructor for the LOSMMDP class.
	 */
	virtual ~LOSMMDP();

private:
	/**
	 * Create the MDP's states from the LOSM object provided.
	 * @param	losm	The Light-OSM object.
	 */
	void create_states(LOSM *losm);

	/**
	 * Create the MDP's actions from the LOSM object provided.
	 * @param	losm	The Light-OSM object.
	 */
	void create_actions(LOSM *losm);

	/**
	 * Create the MDP's state transitions from the LOSM object provided.
	 * @param	losm	The Light-OSM object.
	 */
	void create_state_transitions(LOSM *losm);

	/**
	 * Create the MDP's rewards from the LOSM object provided.
	 * @param	losm	The Light-OSM object.
	 */
	void create_rewards(LOSM *losm);

	/**
	 * Create the MDP's initial and horizon objects from the LOSM object provided.
	 * @param 	losm	The Light-OSM object.
	 */
	void create_misc(LOSM *losm);

	/**
	 * Check if the point (n3) is left of the line formed by two points (n1 and n2).
	 * @param	n1	The first point on the line.
	 * @param	n2	The second point on the line.
	 * @param	n3	The node to check.
	 * @return	If the point is on the left of the line.
	 */
	bool check_left(const LOSMNode *n1, const LOSMNode *n2, const LOSMNode *n3);

	/**
	 * Compute the distance from the point (n3) to the line formed by two points (n1 and n2).
	 * @param	n1	The first point on the line.
	 * @param	n2	The second point on the line.
	 * @param	n3	The node to check.
	 * @return	The distance from the point to the line.
	 */
	double distance(const LOSMNode *n1, const LOSMNode *n2, const LOSMNode *n3);

	/**
	 * The move forward action.
	 */
	const Action *forward;

	/**
	 * The move forward action.
	 */
	const Action *right;

	/**
	 * The move forward action.
	 */
	const Action *left;

	/**
	 * The move forward action.
	 */
	const Action *uTurn;

};


#endif // LOSMMDP_H
