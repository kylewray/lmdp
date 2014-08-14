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


#ifndef LOSMMDP_H
#define LOSMMDP_H

#include "lmdp.h"

#include "../../librbr/librbr/include/core/states/state.h"
#include "../../librbr/librbr/include/core/actions/action.h"

#include "../../losm/losm/include/losm.h"

#include "losm_state.h"

#include <vector>
#include <unordered_map>

/**
 * A Multi-Objective Markov Decision Process (MOMDP) with lexicographic reward preferences for a LOSM object.
 */
class LOSMMDP : public LMDP {
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

	/**
	 * Set the delta values.
	 * @param	d1	The first delta.
	 * @param	d2	The second delta.
	 */
	void set_slack(float d1, float d2);

	/**
	 * Define a 1-partition with the ordering (1, 2) for all states.
	 */
	void set_uniform_conditional_preference();

	/**
	 * Define a 2-partition with the ordering (1, 2) for all states without a tired driver, and
	 * (2, 1) for all states with a tired driver.
	 */
	void set_tiredness_conditional_preference();

private:
	/**
	 * Create the helper hash function for edges.
	 * @param	losm	The Light-OSM object.
	 */
	void create_edges_hash(LOSM *losm);

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
	 * Map directed path on the graph to merge nodes between intersections.
	 * @param	losm			The LOSM object.
	 * @param	current			The current node.
	 * @param	previous		The previous node.
	 * @param	distance		The distance (mi) traveled so far.
	 * @param	speedLimit		The weighted average speed limit (mi / h) traveled so far.
	 * @param	isGoal			If this is a primary goal state or not.
	 */
	const LOSMNode *map_directed_path(const LOSM *losm, const LOSMNode *current, const LOSMNode *previous,
			float &distance, float &speedLimit, bool &isGoal);

	/**
	 * Distance from a point to a line formed by two points.
	 * @param	x0	The point's x-axis.
	 * @param	y0	The point's y-axis.
	 * @param	x1	The first point on the line's x-axis.
	 * @param	y1	The first point on the line's y-axis.
	 * @param	x2	The second point on the line's x-axis.
	 * @param	y2	The second point on the line's y-axis.
	 * @return	The distance from the point to the line formed by the two points.
	 */
	float point_to_line_distance(float x0, float y0, float x1, float y1, float x2, float y2);

	/**
	 * A helper hash which maps the combination of two LOSMNode pointers to an edge.
	 * This is used to quickly reference the edges from the combination of LOSMNode UIDs.
	 */
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, const LOSMEdge *> > edgeHash;

	/**
	 * A map of possible successors from an action. Used to create videos.
	 */
//	std::unordered_map<const LOSMNode *, std::unordered_map<const Action *, std::vector<const LOSMNode *> > > successors;

};


#endif // LOSMMDP_H
