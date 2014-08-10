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


#ifndef LOSM_STATE_H
#define LOSM_STATE_H


#include "../../librbr/librbr/include/core/states/indexed_state.h"

#include "../../losm/losm/include/losm_node.h"

#define NUM_TIREDNESS_LEVELS 3

/**
 * A custom LSOM state class which holds the two LOSM nodes (in order) and other relevant information.
 */
class LOSMState : public IndexedState {
public:
	/**
	 * The default constructor of the LOSMState object.
	 * @param	currentNode			The current LOSM node in which a decision must be made (an intersection).
	 * @param	previousNode		The previous LOSM node from which the agent originated (an intersection).
	 * @param	tirednessLevel		The level of tiredness from 0 to MAX_TIREDNESS - 1.
	 * @param	travelDistance		The distance (mi) from the current to previous nodes.
	 * @param	travelSpeedLimit	The time (h) from the current to previous nodes.
	 * @param	isPrimaryGoal		If this is a primary goal state or not.
	 * @param	isSecondaryGoal		If this is a secondary goal state or not.
	 */
	LOSMState(const LOSMNode *currentNode, const LOSMNode *previousNode, unsigned int tirednessLevel,
			float travelDistance, float travelSpeedLimit, bool isPrimaryGoal, bool isSecondaryGoal);

	/**
	 * The copy constructor of the LOSMState object.
	 * @param	other	The state to copy.
	 */
	LOSMState(const LOSMState &other);

	/**
	 * The default deconstructor of the LOSMState object.
	 */
	virtual ~LOSMState();

	/**
	 * Get the current LOSMNode.
	 * @return	The current LOSMNode.
	 */
	const LOSMNode *get_current() const;

	/**
	 * Get the previous LOSMNode.
	 * @return	The previous LOSMNode.
	 */
	const LOSMNode *get_previous() const;

	/**
	 * Get the level of tiredness.
	 * @return	The level of tiredness.
	 */
	unsigned int get_tiredness() const;

	/**
	 * Get the distance traveled.
	 * @return	The distance traveled.
	 */
	float get_distance() const;

	/**
	 * Get the speed limit traveled.
	 * @return	The speed limit traveled.
	 */
	float get_speed_limit() const;

	/**
	 * Return if this is a primary goal state or not.
	 * @return	Returns if this is a primary goal state or not.
	 */
	bool is_primary_goal_state() const;

	/**
	 * Return if this is a secondary goal state or not.
	 * @return	Returns if this is a secondary goal state or not.
	 */
	bool is_secondary_goal_state() const;

	/**
	 * Overload the equals operator to set this state equal to the state provided.
	 * @param	other	The state to copy.
	 * @return	The new version of this state.
	 */
	virtual State &operator=(const State &other);

	/**
	 * Returns a string representation of this state.
	 * @return	Returns the string representing this state.
	 */
	virtual std::string to_string() const;

	/**
	 * Returns a hash value used to quickly identify this state in a collection of states.
	 * @return	Returns the hash value of this state.
	 */
	virtual unsigned int hash_value() const;

private:
	/**
	 * The current LOSM node in which a decision must be made (an intersection).
	 */
	const LOSMNode *current;

	/**
	 * The previous LOSM node from which the agent originated.
	 */
	const LOSMNode *previous;

	/**
	 * The tiredness level from 0 to MAX_TIREDNESS - 1.
	 */
	unsigned int tiredness;

	/**
	 * The distance from the current node to the previous node in miles.
	 */
	float distance;

	/**
	 * The average speed limit along this road.
	 */
	float speedLimit;

	/**
	 * If this is a primary goal state or not.
	 */
	bool primaryGoalState;

	/**
	 * If this is a secondary goal state or not.
	 */
	bool secondaryGoalState;

};


#endif // LOSM_STATE_H
