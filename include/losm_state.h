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
	 * @param	autonomyEnabled		If the autonomy is enabled or not.
	 * @param	travelDistance		The distance (mi) from the current to previous nodes.
	 * @param	travelSpeedLimit	The time (h) from the current to previous nodes.
	 * @param	isGoal				If this is a goal state or not.
	 * @param	isAutonomyCapable	If this is an autonomy capable state or not.
	 * @param	currentStepNode		One edge step from the current node towards the previous node. Used for animations later.
	 * @param	previousStepNode	One edge step from the previous node towards the current node. Used for animations later.
	 */
	LOSMState(const LOSMNode *currentNode, const LOSMNode *previousNode, unsigned int tirednessLevel,
			bool autonomyEnabled, float travelDistance, float travelSpeedLimit,
			bool isGoalState, bool isAutonomyCapableState,
			const LOSMNode *currentStepNode, const LOSMNode *previousStepNode);

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
	 * Get if the autonomy is enabled or not.
	 * @return	If the autonomy is enabled or not.
	 */
	bool get_autonomy() const;

	/**
	 * Get the uniqueness index.
	 * @return	The uniqueness index.
	 */
	unsigned int get_uniqueness_index() const;

	/**
	 * Reset the uniqueness counters.
	 */
	static void reset_uniqueness_counters();

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
	 * Return if this is a goal state or not.
	 * @return	Returns if this is a goal state or not.
	 */
	bool is_goal() const;

	/**
	 * Return if this is an autonomy capable state or not.
	 * @return	Returns if this is an autonomy capable state or not.
	 */
	bool is_autonomy_capable() const;

	/**
	 * Get the current step node.
	 * @return	The current step node.
	 */
	const LOSMNode *get_current_step() const;

	/**
	 * Get the previous step node.
	 * @return	The previous step node.
	 */
	const LOSMNode *get_previous_step() const;

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
	 * The previous LOSM node from which the agent originated.
	 */
	const LOSMNode *previous;

	/**
	 * The current LOSM node in which a decision must be made (an intersection).
	 */
	const LOSMNode *current;

	/**
	 * The tiredness level from 0 to MAX_TIREDNESS - 1.
	 */
	unsigned int tiredness;

	/**
	 * If autonomy has been enabled or not.
	 */
	bool autonomy;

	/**
	 * The uniqueness index, since the four above values can actually be found in
	 * multiple parts of a map due to multiple paths which lead between the same
	 * intersections.
	 */
	unsigned int uniquenessIndex;

	/**
	 * The static variable to keep track of the current uniqueness.
	 */
	static std::unordered_map<const LOSMNode *,
		std::unordered_map<const LOSMNode *,
			std::unordered_map<unsigned int,
				std::unordered_map<bool, unsigned int> > > > uniquenessCounter;

	/**
	 * The distance from the current node to the previous node in miles.
	 */
	float distance;

	/**
	 * The average speed limit along this road.
	 */
	float speedLimit;

	/**
	 * If this is a goal state or not.
	 */
	bool isGoal;

	/**
	 * If this is an autonomy capable state or not.
	 */
	bool isAutonomyCapable;

	/**
	 * One edge step from the current node towards the previous node. Used for animations later.
	 */
	const LOSMNode *currentStep;

	/**
	 * One edge step from the previous node towards the current node. Used for animations later.
	 */
	const LOSMNode *previousStep;

};


#endif // LOSM_STATE_H
