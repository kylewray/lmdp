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


#include "../include/losm_state.h"

#include "../../librbr/librbr/include/core/states/state_exception.h"

#include <iostream>

LOSMState::LOSMState(const LOSMNode *currentNode, const LOSMNode *previousNode, unsigned int tirednessLevel,
		float travelDistance, float travelSpeedLimit, bool isPrimaryGoal, bool isSecondaryGoal)
{
	current = currentNode;
	previous = previousNode;
//	current = new LOSMNode(currentNode->get_uid(),
//			currentNode->get_x(), currentNode->get_y(), currentNode->get_degree());
//	previous = new LOSMNode(previousNode->get_uid(),
//			previousNode->get_x(), previousNode->get_y(), previousNode->get_degree());
	tiredness = tirednessLevel;
	distance = travelDistance;
	speedLimit = travelSpeedLimit;
	primaryGoalState = isPrimaryGoal;
	secondaryGoalState = isSecondaryGoal;
}

LOSMState::LOSMState(const LOSMState &other)
{
	*this = other;
}

LOSMState::~LOSMState()
{
	delete current;
	delete previous;
}

const LOSMNode *LOSMState::get_current() const
{
	return current;
}

const LOSMNode *LOSMState::get_previous() const
{
	return previous;
}

unsigned int LOSMState::get_tiredness() const
{
	return tiredness;
}

float LOSMState::get_distance() const
{
	return distance;
}

float LOSMState::get_speed_limit() const
{
	return speedLimit;
}

bool LOSMState::is_primary_goal_state() const
{
	return primaryGoalState;
}

bool LOSMState::is_secondary_goal_state() const
{
	return secondaryGoalState;
}

State &LOSMState::operator=(const State &other)
{
	const LOSMState *state = dynamic_cast<const LOSMState *>(&other);
	if (state == nullptr) {
		std::cerr << "Failed to dynamically cast the other state to a LOSMState." << std::endl;
		throw StateException();
	}

	delete current;
	delete previous;

	index = state->index;

	current = new LOSMNode(state->current->get_uid(),
			state->current->get_x(),
			state->current->get_y(),
			state->current->get_degree());
	previous = new LOSMNode(state->previous->get_uid(),
			state->previous->get_x(),
			state->previous->get_y(),
			state->previous->get_degree());
	tiredness = state->tiredness;
	distance = state->distance;
	speedLimit = state->speedLimit;
	primaryGoalState = state->primaryGoalState;
	secondaryGoalState = state->secondaryGoalState;

	return *this;
}

std::string LOSMState::to_string() const
{
	std::string result = "Current Node ";
	result += current->get_uid();
	result += " has position (";
	result += current->get_x();
	result += ", ";
	result += current->get_y();
	result += ") with degree ";
	result += current->get_degree();

	result += "; Previous Node ";
	result += previous->get_uid();
	result += " has position (";
	result += previous->get_x();
	result += ", ";
	result += previous->get_y();
	result += ") with degree ";
	result += previous->get_degree();

	result += "; Tiredness of ";
	result += tiredness;

	result += "; Distance of ";
	result += distance;

	result += "; Speed Limit of ";
	result += speedLimit;

	if (primaryGoalState) {
		result += "; Primary Goal State";
	}

	if (secondaryGoalState) {
		result += "; Secondary Goal State";
	}

	result += ".";

	return result;
}

unsigned int LOSMState::hash_value() const
{
//	unsigned int hash = 7;
//	hash = 31 * hash + node->get_uid();
//	hash = 31 * hash + (int)direction;
//	return hash;

	return index;
}
