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
		bool autonomyEnabled, float travelDistance, float travelSpeedLimit, bool isGoal, bool isAutonomyCapable)
{
	current = currentNode;
	previous = previousNode;
	tiredness = tirednessLevel;
	autonomy = autonomyEnabled;
	distance = travelDistance;
	speedLimit = travelSpeedLimit;
	goal = isGoal;
	autonomyCapable = isAutonomyCapable;
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

bool LOSMState::is_autonomy_enabled() const
{
	return autonomy;
}

float LOSMState::get_distance() const
{
	return distance;
}

float LOSMState::get_speed_limit() const
{
	return speedLimit;
}

bool LOSMState::is_goal() const
{
	return goal;
}

bool LOSMState::is_autonomy_capable() const
{
	return autonomyCapable;
}

State &LOSMState::operator=(const State &other)
{
	const LOSMState *state = dynamic_cast<const LOSMState *>(&other);
	if (state == nullptr) {
		std::cerr << "Failed to dynamically cast the other state to a LOSMState." << std::endl;
		throw StateException();
	}

	index = state->index;

	current = state->current;
	previous = state->previous;
	tiredness = state->tiredness;
	autonomy = state->autonomy;
	distance = state->distance;
	speedLimit = state->speedLimit;
	goal = state->goal;
	autonomyCapable = state->autonomyCapable;

	return *this;
}

std::string LOSMState::to_string() const
{
	return IndexedState::to_string();
}

unsigned int LOSMState::hash_value() const
{
	return IndexedState::hash_value();
}
