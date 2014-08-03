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

LOSMState::LOSMState(const LOSMNode *other, bool dir)
{
	node = new LOSMNode(other->get_uid(), other->get_x(), other->get_y(), other->get_degree());
	direction = dir;
}

LOSMState::LOSMState(const LOSMState &other)
{
	*this = other;
}

LOSMState::~LOSMState()
{
	delete node;
}

State &LOSMState::operator=(const State &other)
{
	const LOSMState *state = static_cast<const LOSMState *>(&other);
	if (state == nullptr) {
		std::cerr << "Failed to dynamically cast the other state to a LOSMState." << std::endl;
		throw StateException();
	}

	delete node;

	node = new LOSMNode(state->node->get_uid(), state->node->get_x(), state->node->get_y(), state->node->get_degree());
	direction = state->direction;

	return *this;
}

std::string LOSMState::to_string() const
{
	std::string result = "Node ";
	result += node->get_uid();
	result += " has position (";
	result += node->get_x();
	result += ", ";
	result += node->get_y();
	result += ") with degree ";
	result += node->get_degree();
	result += ".";
	return result;
}

unsigned int LOSMState::hash_value() const
{
	unsigned int hash = 7;
	hash = 31 * hash + node->get_uid();
	hash = 31 * hash + (int)direction;
	return hash;
}
