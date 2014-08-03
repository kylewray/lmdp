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


#include "../../librbr/librbr/include/core/states/state.h"

#include "../../losm/losm/include/losm_node.h"

class LOSMState : public State {
public:
	/**
	 * The default constructor of the LOSMState object.
	 * @param	other	A LOSM node to copy.
	 * @param	dir		The direction the agent is facing (with or against the edge ordering).
	 */
	LOSMState(const LOSMNode *other, bool dir);

	/**
	 * The copy constructor of the LOSMState object.
	 * @param	other		The state to copy.
	 */
	LOSMState(const LOSMState &other);

	/**
	 * The default deconstructor of the LOSMState object.
	 */
	virtual ~LOSMState();

	/**
	 * Overload the equals operator to set this state equal to the state provided.
	 * @param	other		The state to copy.
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
	 * The LOSM node corresponding to this state.
	 */
	LOSMNode *node;

	/**
	 * The direction the agent is facing. This is true if the agent is following the order in which
	 * edges are defined. It is false if the agent is following the opposite order. At intersections,
	 * the agent may ignore this and turn any way it wants.
	 */
	bool direction;

};


#endif // LOSM_STATE_H
