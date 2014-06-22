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
#include "../include/losm_state.h"

#include "../../librbr/librbr/include/core/states/finite_states.h"
#include "../../librbr/librbr/include/core/actions/finite_actions.h"
#include "../../librbr/librbr/include/core/state_transitions/finite_state_transitions.h"
#include "../../librbr/librbr/include/core/rewards/factored_rewards.h"
#include "../../librbr/librbr/include/core/initial.h"
#include "../../librbr/librbr/include/core/horizon.h"

#include "../../librbr/librbr/include/core/actions/named_action.h"

#include "../../librbr/librbr/include/core/states/state_exception.h"

#include "../../losm/losm/include/losm_exception.h"

LOSMMDP::LOSMMDP(std::string nodesFilename, std::string edgesFilename, std::string landmarksFilename)
{
	LOSM *losm = new LOSM(nodesFilename, edgesFilename, landmarksFilename);

	create_states(losm);
	create_actions(losm);
	create_state_transitions(losm);
	create_rewards(losm);
	create_misc(losm);

	delete losm;
}

LOSMMDP::~LOSMMDP()
{ }

void LOSMMDP::create_states(LOSM *losm)
{
	states = new FiniteStates();

	// Create the set of states from the LOSM object's nodes.
	for (const LOSMNode *node : losm->get_nodes()) {
		LOSMState *state = new LOSMState(node, false);
		((FiniteStates *)states)->add(state);

		state = new LOSMState(node, true);
		((FiniteStates *)states)->add(state);
	}
}

void LOSMMDP::create_actions(LOSM *losm)
{
	forward = new NamedAction("Forward");
	right = new NamedAction("Right");
	left = new NamedAction("Left");
	uTurn = new NamedAction("U-Turn");

	actions = new FiniteActions();
	((FiniteActions *)actions)->add(forward);
	((FiniteActions *)actions)->add(right);
	((FiniteActions *)actions)->add(left);
	((FiniteActions *)actions)->add(uTurn);
}

void LOSMMDP::create_state_transitions(LOSM *losm)
{
	// Use a temporary variable to find the forward action.
	const Action *findAction = new NamedAction("Forward");
	const Action *forward = ((FiniteActions *)actions)->get(findAction->hash_value());
	delete findAction;

	// Use a temporary variable to find the left action.
	findAction = new NamedAction("Left");
	const Action *left = ((FiniteActions *)actions)->get(findAction->hash_value());
	delete findAction;

	// Use a temporary variable to find the right action.
	findAction = new NamedAction("Right");
	const Action *right = ((FiniteActions *)actions)->get(findAction->hash_value());
	delete findAction;

	// Use a temporary variable to find the right action.
	findAction = new NamedAction("U-Turn");
	const Action *uTurn = ((FiniteActions *)actions)->get(findAction->hash_value());
	delete findAction;

	for (const LOSMEdge *edge : losm->get_edges()) {
		// Get the two nodes.
		const LOSMNode *n1 = edge->get_node_1();
		const LOSMNode *n2 = edge->get_node_2();

		// Use a temporary variable to find the first state.
		const LOSMState *findState = new LOSMState(n1, false);
		const State *s1 = ((FiniteStates *)states)->get(findState->hash_value());
		delete findState;

		// Use a temporary variable to find the first state in the opposite direction.
		findState = new LOSMState(n1, true);
		const State *s1r = ((FiniteStates *)states)->get(findState->hash_value());
		delete findState;

		// Use a temporary variable to find the second state.
		findState = new LOSMState(n2, false);
		const State *s2 = ((FiniteStates *)states)->get(findState->hash_value());
		delete findState;

		// Use a temporary variable to find the second state in the opposite direction.
		findState = new LOSMState(n2, true);
		const State *s2r = ((FiniteStates *)states)->get(findState->hash_value());
		delete findState;

		// Compute the base probability of getting stuck at a node.
		double stuck = FRICTION * (edge->get_distance() / D_0) *
				(L_0 / edge->get_lanes()) * (S_0 / edge->get_speed_limit());

		// Now that we have all four states, we need to first link them such that the direction
		// is preserved when taking each action given node 2 has degree 2, 3, or 4.
		if (n2->get_degree() == 2) {
			((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, stuck);
			((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s2, 1.0 - stuck);
		} else if (n2->get_degree() == 3) {
			// Get the list of neighbors.
			std::vector<const LOSMNode *> neighbors;
			losm->get_neighbors(n2, neighbors);

			// Figure out which one n1 is.
			int n3Index = 0;
			if (n1 == neighbors[0]) {
				n3Index = 1;
			}

			// NOTE: THIS MAY BE COMPLETELY WRONG WITH THE WHOLE n2 and s1 thing going on here...

			// Check if n1 is on the left. If it is, it means it is more likely you'll get stuck here.
			if (check_left(n2, neighbors[n3Index], n1)) {
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s1, 2.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s2, 1.0 - 3.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, uTurn, s1, 1.0);
			} else {
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s2, 1.0 - 3.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, uTurn, s1, 1.0);
			}
		} else if (n2->get_degree() == 4) {
			// Get the list of neighbors.
			std::vector<const LOSMNode *> neighbors;
			losm->get_neighbors(n2, neighbors);

			// Figure out which one n1 is.
			int n3Index = 0;
			int n4Index = 1;
			if (n1 == neighbors[0]) {
				n3Index = 1;
				n4Index = 2;
			} else if (n1 == neighbors[1]){
				n4Index = 2;
			}

			// Check if n1 is on the left, center, or right. Based on this, it goes from more likely
			// to get stuck, to less likely to get stuck.
			bool leftOfN3 = check_left(n2, neighbors[n3Index], n1);
			bool leftOfN4 = check_left(n2, neighbors[n4Index], n1);
			if (leftOfN3 && leftOfN4) {
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s1, 3.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s2, 1.0 - 6.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, uTurn, s1, 1.0);
			} else if ((leftOfN3 && !leftOfN4) || (!leftOfN3 && leftOfN4)) {
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, 2.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s2, 1.0 - 6.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, uTurn, s1, 1.0);
			} else {
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s1, stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, right, s2, 1.0 - 6.0 * stuck);
				((FiniteStateTransitions *)stateTransitions)->set(s1, left, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, forward, s1, 1.0);
				((FiniteStateTransitions *)stateTransitions)->set(s1, uTurn, s1, 1.0);
			}
		}

		// Now that we have all four states, we need to first link them such that the direction
		// is preserved when taking each action given node 1 has degree 2, 3, or 4.
		if (n1->get_degree() == 2) {
			((FiniteStateTransitions *)stateTransitions)->set(s1r, forward, s1r, stuck);
			((FiniteStateTransitions *)stateTransitions)->set(s1r, forward, s2r, 1.0 - stuck);
		} else if (n1->get_degree() == 3) {

		} else if (n1->get_degree() == 4) {

		}
	}
}

void LOSMMDP::create_rewards(LOSM *losm)
{

}

void LOSMMDP::create_misc(LOSM *losm)
{

}

bool LOSMMDP::check_left(const LOSMNode *n1, const LOSMNode *n2, const LOSMNode *n3)
{
	return (n2->get_x() - n1->get_x()) * (n3->get_y() - n1->get_y()) -
			(n2->get_y() - n1->get_y()) * (n3->get_x() - n1->get_x()) > 0;
}

double LOSMMDP::distance(const LOSMNode *n1, const LOSMNode *n2, const LOSMNode *n3)
{
	double dx = n2->get_x() - n1->get_x();
	double dy = n2->get_y() - n1->get_y();
	return std::fabs(dy * n3->get_x() - dx * n3->get_y() - n1->get_x() * n2->get_y() + n1->get_y() * n2->get_x()) /
			std::sqrt(dx * dx + dy * dy);
}
