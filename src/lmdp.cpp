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


#include "../include/lmdp.h"

#include "../../librbr/librbr/include/core/rewards/reward_exception.h"

LMDP::LMDP()
{ }

LMDP::LMDP(States *S, Actions *A, StateTransitions *T, FactoredRewards *R, Initial *s, Horizon *h,
		std::vector<float> *d, std::vector<std::vector<State *> > *P,
		std::vector<std::vector<unsigned int> > *o) : MDP(S, A, T, R, h)
{
	for (float val : *d) {
		delta.push_back(val);
	}

	for (std::vector<State *> p : *P) {
		partition.push_back(p);
	}

	for (std::vector<unsigned int> r : *o) {
		ordering.push_back(r);
	}
}

LMDP::~LMDP()
{
	delta.clear();
	partition.clear();
	ordering.clear();
}

FactoredRewards *LMDP::get_rewards()
{
	FactoredRewards *R = dynamic_cast<FactoredRewards *>(rewards);
	if (R == nullptr) {
		throw RewardException();
	}
	return R;
}

void LMDP::set_slack(const std::vector<float> &d)
{
	delta.clear();
	for (float val : d) {
		delta.push_back(val);
	}
}

std::vector<float> &LMDP::get_slack()
{
	return delta;
}

void LMDP::set_partitions(const std::vector<std::vector<State *> > &P)
{
	partition.clear();
	for (std::vector<State *> p : P) {
		partition.push_back(p);
	}
}

std::vector<std::vector<State *> > &LMDP::get_partitions()
{
	return partition;
}

void LMDP::set_orderings(const std::vector<std::vector<unsigned int> > &o)
{
	ordering.clear();
	for (std::vector<unsigned int> r : o) {
		ordering.push_back(r);
	}
}

std::vector<std::vector<unsigned int> > &LMDP::get_orderings()
{
	return ordering;
}
