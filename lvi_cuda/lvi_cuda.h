/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Kyle Hollins Wray, University of Massachusetts
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


#ifndef NSIGHT_LVI_CUDA_H
#define NSIGHT_LVI_CUDA_H


/**
 * Execute value iteration for the infinite horizon MDP model specified, except this time we
 * limit actions taken at a state to within an array of available actions.
 * @param	n			The number of states.
 * @param	z			The number of states in the j-th partition.
 * @param	Pj			The j-th partition, an array of z states. These states will be updated.
 * @param	m			The number of actions, in total, that are possible.
 * @param	A			A mapping of states-action pairs (n-m array) to a boolean if the action
 * 						is available at that state or not.
 * @param	T			A mapping of state-action-state triples (n-m-n array) to a
 * 						transition probability.
 * @param	R			A mapping of state-action-state triples (n-m-n array) to a reward.
 * @param	Rmin		The minimum reward possible, for use in computing the number
 * 						of iterations.
 * @param	Rmax		The maximum reward possible, for use in computing the number
 * 						of iterations.
 * @param	gamma		The discount factor in [0.0, 1.0).
 * @param	epsilon		The convergence criterion tolerance to within optimal.
 * @param	Vi			The final value function, mapping states (n-array) to floats. Only the states
 * 						listed in Pj will be updated; the rest are essentially ViFixed.
 * @param	pi			The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1). This will be modified.
 * @param	numBlocks	The number of CUDA blocks. Ensure that numBlocks * numThreads >= n.
 * @param	numThreads	The number of CUDA threads per block. Use 128, 256, or 512 (multiples of 32).
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -2 if the number
 * 			of blocks and threads is less than the number of states.
 */
int lvi_cuda(unsigned int n, unsigned int z, unsigned int *Pj, unsigned int m, const bool *A,
		const float *T, const float *R, float Rmin, float Rmax, float gamma, float epsilon,
		float *Vi, unsigned int *pi, unsigned int numBlocks, unsigned int numThreads);


#endif // NSIGHT_LVI_CUDA_H
