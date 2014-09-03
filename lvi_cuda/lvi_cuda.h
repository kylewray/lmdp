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
 * @param	m			The number of actions, in total, that are possible.
 * @param	A			A mapping of states-action pairs (n-m array) to a boolean if the action
 * 						is available at that state or not.
 * @param	d_T			A mapping of state-action-state triples (n-m-n array) to a
 * 						transition probability. (Device-side pointer.)
 * @param	d_Ri			A mapping of state-action-state triples (n-m-n array) to a reward.
 * 						(Device-side pointer.)
 * @param	d_Pj		The j-th partition, an array of z states. It is these states that will
 * 						be updated. (Device-side pointer.)
 * @param	d_pi		The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1). This will be modified. (Device-side pointer.)
 * @param	Rmin		The minimum reward possible, for use in computing the number
 * 						of iterations.
 * @param	Rmax		The maximum reward possible, for use in computing the number
 * 						of iterations.
 * @param	gamma		The discount factor in [0.0, 1.0).
 * @param	epsilon		The convergence criterion tolerance to within optimal.
 * @param	numBlocks	The number of CUDA blocks. Ensure that numBlocks * numThreads >= n.
 * @param	numThreads	The number of CUDA threads per block. Use 128, 256, or 512 (multiples of 32).
 * @param	Vi			The final value function, mapping states (n-array) to floats. Only the states
 * 						listed in Pj will be updated; the rest are essentially ViFixed.
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -2 if the number
 * 			of blocks and threads is less than the number of states; -3 if an error with
 * 			the CUDA functions arose.
 */
int lvi_cuda(unsigned int n, unsigned int z, unsigned int m, const bool *A,
		const float *d_T, const float *d_Ri, const unsigned int *d_Pj, unsigned int *d_pi,
		float Rmin, float Rmax, float gamma, float epsilon,
		unsigned int numBlocks, unsigned int numThreads,
		float *Vi);

/**
 * Initialize CUDA by transferring all of the constant MDP model information to the device.
 * @param	n			The number of states.
 * @param	m			The number of actions, in total, that are possible.
 * @param	T			A mapping of state-action-state triples (n-m-n array) to a
 * 						transition probability.
 * @param	d_T			A mapping of state-action-state triples (n-m-n array) to a
 * 						transition probability. (Device-side pointer.)
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -3 if an error with
 * 			the CUDA functions arose.
 */
int lvi_initialize_state_transitions(unsigned int n, unsigned int m, const float *T, float *&d_T);

/**
 * Initialize CUDA by transferring all of the constant MDP model information to the device.
 * @param	n			The number of states.
 * @param	m			The number of actions, in total, that are possible.
 * @param	R			A mapping of state-action-state triples (n-m-n array) to a reward.
 * @param	d_R			A mapping of state-action-state triples (n-m-n array) to a reward.
 * 						(Device-side pointer.)
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -3 if an error with
 * 			the CUDA functions arose.
 */
int lvi_initialize_rewards(unsigned int n, unsigned int m, const float *R, float *&d_R);

/**
 * Initialize CUDA by transferring all of the constant partition information to the device.
 * @param	z			The number of states in the j-th partition.
 * @param	Pj			The j-th partition, an array of z states.
 * @param	pi			The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1).
 * @param	d_Pj		The j-th partition, an array of z states. (Device-side pointer.)
 * @param	d_pi		The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1). (Device-side pointer.)
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -3 if an error with
 * 			the CUDA functions arose.
 */
int lvi_initialize_partition(unsigned int z,
		const unsigned int *Pj, const unsigned int *pi,
		unsigned int *&d_Pj, unsigned int *&d_pi);

/**
 * Get the policy by copying the device information to the policy pointer provided.
 * @param	z			The number of states in the j-th partition.
 * @param	pi			The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1).
 * @param	d_pi		The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1). (Device-side pointer.)
 * @return	Returns 0 upon success; -1 if invalid arguments were passed; -3 if an error with
 * 			the CUDA functions arose.
 */
int lvi_get_policy(unsigned int z, const unsigned int *d_pi, unsigned int *pi);

/**
 * Uninitialize CUDA by freeing all of the constant MDP model information on the device.
 * @param	d_T			A mapping of state-action-state triples (n-m-n array) to a
 * 						transition probability. (Device-side pointer.)
 * @param	k			The number of reward factors.
 * @param	d_R			A mapping of state-action-state triples (n-m-n array) to a reward.
 * 						(Device-side pointer.)
 * @param	ell			The number of partitions.
 * @param	d_P			The j partitions, an array of z states. (Device-side pointer.)
 * @param	d_pi		The resultant policy, mapping every state (n array) to an
 * 						action (in 0 to m-1). (Device-side pointer.)
 * @return	Returns 0 upon success.
 */
int lvi_uninitialize(float *&d_T,
		unsigned int k, float **&d_R,
		unsigned int ell, unsigned int **&d_P, unsigned int **&d_pi);


#endif // NSIGHT_LVI_CUDA_H
