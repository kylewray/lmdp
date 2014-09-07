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


#include "lvi_cuda.h"

#include <cmath>

#include <stdio.h>

// This is not C++0x, unfortunately.
#define nullptr NULL

// This is determined by hardware, so what is below is a 'safe' guess. If this is
// off, the program might return 'nan' or 'inf'.
#define FLT_MAX 1e+35

__global__ void lvi_bellman_update(unsigned int n, unsigned int z, unsigned int m,
		const bool *A, const unsigned int *Pj, const float *T, const float *Ri, unsigned int *pi,
		float gamma, const float *Vi,
		float *ViPrime)
{
	// The current state as a function of the blocks and threads.
	int s;

	// The intermediate Q(s, a) value.
	float Qsa;

	// The 1-d index version of the 3-d arrays in the innermost loop.
	int k;

	// Compute the index of the state. Return if it is beyond the state.
	s = blockIdx.x * blockDim.x + threadIdx.x;
	if (s >= z) {
		return;
	}

	// Nvidia GPUs follow IEEE floating point standards, so this should be safe.
	ViPrime[Pj[s]] = -FLT_MAX;

	// Compute max_{a in A} Q(s, a).
	for (int a = 0; a < m; a++) {
		// Skip this action if it is locked.
		if (!A[s * m + a]) {
			continue;
		}

		// Compute Q(s, a) for this action.
		Qsa = 0.0f;
		for (int sp = 0; sp < n; sp++) {
			k = Pj[s] * m * n + a * n + sp;
			Qsa += T[k] * (Ri[k] + gamma * Vi[sp]);
		}

		if (a == 0 || Qsa > ViPrime[Pj[s]]) {
			ViPrime[Pj[s]] = Qsa;
			pi[s] = a;
		}
	}
}

int lvi_initialize_state_transitions(unsigned int n, unsigned int m, const float *T, float *&d_T)
{
	// Ensure the data is valid.
	if (n == 0 || m == 0 || T == nullptr) {
		return -1;
	}

	// Allocate the memory on the device.
	if (cudaMalloc(&d_T, n * m * n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the state transitions.");
		return -3;
	}

	// Copy the data from the host to the device.
	if (cudaMemcpy(d_T, T, n * m * n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the state transitions.");
		return -3;
	}

	return 0;
}

int lvi_initialize_rewards(unsigned int n, unsigned int m, const float *R, float *&d_R)
{
	// Ensure the data is valid.
	if (n == 0 || m == 0 || R == nullptr) {
		return -1;
	}

	// Allocate the memory on the device.
	if (cudaMalloc(&d_R, n * m * n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the rewards.");
		return -3;
	}

	// Copy the data from the host to the device.
	if (cudaMemcpy(d_R, R, n * m * n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the rewards.");
		return -3;
	}

	return 0;
}

int lvi_initialize_partition(unsigned int z,
		const unsigned int *Pj, const unsigned int *pi,
		unsigned int *&d_Pj, unsigned int *&d_pi)
{
	// Ensure the data is valid.
	if (z == 0 || Pj == nullptr || pi == nullptr) {
		return -1;
	}

	// Allocate the memory on the device.
	if (cudaMalloc(&d_Pj, z * sizeof(unsigned int)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the partition array.");
		return -3;
	}

	if (cudaMalloc(&d_pi, z * sizeof(unsigned int)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the policy (pi).");
		return -3;
	}

	// Copy the data from the host to the device.
	if (cudaMemcpy(d_Pj, Pj, z * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the partition array.");
		return -3;
	}

	if (cudaMemcpy(d_pi, pi, z * sizeof(unsigned int), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the policy (pi).");
		return -3;
	}

	return 0;
}

int lvi_get_policy(unsigned int z, const unsigned int *d_pi, unsigned int *pi)
{
	if (cudaMemcpy(pi, d_pi, z * sizeof(unsigned int), cudaMemcpyDeviceToHost) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from device to host for the policy (pi).");
		return -3;
	}

	return 0;
}

int lvi_uninitialize(float *&d_T,
		unsigned int k, float **&d_R,
		unsigned int ell, unsigned int **&d_P, unsigned int **&d_pi)
{
	cudaFree(d_T);

	for (int i = 0; i < k ; i++) {
		cudaFree(d_R[i]);
	}

	for (int j = 0; j < ell; j++) {
		cudaFree(d_P[j]);
		cudaFree(d_pi[j]);
	}

	return 0;
}

int lvi_cuda(unsigned int n, unsigned int z, unsigned int m, const bool *A,
		const float *d_T, const float *d_Ri, const unsigned int *d_Pj, unsigned int *d_pi,
		float Rmin, float Rmax, float gamma, float epsilon,
		unsigned int numBlocks, unsigned int numThreads,
		float *Vi)
{
	// The device pointers for the MDP: A, T, and R.
	bool *d_A;

	// The host and device pointers for the value functions: V and VPrime.
	float *d_Vi;
	float *d_ViPrime;

	// First, ensure data is valid.
	if (n == 0 || z == 0 || m == 0 || A == nullptr ||
			d_Pj == nullptr || d_T == nullptr || d_Ri == nullptr || d_pi == nullptr ||
			gamma < 0.0f || gamma >= 1.0f) {
		return -1;
	}

	// Also ensure that there are enough blocks and threads to run the program.
	if (numBlocks * numThreads < z) {
		return -2;
	}

	// Next, determine how many iterations it will have to run. Then, multiply that by 10.
	int iterations = max(10, (int)std::ceil(std::log(2.0 * (Rmax - Rmin) / (epsilon * (1.0 - gamma)) / std::log(1.0 / gamma))));

	// Allocate the device-side memory.
	if (cudaMalloc(&d_A, z * m * sizeof(bool)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the restricted actions.");
		return -3;
	}

	if (cudaMalloc(&d_Vi, n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the value function.");
		return -3;
	}
	if (cudaMalloc(&d_ViPrime, n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the value function (prime).");
		return -3;
	}

//	// Assume that V and pi are initialized *properly* (either 0, or, with MPI, perhaps
//	// with previous V values).
//	for (int s = 0; s < n; s++) {
//		V[s] = 0.0f;
//		pi[s] = 0;
//	}

	// Copy the data from host to device.
	if (cudaMemcpy(d_A, A, z * m * sizeof(bool), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the restricted actions.");
		return -3;
	}

	if (cudaMemcpy(d_Vi, Vi, n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the value function.");
		return -3;
	}
	if (cudaMemcpy(d_ViPrime, Vi, n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the value function (prime).");
		return -3;
	}

	// Execute value iteration for these number of iterations. For each iteration, however,
	// we will run the state updates in parallel.
//	printf("Total Number of Iterations: %i\n", iterations);
	for (int i = 0; i < iterations; i++) {
//		printf("Iteration %d / %d\n", i, iterations);
//		printf("Blocks: %d\nThreads: %d\nGamma: %f\nn: %d\nm: %d\n", numBlocks, numThreads, gamma, n, m);

		if (i % 2 == 0) {
			lvi_bellman_update<<< numBlocks, numThreads >>>(n, z, m, d_A, d_Pj, d_T, d_Ri, d_pi, gamma, d_Vi, d_ViPrime);
		} else {
			lvi_bellman_update<<< numBlocks, numThreads >>>(n, z, m, d_A, d_Pj, d_T, d_Ri, d_pi, gamma, d_ViPrime, d_Vi);
		}
	}

	// Copy the final result, both V and pi, from device to host.
	if (iterations % 2 == 1) {
		if (cudaMemcpy(Vi, d_Vi, n * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
			fprintf(stderr, "Error[lvi_cuda]: %s",
					"Failed to copy memory from device to host for the value function.");
			return -3;
		}
	} else {
		if (cudaMemcpy(Vi, d_ViPrime, n * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
			fprintf(stderr, "Error[lvi_cuda]: %s",
					"Failed to copy memory from device to host for the value function (prime).");
			return -3;
		}
	}

//	for (int s = 0; s < n; s++) {
//		printf("V[%d] =   %f\t", s, V[s]);
//		if (s % 8 == 7) {
//			printf("\n");
//		}
//	}

	// Free the device-side memory.
	cudaFree(d_A);

	cudaFree(d_Vi);
	cudaFree(d_ViPrime);

	return 0;
}
