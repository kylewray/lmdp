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

__global__ void lvi_bellman_update(unsigned int n, unsigned int z, const unsigned int *Pj,
		unsigned int m, const bool *A, const float *T, const float *R, float gamma,
		const float *Vi, float *ViPrime, unsigned int *pi)
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
			Qsa += T[k] * (R[k] + gamma * Vi[sp]);
		}

		if (a == 0 || Qsa > ViPrime[Pj[s]]) {
			ViPrime[Pj[s]] = Qsa;
			pi[s] = a;
		}
	}
}

int lvi_cuda(unsigned int n, unsigned int z, unsigned int *Pj, unsigned int m, const bool *A,
		const float *T, const float *R, float Rmin, float Rmax, float gamma, float epsilon,
		float *Vi, unsigned int *pi, unsigned int numBlocks, unsigned int numThreads)
{
	// The device pointers for the MDP: A, T, and R.
	bool *d_A;
	float *d_T;
	float *d_R;

	// The host and device pointers for the value functions: V and VPrime.
	float *d_Vi;
	float *d_ViPrime;

	// The partition of states as an array of state indices.
	unsigned int *d_Pj;

	// The device pointer for the final policy: pi.
	unsigned int *d_pi;

	// First, ensure data is valid.
	if (n == 0 || z == 0 || Pj == nullptr || m == 0 || A == nullptr || T == nullptr || R == nullptr ||
			gamma < 0.0f || gamma >= 1.0f || pi == nullptr) {
		return -1;
	}

	// Also ensure that there are enough blocks and threads to run the program.
	if (numBlocks * numThreads < z) {
		return -2;
	}

	// Next, determine how many iterations it will have to run. Then, multiply that by 10.
	int iterations = 100; // (int)std::ceil(std::log(2.0 * (Rmax - Rmin) / (epsilon * (1.0 - gamma)) / std::log(1.0 / gamma)));

	// Allocate the device-side memory.
	if (cudaMalloc(&d_A, z * m * sizeof(bool)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the restricted actions.");
		return -3;
	}
	if (cudaMalloc(&d_T, n * m * n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the state transitions.");
		return -3;
	}
	if (cudaMalloc(&d_R, n * m * n * sizeof(float)) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to allocate device-side memory for the rewards.");
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
	if (cudaMemcpy(d_T, T, n * m * n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the state transitions.");
		return -3;
	}
	if (cudaMemcpy(d_R, R, n * m * n * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from host to device for the rewards.");
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

	// Execute value iteration for these number of iterations. For each iteration, however,
	// we will run the state updates in parallel.
	printf("Total Number of Iterations: %i\n", iterations);
	for (int i = 0; i < iterations; i++) {
//		printf("Iteration %d / %d\n", i, iterations);
//		printf("Blocks: %d\nThreads: %d\nGamma: %f\nn: %d\nm: %d\n", numBlocks, numThreads, gamma, n, m);

		if (i % 2 == 0) {
			lvi_bellman_update<<< numBlocks, numThreads >>>(n, z, d_Pj, m, d_A, d_T, d_R, gamma, d_Vi, d_ViPrime, d_pi);
		} else {
			lvi_bellman_update<<< numBlocks, numThreads >>>(n, z, d_Pj, m, d_A, d_T, d_R, gamma, d_ViPrime, d_Vi, d_pi);
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
	if (cudaMemcpy(pi, d_pi, z * sizeof(unsigned int), cudaMemcpyDeviceToHost) != cudaSuccess) {
		fprintf(stderr, "Error[lvi_cuda]: %s",
				"Failed to copy memory from device to host for the policy (pi).");
		return -3;
	}

//	for (int s = 0; s < n; s++) {
//		printf("V[%d] =   %f\t", s, V[s]);
//		if (s % 8 == 7) {
//			printf("\n");
//		}
//	}

	// Free the device-side memory.
	cudaFree(d_A);
	cudaFree(d_T);
	cudaFree(d_R);

	cudaFree(d_Vi);
	cudaFree(d_ViPrime);

	cudaFree(d_Pj);

	cudaFree(d_pi);

	return 0;
}
