""" The MIT License (MIT)
 
    Copyright (c) 2014 Kyle Wray

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from __future__ import print_function

import numpy as np


class MDP(object):
    """ A Markov Decision Process class. """

    def __init__(self):
        """ The constructor for the MDP class. """

        # A set of states.
        self.S = set()

        # A set of actions.
        self.A = set()

        # A transition function which maps a tuple (s, a, s') to a probability.
        self.P = dict()

        # A list of reward functions, each map a tuple (s, a, s') to a real value.
        self.R = list()

        # The initial state (if finite), horizon (if finite), and the discount factor (if infinite).
        self.s0 = None
        self.horizon = 0
        self.gamma = 0.9


    def value_iteration(self, epsilon=0.0000001, maxIterations=None):
        """ Perform value iteration to solve the MDP. This is for infinite horizon MDPs.

            Based on the book "Artificial Intelligence: A Modern Approach," Third Edition,
            by Russell and Norvig.

            Parameters:
                epsilon -- The maximum error allowed in the utility for any state-action pair.
                    Default is 0.00001.
                maxIterations -- The maximum number of iterations. None implies infinite horizon.

            Returns:
                pi -- The dictionary mapping states to actions, i.e., the policy.
                V -- The dictionary mapping states to expected long-term utilities.
        """

        # Get the number of reward functions.
        K = len(self.R)

        # Initialize the value functions for the previous and current time steps.
        Vp = [{s: 0.0 for s in self.S} for i in range(K)]
        V = [{s: 0.0 for s in self.S} for i in range(K)]

        # Initialize the policy variable.
        pi = {s: None for s in self.S}

        # Two variables to check for convergence.
        delta = epsilon + 1.0
        i = 0

        # While the max 1-norm of the state-utility estimates is larger than the tolerance (see proof)...
        while delta > epsilon * (1.0 - self.gamma) / self.gamma:
            V = [Vp[k].copy() for k in range(K)]

            # ------- DEBUG -------
            for y in range(int(np.sqrt(len(self.S)))):
                for x in range(int(np.sqrt(len(self.S)))):
                    print("%.6f\t" % (V[1][(x, y)]), end="")
                print()
            print()
            # happy = raw_input()
            # ------- DEBUG -------

            delta = 0.0

            # For every state in the MDP...
            for s in self.S:
                # We will begin with all actions being possible, and slowly reduce the size of the set.
                AStar = self.A.copy()

                # For every reward in the MDP...
                for k in range(K):
                    # This variable contains the best actions for the current time step.
                    ATmp = {}

                    # Compute the Bellman Equation and set it as the new estimate for the state.
                    VStar = -np.Infinity

                    # Note: We only consider actions which were a tie for the previous value function computation at this state.
                    for a in AStar:
                        VTmp = sum([self.P[(s, a, sp)] * (self.R[k][(s, a, sp)] + self.gamma * V[k][sp]) for sp in self.S])

                        if VTmp > VStar:
                            # If this is strictly better, then we prefer it.
                            VStar = VTmp
                            ATmp = {a}
                        elif VTmp == VStar:
                            # Else if this is a tie, add it to the list.
                            ATmp |= {a}

                    # Regardless of how many ties there were, set the value of the state.
                    Vp[k][s] = VStar

                    # Reduce the space of possible actions. Note: The smallest this can get is a set of size 1.
                    AStar &= ATmp

                    # Perform the max 1-norm and store it in delta.
                    if abs(Vp[k][s] - V[k][s]) > delta:
                        delta = abs(Vp[k][s] - V[k][s])

                # Pick one of the actions to take at this state.
                pi[s] = list(AStar)[0]

            print("Completed iteration %i with delta %f." % (i + 1, delta))
            if maxIterations != None and i >= maxIterations:
                break
            i += 1

        return pi, Vp
