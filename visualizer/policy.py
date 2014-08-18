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

import csv
import random as rnd


NUM_TIREDNESS_LEVELS = 2
TIREDNESS_PROBABILITY = 0.1


class Policy:
    """ A policy object specifically designed for LMDP and LOSM. """

    def __init__(self, policyFile):
        """ The constructor for the Policy object.

            Parameters:
                policyFile  -- The file name of the policy.
        """

        self.policy = dict()

        self._load(policyFile)

        self.initialState = None
        self.goalState = None
        self.currentState = None


    def _load(self, policyFile):
        """ Load the policy file provided.

            Parameters:
                policyFile -- The file name of the policy.
        """

        self.policy = dict()

        with open(policyFile, 'r') as f:
            reader = csv.reader(f, delimiter=',')
            for row in reader:
                if len(row) != 7:
                    print("Failed to parse 7 arguments for policy line: '" + \
                        ",".join(row) + "'.")
                    break

                state = (int(row[0]), int(row[1]), int(row[2]), row[3] == '1')
                action = (int(row[4]), int(row[5]), row[6] == '1')

                self.policy[state] = action


    def is_path_prepared(self):
        """ Return if a path is prepared or not.

            Parameters:
                True if all the necessary variables are defined, and False otherwise.
        """

        return self.initialState and self.goalState and self.currentState


    def set_path_scenario(self, initialState, goalState):
        """ Set the initial state and goal state. The policy path will be computed from
            these two states.

            Parameters:
                initialState    -- The initial state as a 4-tuple
                                    (start, end, tiredness, autonomy).
                goalState       -- The goal state as a 3-tuple
                                    (start, end, autonomy).
        """

        self.initialState = initialState
        self.goalState = goalState
        self.currentState = initialState


    def reset(self):
        """ Reset the internal current state. """

        self.currentState = self.initialState


    def set_current_state_tiredness(self, tiredness):
        """ Set the current state to one with a tiredness as defined. It clamps this value
            to guarantee it is valid.

            Parameters:
                tiredness -- The new tiredness value.
        """

        self.currentState[2] = max(0, min(NUM_TIREDNESS_LEVELS - 1, tiredness))


    def next(self, probabilistic=False):
        """ Determine the next state based on the policy.

            Parameters:
                probabilistic -- Optionally enable probabilistic tiredness updating.

            Returns:
                The next state as a 4-tuple (start, end, tiredness, autonomy).
                If the goal state has been reached, then return None.
        """

        tiredness = self.currentState[2]

        if probabilistic and tiredness < NUM_TIREDNESS_LEVELS - 1:
            tiredness += int(rnd.random() < TIREDNESS_PROBABILITY)

        self.currentState = (self.policy[self.currentState][0],
                            self.policy[self.currentState][1],
                            tiredness,
                            self.policy[self.currentState][2])

        if self.currentState[0] == self.goalState[0] and \
                self.currentState[1] == self.goalState[1] and \
                self.currentState[3] == self.goalState[2]:
            return None
        else:
            return self.currentState

