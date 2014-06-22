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

import random as rnd

from mdp import MDP


rnd.seed(3)


def create_mdp_three_directions(width, height):
    """ Create the grid world MDP without a reward function defined. This has 3 directions
        of movement possible.

        Parameters:
            width -- The width of the grid world.
            height -- The height of the grid world.

        Returns:
            The MDP object with states, actions, and transitions, but no reward.
    """

    mdp = MDP()

    # Create the states.
    for x in range(width):
        for y in range(height):
            mdp.S |= {(x, y)}

    # Create the actions.
    mdp.A = {"n", "s", "e", "w"}

    # Create the transition probabilities.
    for s in mdp.S:
        for a in mdp.A:
            for sp in mdp.S:
                mdp.P[(s, a, sp)] = 0.0

    for sx, sy in mdp.S:
        if sy > 0:
            mdp.P[((sx, sy), "n", (sx, sy - 1))] = 0.8
            if sx == 0:
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.2
            elif sx == width - 1:
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.2
            else:
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1
        else:
            if sx == 0:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1
            elif sx == width - 1:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
            else:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1

        if sy < height - 1:
            mdp.P[((sx, sy), "s", (sx, sy + 1))] = 0.8
            if sx == 0:
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.2
            elif sx == width - 1:
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.2
            else:
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1
        else:
            if sx == 0:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1
            elif sx == width - 1:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
            else:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1

        if sx > 0:
            mdp.P[((sx, sy), "w", (sx - 1, sy))] = 0.8
            if sy == 0:
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.2
            elif sy == height - 1:
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.2
            else:
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1
        else:
            if sy == 0:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1
            elif sy == height - 1:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
            else:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1

        if sx < width - 1:
            mdp.P[((sx, sy), "e", (sx + 1, sy))] = 0.8
            if sy == 0:
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.2
            elif sy == height - 1:
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.2
            else:
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1
        else:
            if sy == 0:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1
            elif sy == height - 1:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.9
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
            else:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1

    return mdp


def create_mdp_four_directions(width, height):
    """ Create the grid world MDP without a reward function defined. This has 4 directions
        of movement possible.

        Parameters:
            width -- The width of the grid world.
            height -- The height of the grid world.

        Returns:
            The MDP object with states, actions, and transitions, but no reward.
    """

    mdp = MDP()

    # Create the states.
    for x in range(width):
        for y in range(height):
            mdp.S |= {(x, y)}

    # Create the actions.
    mdp.A = {"n", "s", "e", "w"}

    # Create the transition probabilities.
    for s in mdp.S:
        for a in mdp.A:
            for sp in mdp.S:
                mdp.P[(s, a, sp)] = 0.0

    for sx, sy in mdp.S:
        if sy > 0:
            mdp.P[((sx, sy), "n", (sx, sy - 1))] = 0.7
            mdp.P[((sx, sy), "n", (sx, sy + 1))] = 0.1
            if sx == 0:
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.2
            elif sx == width - 1:
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.2
            else:
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1
        else:
            mdp.P[((sx, sy), "n", (sx, sy + 1))] = 0.1
            if sx == 0:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1
            elif sx == width - 1:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
            else:
                mdp.P[((sx, sy), "n", (sx, sy))] = 0.7
                mdp.P[((sx, sy), "n", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "n", (sx + 1, sy))] = 0.1

        if sy < height - 1:
            mdp.P[((sx, sy), "s", (sx, sy + 1))] = 0.7
            mdp.P[((sx, sy), "s", (sx, sy - 1))] = 0.1
            if sx == 0:
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.2
            elif sx == width - 1:
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.2
            else:
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1
        else:
            mdp.P[((sx, sy), "s", (sx, sy - 1))] = 0.1
            if sx == 0:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1
            elif sx == width - 1:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
            else:
                mdp.P[((sx, sy), "s", (sx, sy))] = 0.7
                mdp.P[((sx, sy), "s", (sx - 1, sy))] = 0.1
                mdp.P[((sx, sy), "s", (sx + 1, sy))] = 0.1

        if sx > 0:
            mdp.P[((sx, sy), "w", (sx - 1, sy))] = 0.7
            mdp.P[((sx, sy), "w", (sx + 1, sy))] = 0.1
            if sy == 0:
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.2
            elif sy == height - 1:
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.2
            else:
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1
        else:
            mdp.P[((sx, sy), "w", (sx + 1, sy))] = 0.1
            if sy == 0:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1
            elif sy == height - 1:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
            else:
                mdp.P[((sx, sy), "w", (sx, sy))] = 0.7
                mdp.P[((sx, sy), "w", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "w", (sx, sy + 1))] = 0.1

        if sx < width - 1:
            mdp.P[((sx, sy), "e", (sx + 1, sy))] = 0.7
            mdp.P[((sx, sy), "e", (sx - 1, sy))] = 0.1
            if sy == 0:
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.2
            elif sy == height - 1:
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.2
            else:
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1
        else:
            mdp.P[((sx, sy), "e", (sx - 1, sy))] = 0.1
            if sy == 0:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1
            elif sy == height - 1:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.8
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
            else:
                mdp.P[((sx, sy), "e", (sx, sy))] = 0.7
                mdp.P[((sx, sy), "e", (sx, sy - 1))] = 0.1
                mdp.P[((sx, sy), "e", (sx, sy + 1))] = 0.1

    return mdp


def add_dead_ends(mdp, width, height, numDeadEnds, avoid=list()):
    """ Add dead ends to the MDP. This adds another reward function in sequence for
        the dead ends. The reward provides a 0.0 for all non-dead end states, and a
        -1.0 for an explicit dead end state. In both explicit and implicit dead ends,
        once you get in there, you can never escape (transition) out of it/them.
        This will diverge to negative infinity in value iteration.

        Parameters:
            mdp -- The MDP object, passed by reference. This object is modified.
            width -- The width of the grid world.
            height -- The height of the grid world.
            numDeadEnds -- The number of dead ends to include.
            avoid -- Optionally, the states on which to avoid placing a loop.

        Returns:
            The list of dead ends.
    """

    # Create the list of dead ends.
    deadEnds = list()
    newDeadEnd = (rnd.randint(0, width - 1), rnd.randint(0, height - 1))
    while len(deadEnds) < numDeadEnds:
        if newDeadEnd in deadEnds + avoid:
            newDeadEnd = (rnd.randint(0, width - 1), rnd.randint(0, height - 1))
        else:
            deadEnds += [newDeadEnd]

    # Set dead ends to literally be black holes.
    for sde in deadEnds:
        for a in mdp.A:
            for sp in mdp.S:
                mdp.P[(sde, a, sp)] = 0.0
            mdp.P[(sde, a, sde)] = 1.0

    # Create the reward functions, which captures dead ends.
    R = dict()
    for s in mdp.S:
        for a in mdp.A:
            for sp in mdp.S:
                if sp in deadEnds and sp != s:
                    R[(s, a, sp)] = -1.0
                else:
                    R[(s, a, sp)] = 0.0

    # Add the reward function to the MDP.
    mdp.R += [R]

    return deadEnds


def add_goal_states(mdp, width, height, numGoalStates, avoid=list()):
    """ Add goal states to the MDP. This adds two more reward functions, in sequence,
        for the goals. The first reward function is used to only award 1.0 for the positive
        rewards, and a 0.0 for all other states. You only gain the reward upon entering the
        goal state, and a 0.0 for being in it. The second reward breaks the ties of the
        first by slightly penalizing the non-reward states, which encourages convergence
        to the shortest path. The second reward will diverge to positive or negative infinity;
        however, the first will not diverge.

        Parameters:
            mdp -- The MDP object, passed by reference. This object is modified.
            width -- The width of the grid world.
            height -- The height of the grid world.
            numGoalStates -- The number of goal states to add.
            avoid -- Optionally, the states on which to avoid placing a loop.

        Returns:
            The list of goal states.
    """

    # Create the list of goal states.
    goalStates = list()
    newGoalState = (rnd.randint(0, width - 1), rnd.randint(0, height - 1))
    while len(goalStates) < numGoalStates:
        if newGoalState in goalStates + avoid:
            newGoalState = (rnd.randint(0, width - 1), rnd.randint(0, height - 1))
        else:
            goalStates += [newGoalState]

    # Set goal states to literally be black holes.
    for sg in goalStates:
        for a in mdp.A:
            for sp in mdp.S:
                mdp.P[(sg, a, sp)] = 0.0
            mdp.P[(sg, a, sg)] = 1.0

    # Create the reward functions, which captures goal states.
    R1 = dict()
    R2 = dict()
    for s in mdp.S:
        for a in mdp.A:
            for sp in mdp.S:
                if sp in goalStates and sp != s:
                    R1[(s, a, sp)] = 1.0
                    R2[(s, a, sp)] = 1.0
                elif sp in goalStates and sp == s:
                    R1[(s, a, sp)] = 0.0
                    R2[(s, a, sp)] = 0.0
                else:
                    R1[(s, a, sp)] = 0.0
                    R2[(s, a, sp)] = -0.03

    # Add the reward functions to the MDP.
    mdp.R += [R1, R2]

    return goalStates


def add_loop_states(mdp, width, height, numLoopStates, avoid=list()):
    """ Add loop states to the MDP. This only modifies the transition function.

        Parameters:
            mdp -- The MDP object, passed by reference. This object is modified.
            width -- The width of the grid world.
            height -- The height of the grid world.
            numLoopStates -- The number of loop states to add.
            avoid -- Optionally, the states on which to avoid placing a loop.

        Returns:
            The list of loop states.
    """

    loopStates = list()
    newLoopState = (rnd.randint(0, width - 2), rnd.randint(0, height - 2))
    while len(loopStates) < numLoopStates:
        if newLoopState in loopStates + avoid or \
                (newLoopState[0], newLoopState[1] + 1) in loopStates + avoid or \
                (newLoopState[0] + 1, newLoopState[1]) in loopStates + avoid or \
                (newLoopState[0] + 1, newLoopState[1] + 1) in loopStates + avoid:
            newLoopState = (rnd.randint(0, width - 2), rnd.randint(0, height - 2))
        else:
            loopStates += [newLoopState, \
                (newLoopState[0], newLoopState[1] + 1), \
                (newLoopState[0] + 1, newLoopState[1]), \
                (newLoopState[0] + 1, newLoopState[1] + 1)]

    for i in range(0, numLoopStates * 4, 4):
        for a in mdp.A:
            for sp in mdp.S:
                mdp.P[(loopStates[i], a, sp)] = 0.0
                mdp.P[(loopStates[i + 1], a, sp)] = 0.0
                mdp.P[(loopStates[i + 2], a, sp)] = 0.0
                mdp.P[(loopStates[i + 3], a, sp)] = 0.0
        for a in mdp.A:
            mdp.P[(loopStates[i], a, loopStates[i + 1])] = 1.0
            mdp.P[(loopStates[i + 1], a, loopStates[i + 2])] = 1.0
            mdp.P[(loopStates[i + 2], a, loopStates[i + 3])] = 1.0
            mdp.P[(loopStates[i + 3], a, loopStates[i])] = 1.0

    return loopStates


width = 10
height = 10

mdp = create_mdp_three_directions(width, height)
# mdp = create_mdp_four_directions(width, height)

# If you set gamma to 1, it perfectly avoids loops with the 3 reward functions... but diverges to the -(-0.03) with loops...
mdp.gamma = 1.0

# If you set gamma to <1, it does not avoid loops with 2 or 3 reward functions... but converges...

deadEnds = add_dead_ends(mdp, width, height, 1, list())
# deadEnds = list()

goalStates = add_goal_states(mdp, width, height, 1, deadEnds)
# goalStates = list()

loopStates = add_loop_states(mdp, width, height, 1, list())
# loopStates = list()

# pi, V = mdp.value_iteration()
pi, V = mdp.value_iteration(maxIterations=100)

print("V[0] -- First Reward for Explicit Dead End Avoidance")
for y in range(height):
    for x in range(width):
        print("%.6f\t" % (V[0][(x, y)]), end="")
    print()
print()

print("V[1] -- Second Reward for Goal Setting")
for y in range(height):
    for x in range(width):
        print("%.6f\t" % (V[1][(x, y)]), end="")
    print()
print()

print("V[2] -- Third Reward for Shortest Path Tie Breaking")
for y in range(height):
    for x in range(width):
        print("%.2f\t" % (V[2][(x, y)]), end="")
    print()
print()

print(pi)

# print("Dead Ends: %s" % str(deadEnds))
# print("Goal States: %s" % str(goalStates))


# Print out the final result of value iteration.
for y in range(height):
    for x in range(width):
        if (x, y) in deadEnds:
            print("- ", end="")
        elif (x, y) in goalStates:
            print("+ ", end="")
        elif (x, y) in loopStates:
            print("l ", end="")
        else:
            print("%s " % (pi[(x, y)]), end="")
    print("")
