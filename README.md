lmdp
====

Lexicographic Markov Decision Processes (LMDPs) are MOMDPs with state-dependent lexicographic preferences over the reward functions, allowing for slack in optimization. Value iteration for LMDPs solves this problem by applying dynamic programming over the states and rewards in a particular order, yielding one of the solutions which satisfy the slack constraints of the LMDP.

For more information, please see our AAAI 2015 paper:

Wray, Kyle H., Zilberstein, Shlomo, and Mouaddib, Abdel-Illah. "Multi-Objective MDPs with Conditional Lexicographic Reward Preferences." In Proceedings of the Twenty Ninth Conference on Artificial Intelligence (AAAI), Austin, TX, USA, January 2015.
