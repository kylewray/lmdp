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

import sys
sys.path.append("../../losm/losm_converter")
from losm_converter import *
from losm_objects import *


class LMDPVisualizer:
    """ Provide a graphical visualization of the LOSM objects and the policy produced
        by solving the corresponding LMDP.
    """

    def __init__(self, filePrefix=None, policyFile=None):
        """ The constructor for the LMDP class. Optionally, allow for the LOSM files
            to be loaded with the corresponding prefix. Also, optionally allow for
            the policy to be loaded.

            Parameters:
                filePrefix -- The prefix for the three LOSM files.
                policyFile -- The policy file to load.
        """

        self.losm = None
        if filePrefix != None:
            self.load_losm(filePrefix)

        self.policy = None
        if policyFile != None:
            self.load_policy(policyFile)


    def load_losm(self, filePrefix):
        """ Load the LOSM map given the file prefix.

            Parameters:
                filePrefix -- The prefix for the three LOSM files.
        """

        self.losm = LOSMConverter(filePrefix)


    def load_policy(self, policyFile):
        """ Load the LMDP policy from the file provided.

            Parameters:
                policyFile -- The policy file to load.
        """

        
