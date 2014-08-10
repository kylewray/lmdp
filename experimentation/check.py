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
import csv
import numpy as np


def load_check_file(filename):
    """ Load one of the check files given.

        Parameters:
            filename -- The name of the checkfile to load.

        Returns:
            The data as a numpy matrix; None if an error occurred.
    """

    try:
        with open(filename, 'r') as f:
            data = list()

            reader = csv.reader(f, delimiter=',')

            for row in reader:
                dataRow = list()
                for val in row:
                    dataRow += [float(val)]
                data += [dataRow]

            return np.matrix(data)
    except IOError:
        print("Failed to open file '%s'." % (filename))

    return None

def execute():
    """ Execute the script. """

    # Ensure two arguments are given.
    if len(sys.argv) != 3:
        print("Please specify the two check files to load.")
        return


    # Load the two files using the two arguments. Return on error.
    check1 = load_check_file(sys.argv[1])
    if check1 == None:
        return

    check2 = load_check_file(sys.argv[2])
    if check2 == None:
        return

    if np.shape(check1) != np.shape(check2):
        print("The two check files are not the same dimension.")
        return

    # Subtract the two and take the absolute value.
    difference = np.abs(check1 - check2)

    # Find the maximum difference between the two.
    maxDifference = np.max(difference)

    # Print the results.
    print("File '%s':\n%s\n" % (sys.argv[1], str(check1)))
    print("File '%s':\n%s\n" % (sys.argv[2], str(check2)))
    print("Difference:\n%s\n" % (str(difference)))
    print("Max Difference: %f" % (maxDifference))


if __name__ == "__main__":
    execute()

