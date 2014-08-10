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

import sdl2
import sdl2.ext

import sys
sys.path.append("../../losm/losm_converter")
from losm_converter import *
from losm_objects import *


class LMDPVisualizer:
    """ Provide a graphical visualization of the LOSM objects and the policy produced
        by solving the corresponding LMDP.
    """

    def __init__(self, maxSize=1500, filePrefix=None, policyFile=None):
        """ The constructor for the LMDP class. Optionally, allow for the LOSM files
            to be loaded with the corresponding prefix. Also, optionally allow for
            the policy to be loaded.

            Parameters:
                maxSize     -- The max size in pixels of width or height of the window.
                filePrefix  -- The prefix for the three LOSM files.
                policyFile  -- The policy file to load.
        """

        self.width = maxSize
        self.height = maxSize
        self.objSize = 20

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

        self._compute_bounds()
        self._adjust_nodes_and_landmarks()
        self._create_uid_to_node_dict()


    def _compute_bounds(self):
        """ Compute the bounds of the LOSM object. """

        self.bounds = {'xmin': float("inf"), 'ymin': float("inf"), \
                       'xmax': float("-inf"), 'ymax': float("-inf")}

        for obj in self.losm.nodes + self.losm.landmarks:
            if self.bounds['xmax'] < obj.x:
                self.bounds['xmax'] = obj.x
            if self.bounds['ymax'] < obj.y:
                self.bounds['ymax'] = obj.y
            if self.bounds['xmin'] > obj.x:
                self.bounds['xmin'] = obj.x
            if self.bounds['ymin'] > obj.y:
                self.bounds['ymin'] = obj.y

        # Modify the bounds of the window based on these bounds. This
        # preserves the apsect ratio of the map, while maximizing the
        # window as large as possible.
        arWindow = float(self.width) / float(self.height)
        arMap = float(self.bounds['xmax'] - self.bounds['xmin']) / \
                float(self.bounds['ymax'] - self.bounds['ymin'])

        if arMap < arWindow:
            self.width = int(arMap * self.height)
        else:
            self.height = int(arMap / self.width)


    def _adjust_nodes_and_landmarks(self):
        """ Adjust the location of each node and landmark based on the
            bounds to fit in the screen.
        """

        for obj in self.losm.nodes + self.losm.landmarks:
            obj.x = int(float(self.width) * (obj.x - self.bounds['xmin']) / \
                            (self.bounds['xmax'] - self.bounds['xmin']))
            obj.y = int(float(self.height) * (obj.y - self.bounds['ymin']) / \
                            (self.bounds['ymax'] - self.bounds['ymin']))

        # We must flip x and y since longitude and latitude are backwards.
        # Don't ask me why. We also must invert the (new) y-axis since pixels
        # are also backwards.
        temp = None
        for obj in self.losm.nodes + self.losm.landmarks:
            temp = obj.x
            obj.x = obj.y
            obj.y = self.width - temp

        temp = self.width
        self.width = self.height
        self.height = temp


    def _create_uid_to_node_dict(self):
        """ Create the UID to LOSMNode dictionary for quick edge rendering. """

        self.uidToNode = {node.uid: node for node in self.losm.nodes}


    def load_policy(self, policyFile):
        """ Load the LMDP policy from the file provided.

            Parameters:
                policyFile -- The policy file to load.
        """

        pass


    def execute(self):
        """ Begin the visualizer. """

        sdl2.ext.init()

        window = sdl2.ext.Window("LMDP Visualizer",
                                 size=(self.width, self.height))
        window.show()

        renderer = sdl2.ext.Renderer(window)

        running = True
        while running:
            events = sdl2.ext.get_events()
            for event in events:
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    break

            renderer.color = sdl2.ext.Color(0, 0, 0)
            renderer.clear()

            for edge in self.losm.edges:
                n1 = self.uidToNode[edge.uid1]
                n2 = self.uidToNode[edge.uid2]

                if edge.name == "Gray Street":
                    renderer.color = sdl2.ext.Color(255, 0, 0)
                else:
                    renderer.color = sdl2.ext.Color(255, 255, 255)

                renderer.draw_line([n1.x, n1.y, n2.x, n2.y])

            for obj in self.losm.landmarks:
            #for node in self.losm.nodes + self.losm.landmarks:
                r = (obj.x - int(self.objSize / 2), \
                     obj.y - int(self.objSize / 2), \
                     self.objSize, \
                     self.objSize)

                renderer.color = sdl2.ext.Color(0, 255, 0)
                renderer.draw_rect([r])

            renderer.present()

        sdl2.ext.quit()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        v = LMDPVisualizer(filePrefix=sys.argv[1])
        v.execute()
    elif len(sys.argv) == 3:
        v = LMDPVisualizer(filePrefix=sys.argv[1], policyFile=sys.argv[2])
        v.execute()
    else:
        print("python visualizer.py " +
              "<LOSM file prefix> " +
              "<optional, policy file name>")

