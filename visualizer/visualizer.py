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
import sdl2.sdlgfx

import math

import sys
sys.path.append("../../losm/losm_converter")

from losm_converter import *
from losm_objects import *


class LMDPVisualizer:
    """ Provide a graphical visualization of the LOSM objects and the policy produced
        by solving the corresponding LMDP.
    """

    def __init__(self, highlight=dict(), maxSize=1600, filePrefix=None, policyFile=None):
        """ The constructor for the LMDP class. Optionally, allow for the LOSM files
            to be loaded with the corresponding prefix. Also, optionally allow for
            the policy to be loaded.

            Parameters:
                highlight   -- Highlight dictionary, mapping names of edges/landmarks to colors.
                maxSize     -- The max size in pixels of width or height of the window.
                filePrefix  -- The prefix for the three LOSM files.
                policyFile  -- The policy file to load.
        """

        self.width = 1600
        self.height = 900

        self.vwidth = maxSize
        self.vheight = maxSize
        self.objSize = 20

        self.camera = {'x': 0, 'y': 0, 'scale': 1.0, 'target': 1.0, 'original': 1.0, 'speed': 0.05}
        self.mouseDrag = {'enabled': False, 'x': 0, 'y': 0}

        self.highlight = highlight

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
        arWindow = float(self.vwidth) / float(self.vheight)
        arMap = float(self.bounds['xmax'] - self.bounds['xmin']) / \
                float(self.bounds['ymax'] - self.bounds['ymin'])

        if arMap < arWindow:
            self.vwidth = int(arMap * self.vheight)
        else:
            self.vheight = int(arMap / self.vwidth)


    def _adjust_nodes_and_landmarks(self):
        """ Adjust the location of each node and landmark based on the
            bounds to fit in the screen.
        """

        for obj in self.losm.nodes + self.losm.landmarks:
            obj.x = float(self.vwidth) * (obj.x - self.bounds['xmin']) / \
                            (self.bounds['xmax'] - self.bounds['xmin'])
            obj.y = float(self.vheight) * (obj.y - self.bounds['ymin']) / \
                            (self.bounds['ymax'] - self.bounds['ymin'])

        # We must flip x and y since longitude and latitude are backwards.
        # Don't ask me why. We also must invert the (new) y-axis since pixels
        # are also backwards.
        temp = None
        for obj in self.losm.nodes + self.losm.landmarks:
            temp = obj.x
            obj.x = obj.y
            obj.y = self.vwidth - temp

        temp = self.vwidth
        self.vwidth = self.vheight
        self.vheight = temp

        # Center the map at the end.
        for obj in self.losm.nodes + self.losm.landmarks:
            obj.x -= self.vwidth / 2
            obj.y -= self.vheight / 2


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

        window = sdl2.ext.Window("LMDP Visualizer", size=(self.width, self.height))
        window.show()

        #self._create_map_texture()
        renderer = sdl2.ext.Renderer(window)

        running = True
        while running:
            events = sdl2.ext.get_events()

            for event in events:
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    break

                self._check_keyboard(event)
                self._check_mouse(event)

            self._update_camera()

            renderer.color = sdl2.ext.Color(0, 0, 0)
            renderer.clear()
            self._render_map(renderer)
            renderer.present()

        sdl2.ext.quit()


#    def _create_map_texture(self):
#        """ Create the map texture to improve rendering speed. """
#
#        mapTexture = sdl2.ext.TextureSprite()
#
#        renderer = sdl2.ext.Renderer(mapTexture)
#        renderer.color = sdl2.ext.Color(0, 0, 0)
#        renderer.clear()
#        self._render_map(renderer)
#        renderer.present()


    def _check_keyboard(self, event):
        """ Check the event for keyboard input.

            Parameters:
                event -- The SDL2 event.
        """

        if event.type == sdl2.SDL_KEYDOWN:
            pass

        elif event.type == sdl2.SDL_KEYUP:
            activateCamera = False

            if event.key.keysym.sym == sdl2.SDLK_1:
                self.camera['target'] = 1.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_2:
                self.camera['target'] = 10.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_3:
                self.camera['target'] = 20.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_4:
                self.camera['target'] = 30.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_5:
                self.camera['target'] = 40.0
                activateCamera = True

            if activateCamera:
                self.camera['original'] = self.camera['scale']
                self.camera['timer'] = 0.0


    def _check_mouse(self, event):
        """ Check the event for mouse input, either buttons pressed or motion.

            Parameters:
                event -- The SDL2 event.
        """

        if event.type == sdl2.SDL_MOUSEBUTTONDOWN:
            if event.button.button == sdl2.SDL_BUTTON_LEFT:
                self._mouse_button('left', 'down', event.button.x, event.button.y)
            elif event.button.button == sdl2.SDL_BUTTON_RIGHT:
                self._mouse_button('right', 'down', event.button.x, event.button.y)

        elif event.type == sdl2.SDL_MOUSEBUTTONUP:
            if event.button.button == sdl2.SDL_BUTTON_LEFT:
                self._mouse_button('left', 'up', event.button.x, event.button.y)
            elif event.button.button == sdl2.SDL_BUTTON_RIGHT:
                self._mouse_button('right', 'up', event.button.x, event.button.y)

        elif event.type == sdl2.SDL_MOUSEMOTION:
            self._mouse_motion(event.motion.x, event.motion.y)


    def _mouse_button(self, button, state, x, y):
        """ Handle the mouse button presses.

            Parameters:
                button  -- Either 'left' or 'right'.
                state   -- Either 'down' or 'up'.
                x       -- The x-axis location where this occurred.
                y       -- The y-axis location where this occurred.
        """

        if button == 'left':
            self.mouseDrag['enabled'] = (state == 'down')
            self.mouseDrag['x'] = x
            self.mouseDrag['y'] = y


    def _mouse_motion(self, x, y):
        """ Handle the mouse motion.

            Parameters:
                x -- The mouse's x-axis location.
                y -- The mouse's y-axis location.
        """

        if self.mouseDrag['enabled']:
            self.camera['x'] += (x - self.mouseDrag['x']) / self.camera['scale']
            self.camera['y'] += (y - self.mouseDrag['y']) / self.camera['scale']

            self.mouseDrag['x'] = x
            self.mouseDrag['y'] = y


    def _update_camera(self):
        """ Update the camera animations. """

        if abs(self.camera['target'] - self.camera['scale']) > 0.01:
            traveled = self.camera['scale'] - self.camera['original']
            remaining = self.camera['target'] - self.camera['original']

            self.camera['timer'] += self.camera['speed']

            t = 6 - 12 * self.camera['timer']
            sigmoid = (1 / (1 + math.e ** t))

            self.camera['scale'] = self.camera['original'] + remaining * sigmoid


    def _render_map(self, renderer):
        """ Render the map to the window.

            Parameters:
                renderer -- The renderer object.
        """

        for edge in self.losm.edges:
            n1 = self.uidToNode[edge.uid1]
            n2 = self.uidToNode[edge.uid2]

            line = [int(self.camera['scale'] * (n1.x + self.camera['x']) + self.vwidth / 2),
                    int(self.camera['scale'] * (n1.y + self.camera['y']) + self.vheight / 2),
                    int(self.camera['scale'] * (n2.x + self.camera['x']) + self.vwidth / 2),
                    int(self.camera['scale'] * (n2.y + self.camera['y']) + self.vheight / 2)]

            try:
                renderer.color = self.highlight[edge.name]
            except:
                renderer.color = sdl2.ext.Color(255, 255, 255)
            renderer.draw_line(line)

            # Note: Either do above or the one below... Don't do both.
#            sdl2.sdlgfx.thickLineRGBA(renderer.renderer,
#                                        line[0], line[1], line[2], line[3],
#                                        2,
#                                        renderer.color.r,
#                                        renderer.color.g,
#                                        renderer.color.b,
#                                        renderer.color.a)

#            sdl2.sdlgfx.stringRGBA(renderer.renderer,
#                                    int((line[0] + line[2]) / 2),
#                                    int((line[1] + line[3]) / 2),
#                                    str.encode(edge.name),
#                                    renderer.color.r,
#                                    renderer.color.g,
#                                    renderer.color.b,
#                                    renderer.color.a)


        for obj in self.losm.landmarks:
        #for obj in self.losm.nodes + self.losm.landmarks:
            r = (int(self.camera['scale'] * (obj.x + self.camera['x']) + self.vwidth / 2 - self.objSize / 2),
                 int(self.camera['scale'] * (obj.y + self.camera['y']) + self.vheight / 2 - self.objSize / 2),
                 int(self.objSize),
                 int(self.objSize))

            try:
                renderer.color = self.highlight[obj.name]
            except:
                renderer.color = sdl2.ext.Color(255, 255, 255)
            renderer.draw_rect([r])

            try:
                sdl2.sdlgfx.stringRGBA(renderer.renderer,
                                        int(r[0] + r[2] / 2),
                                        int(r[1] + r[3] / 2 - self.objSize),
                                        str.encode(obj.name),
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)
            except:
                pass


if __name__ == "__main__":
    # --- DEBUG ---
    h = dict()
    h['Gray Street'] = sdl2.ext.Color(255, 0, 0)
    h['Rao\'s cafe'] = sdl2.ext.Color(0, 255, 0)
    h['Amherst coffee'] = sdl2.ext.Color(0, 0, 255)

    if len(sys.argv) == 2:
        v = LMDPVisualizer(highlight=h, filePrefix=sys.argv[1])
        v.execute()
    elif len(sys.argv) == 3:
        v = LMDPVisualizer(filePrefix=sys.argv[1], policyFile=sys.argv[2])
        v.execute()
    else:
        print("python visualizer.py " +
              "<LOSM file prefix> " +
              "<optional, policy file name>")

