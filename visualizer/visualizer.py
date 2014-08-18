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

from policy import *

import sys
sys.path.append("../../losm/losm_converter")

from losm_converter import *
from losm_objects import *


NUM_TIREDNESS_LEVELS = 2

# This is for clicking nodes via mouse interaction: radial distance in pixels squared.
STATE_NODE_DISTANCE_THRESHOLD =  30 ** 2

class LMDPVisualizer:
    """ Provide a graphical visualization of the LOSM objects and the policy produced
        by solving the corresponding LMDP.
    """

    def __init__(self, highlight=dict(), maxSize=1920, filePrefix=None, policyFile=None):
        """ The constructor for the LMDP class. Optionally, allow for the LOSM files
            to be loaded with the corresponding prefix. Also, optionally allow for
            the policy to be loaded.

            Parameters:
                highlight   -- Highlight dictionary, mapping names of edges/landmarks to colors.
                maxSize     -- The max size in pixels of width or height of the window.
                filePrefix  -- The prefix for the three LOSM files.
                policyFile  -- The policy file to load.
        """

        # Important: This must match 'maxSize' because <reasons></reasons>.
        self.width = 1920
        self.height = 1200

        self.vwidth = maxSize
        self.vheight = maxSize

        self.roadWidth = 5
        self.markerSize = 20

        self.camera = {'x': 0, 'y': 0, 'scale': 1.0, 'target': 1.0, 'original': 1.0, 'speed': 0.05}
        self.mouseDrag = {'enabled': False, 'x': 0, 'y': 0}
        self.mousePosition = {'x': 0, 'y': 0}

        self.policyColor = sdl2.ext.Color(0, 220, 0)
        self.highlight = highlight

        self.initialState = None
        self.goalState = None
        self.tiredness = 0
        self.autonomy = False

        self.stateNodes = list()
        self.path = {'tiredness': 0, 'autonomy': False}

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
        self._create_useful_objects()


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


    def _create_useful_objects(self):
        """ Create the useful objects. This includes a UID to LOSMNode dictionary
            for quick edge rendering. Also, a state nodes list, consisting of all
            nodes with a degree not equal to 2.
        """

        self.uidToNode = {node.uid: node for node in self.losm.nodes}
        self.stateNodes = [node for node in self.losm.nodes if node.degree != 2]


    def load_policy(self, policyFile):
        """ Load the LMDP policy from the file provided.

            Parameters:
                policyFile -- The policy file to load.
        """

        self.policy = Policy(policyFile)


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

            renderer.color = sdl2.ext.Color(230, 230, 220)
            renderer.clear()

            self._render_map(renderer)
            self._render_policy(renderer)

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
                self.camera['target'] = 5.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_3:
                self.camera['target'] = 10.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_4:
                self.camera['target'] = 20.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_5:
                self.camera['target'] = 40.0
                activateCamera = True

            if activateCamera:
                self.camera['original'] = self.camera['scale']
                self.camera['timer'] = 0.0

            hashID = None

            if event.key.keysym.sym == sdl2.SDLK_q:
                hashID = 'initPrev'
            elif event.key.keysym.sym == sdl2.SDLK_w:
                hashID = 'initCur'
            elif event.key.keysym.sym == sdl2.SDLK_e:
                hashID = 'goalPrev'
            elif event.key.keysym.sym == sdl2.SDLK_r:
                hashID = 'goalCur'

            if hashID != None:
                for node in self.stateNodes:
                    loc = self._camera(node.x, node.y)
                    if (loc[0] - self.mousePosition['x']) ** 2 + \
                            (loc[1] - self.mousePosition['y']) ** 2 < \
                            STATE_NODE_DISTANCE_THRESHOLD:
                        print("Set %s to node %i." % (hashID, node.uid))
                        self.path[hashID] = node.uid
                        break

            if event.key.keysym.sym == sdl2.SDLK_t:
                self.path['tiredness'] = int(not self.path['tiredness'])
            elif event.key.keysym.sym == sdl2.SDLK_y:
                self.path['autonomy'] = not self.path['autonomy']

            elif event.key.keysym.sym == sdl2.SDLK_RETURN:
                try:
                    initialState = (self.path['initPrev'], self.path['initCur'],
                                    self.path['tiredness'], self.path['autonomy'])
                    goalState = (self.path['goalPrev'], self.path['goalCur'],
                                self.path['autonomy'])
                    self.policy.set_path_scenario(initialState, goalState)
                except KeyError:
                    print("Failed due to missing state (initial or goal) definition.")
                    pass


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

        self.mousePosition['x'] = x
        self.mousePosition['y'] = y

    def _update_camera(self):
        """ Update the camera animations. """

        if abs(self.camera['target'] - self.camera['scale']) > 0.01:
            traveled = self.camera['scale'] - self.camera['original']
            remaining = self.camera['target'] - self.camera['original']

            self.camera['timer'] += self.camera['speed']

            t = 6 - 12 * self.camera['timer']
            sigmoid = (1 / (1 + math.e ** t))

            self.camera['scale'] = self.camera['original'] + remaining * sigmoid


    def _camera(self, x, y):
        """ Transform an (x, y) coordinate following the camera.

            Parameters:
                x -- The x-axis location.
                y -- The y-axis location.

            Returns:
                A list corresponding to the adjusted (x, y) coordinate.
        """

        return [int(self.camera['scale'] * (x + self.camera['x']) + self.vwidth / 2),
                int(self.camera['scale'] * (y + self.camera['y']) + self.vheight / 2)]


    def _render_map(self, renderer):
        """ Render the map to the window.

            Parameters:
                renderer -- The renderer object.
        """

        for edge in self.losm.edges:
            n1 = self.uidToNode[edge.uid1]
            n2 = self.uidToNode[edge.uid2]

            line = self._camera(n1.x, n1.y) + self._camera(n2.x, n2.y)

            try:
                renderer.color = self.highlight[edge.name]
            except:
                renderer.color = sdl2.ext.Color(255, 255, 255)
            renderer.draw_line(line)

            # Note: Either do above or the one below... Don't do both.
            sdl2.sdlgfx.thickLineRGBA(renderer.renderer,
                                        line[0], line[1], line[2], line[3],
                                        self.roadWidth,
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)

        #for obj in self.losm.landmarks:
        for obj in self.losm.nodes + self.losm.landmarks:
            r = self._camera(obj.x, obj.y) + [int(self.markerSize), int(self.markerSize)]

            try:
                renderer.color = self.highlight[obj.name]
            except:
                renderer.color = sdl2.ext.Color(0, 0, 0)

            try:
                # Successful at accessing 'name' only if it is a landmark.
                text = str(obj.name)
                renderer.draw_rect([r])
                sdl2.sdlgfx.stringRGBA(renderer.renderer,
                                        int(r[0] + r[2] / 2),
                                        int(r[1] + r[3] / 2 - self.markerSize),
                                        str.encode(text),
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)
            except:
                #if obj.uid == 2518152976 or obj.uid == 2518152981:
                #if obj.uid == 66662044 or obj.uid == 66615634:
                #if obj.uid == 66766106 or obj.uid == 66768014:
                #if obj.uid == 66639588 or obj.uid == 66661455:
                #if obj.uid == 66759366 or obj.uid == 66757758:
                #if obj.uid == 2518152976 or obj.uid == 2518152981:
                #if obj.uid == 66696544 or obj.uid == 66621381 or \
                #    obj.uid == 66757197 or obj.uid == 66703862:
                #    renderer.draw_rect([r])
                pass


    def _render_policy(self, renderer):
        """ Render the policy on the map.

            Parameters:
                renderer -- The renderer object.
        """

        if self.policy == None or not self.policy.is_path_prepared():
            return

        self.policy.reset()
        self._render_policy_segment(renderer, self.policy.next())


    def _render_policy_segment(self, renderer, segment):
        """ Render a segment of the policy on the map. (Essentially the
            arrow for a path.) This is a recursive method which renders
            each segment and follows the policy, according to the current
            attributes 'tiredness' and 'autonomy'.

            Parameters:
                renderer    -- The renderer object.
                segment     -- The segment which is a policy state.
        """

        if segment == None:
            return

        # Get the nodes from the segment UIDs (integers).
        n1 = self.uidToNode[segment[0]]
        n2 = self.uidToNode[segment[1]]

        # Render the segment's arrow.
        line = self._camera(n1.x, n1.y) + self._camera(n2.x, n2.y)
        renderer.color = self.policyColor
        renderer.draw_line(line)

        # Determine the next segment.
        nextSegment = self.policy.next()
        self._render_policy_segment(renderer, nextSegment)


if __name__ == "__main__":
    # --- DEBUG ---
    h = dict()
    h['Gray Street'] = sdl2.ext.Color(200, 0, 0)
    h['Rao\'s cafe'] = sdl2.ext.Color(0, 200, 0)
    h['Amherst coffee'] = sdl2.ext.Color(0, 0, 200)

    if len(sys.argv) == 2:
        v = LMDPVisualizer(highlight=h, filePrefix=sys.argv[1])
        v.execute()
    elif len(sys.argv) == 3:
        v = LMDPVisualizer(highlight=h, filePrefix=sys.argv[1], policyFile=sys.argv[2])
        v.execute()
    else:
        print("python visualizer.py " +
              "<LOSM file prefix> " +
              "<optional, policy file name>")

