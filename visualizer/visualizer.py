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

import ctypes
import math
import numpy as np
import random as rnd

from policy import *

import sys
sys.path.append("../../losm/losm_converter")

from losm_converter import *
from losm_objects import *


NUM_TIREDNESS_LEVELS = 2

# This is for clicking nodes via mouse interaction: radial distance in pixels squared.
STATE_NODE_DISTANCE_THRESHOLD =  30 ** 2

RENDER_NORMAL_LINES = False

AUTONOMY_SPEED_LIMIT_THRESHOLD = 30


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

        self.roadLineWidth = 20
        self.markerSize = 20

        self.fontSize = 30

        self.camera = {'x': 0, 'y': 0, 'scale': 1.0, 'target': 1.0, 'original': 1.0, 'speed': 0.05}
        self.mouseDrag = {'enabled': False, 'x': 0, 'y': 0}
        self.mousePosition = {'x': 0, 'y': 0}

        self.policyLineWidth = 5 # 10
        self.policyOffset = 10 # 2.5
        self.policyTrigonWidth = 6.0 # 1.0
        self.highlight = highlight

        self.tiredness = 0
        self.autonomy = False

        self.stateNodes = list()
        self.showStateNodes = False
        self.selectedNodes = set()

        #self.fastRender = True
        self.fastRender = False
        self.elements = dict()
        self.elementResolution = 6 #12  # Note: How many sub-divisions, i.e., scale as above.

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
        ar = float(self.width) / float(self.height)
        arv = float(self.bounds['xmax'] - self.bounds['xmin']) / \
                float(self.bounds['ymax'] - self.bounds['ymin'])

        w = self.height
        h = self.width

        self.vwidth = min(w, int(arv * h))
        self.vheight = min(h, int(w / arv))


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

        #self.policy = Policy(policyFile)

        self.policy = [[list() for j in range(2)] for i in range(NUM_TIREDNESS_LEVELS)]

        with open(policyFile, 'r') as f:
            reader = csv.reader(f, delimiter=',')

            for row in reader:
                if len(row) < 6:
                    print("Failed to parse >= 6 arguments for policy line: '" + \
                        ",".join(row) + "'.")
                    break

                currentTiredness = int(row[2])
                currentAutonomy = (row[3] == '1')

                uids = (int(row[0]), int(row[1]), int(row[4]))
                nextAutonomy = (row[5] == '1')
                valueVector = [float(r) for r in row[6:]]

                self.policy[currentTiredness][currentAutonomy] += \
                        [(uids, nextAutonomy, valueVector)]


    def execute(self):
        """ Begin the visualizer. """

        sdl2.ext.init()

        window = sdl2.ext.Window("LMDP Visualizer", size=(self.width, self.height))
        window.show()

        renderer = sdl2.ext.Renderer(window)

        if self.fastRender:
            self._create_map(renderer)

        # Execute the main loop.
        running = True
        while running:
            events = sdl2.ext.get_events()

            for event in events:
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    #break

                self._check_keyboard(event)
                self._check_mouse(event)

            self._update_camera()

            renderer.color = sdl2.ext.Color(230, 230, 220)
            renderer.clear()

            if self.fastRender:
                self._render_map_texture(renderer)
            else:
                self._render_map(renderer)
                self._render_policy(renderer)

            renderer.present()

        if self.fastRender:
            self._free_map()

        sdl2.ext.quit()


    def _create_map(self, renderer):
        """ Create the entire map of texture elements.

            Parameters:
                renderer -- The renderer object.
        """

        for t in range(2):
            self.tiredness = t
            for a in [True, False]:
                self.autonomy = a
                print("Generating for (t, a) = (%i, %i)." % (t, int(a)))
                self._create_map_texture(renderer)

        print("Done")


    def _free_map(self):
        """ Free all the texture elements. """

        for t in range(2):
            for a in [True, False]:
                print("Clearing for (t, a) = (%i, %i)." % (t, int(a)))
                for i, element in enumerate(self.elements[(t, int(a))]):
                    if i % self.elementResolution == 0:
                        print("Freeing map texture %i-%i out of %i." % \
                                (i, i + self.elementResolution,
                                self.elementResolution ** 2))
                    sdl2.SDL_DestroyTexture(element[4])

        print("Done")


    def _create_map_texture(self, renderer):
        """ Create the map texture to improve rendering speed.

            Parameters:
                renderer -- The renderer object.
        """

        self.elements[(self.tiredness, self.autonomy)] = list()

        elementWidth = int(self.width / self.elementResolution)
        elementHeight = int(self.height / self.elementResolution)

        offsetX = self.vwidth / 2
        offsetY = self.vheight / 2

        for x in range(self.elementResolution):
            print("Creating map textures %i-%i out of %i." % \
                    (x * self.elementResolution,
                    (x + 1) * self.elementResolution,
                    self.elementResolution ** 2))

            for y in range(self.elementResolution):
                textureElement = sdl2.SDL_CreateTexture(renderer.renderer,
                                                sdl2.SDL_PIXELFORMAT_RGBA8888,
                                                sdl2.SDL_TEXTUREACCESS_TARGET,
                                                self.width,
                                                self.height)

                sdl2.SDL_SetRenderTarget(renderer.renderer, textureElement)

                renderer.color = sdl2.ext.Color(230, 230, 220)
                #renderer.color = sdl2.ext.Color(int(rnd.random() * 255),
                #                                int(rnd.random() * 255),
                #                                int(rnd.random() * 255))

                renderer.clear()

                self.camera['scale'] = self.elementResolution
                self.camera['x'] = -x * elementWidth + offsetX
                self.camera['y'] = -y * elementHeight + offsetY

                self._render_map(renderer)
                self._render_policy(renderer)

                renderer.present()

                self.elements[(self.tiredness, self.autonomy)] += \
                                    [(x * elementWidth - self.vwidth / 2,
                                    y * elementHeight - self.vheight / 2,
                                    elementWidth,
                                    elementHeight,
                                    textureElement)]

        self.camera['scale'] = 1
        self.camera['x'] = 0
        self.camera['y'] = 0

        sdl2.SDL_SetRenderTarget(renderer.renderer, None)


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
                self.camera['target'] = 3.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_3:
                self.camera['target'] = 7.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_4:
                self.camera['target'] = 15.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_5:
                self.camera['target'] = 30.0
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_MINUS:
                self.camera['target'] = max(1.0, self.camera['target'] - 0.25)
                self.camera['scale'] = self.camera['target']
                self.camera['original'] = self.camera['target']
                activateCamera = True
            elif event.key.keysym.sym == sdl2.SDLK_EQUALS:
                self.camera['target'] = min(50.0, self.camera['target'] + 0.25)
                self.camera['scale'] = self.camera['target']
                self.camera['original'] = self.camera['target']
                activateCamera = True

            if activateCamera:
                self.camera['original'] = self.camera['scale']
                self.camera['timer'] = 0.0

            if event.key.keysym.sym == sdl2.SDLK_s:
                self.showStateNodes = not self.showStateNodes

            if event.key.keysym.sym == sdl2.SDLK_q:
                self.tiredness = int(not self.tiredness)
                if self.tiredness == 0:
                    print("Tiredness Disabled. Driver is awake.")
                else:
                    print("Tiredness Enabled. Driver is tired.")

            elif event.key.keysym.sym == sdl2.SDLK_w:
                self.autonomy = int(not self.autonomy)
                if self.autonomy:
                    print("Autonomy Enabled. Car is driving.")
                else:
                    print("Autonomy Disabled. Car is not driving.")


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

        elif button == 'right' and state == 'up':
            for node in self.losm.nodes:
                loc = self._camera(node.x, node.y)

                if (loc[0] - self.mousePosition['x']) ** 2 + \
                        (loc[1] - self.mousePosition['y']) ** 2 < \
                        STATE_NODE_DISTANCE_THRESHOLD:
                    print(node)

                    if node in self.selectedNodes:
                        self.selectedNodes -= {node}
                    else:
                        self.selectedNodes |= {node}

                    break


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


    def _render_map_texture(self, renderer):
        """ Render the map texture to the window.

            Parameters:
                renderer -- The renderer object.
        """

        for element in self.elements[(self.tiredness, self.autonomy)]:
            pos = self._camera(element[0], element[1])
            r = sdl2.SDL_Rect(pos[0],
                            pos[1],
                            int(element[2] * self.camera['scale']),
                            int(element[3] * self.camera['scale']))
            sdl2.SDL_RenderCopy(renderer.renderer, element[4], None, r)


    def _render_map(self, renderer):
        """ Render the map to the window.

            Parameters:
                renderer -- The renderer object.
        """

        nonAutonomyEdges = [edge for edge in self.losm.edges if \
                            edge.speedLimit < AUTONOMY_SPEED_LIMIT_THRESHOLD]
        autonomyEdges = [edge for edge in self.losm.edges if \
                            edge.speedLimit >= AUTONOMY_SPEED_LIMIT_THRESHOLD]

        # Organize the rendering order to ensure the pretty autonomy roads are
        # rendered on top of the other edges.
        for edge in nonAutonomyEdges + autonomyEdges: #self.losm.edges:
            n1 = self.uidToNode[edge.uid1]
            n2 = self.uidToNode[edge.uid2]

            line = self._camera(n1.x, n1.y) + self._camera(n2.x, n2.y)

            if max(line[0], line[2]) < 0 or \
                    max(line[1], line[3]) < 0 or \
                    min(line[0], line[2]) >= self.width or \
                    min(line[1], line[3]) >= self.height:
                continue

            try:
                renderer.color = self.highlight[edge.name]
            except KeyError:
                if edge.speedLimit >= AUTONOMY_SPEED_LIMIT_THRESHOLD:
                    try:
                        renderer.color = h['policyAutonomyCapableColor']
                    except KeyError:
                        renderer.color = sdl2.ext.Color(255, 255, 255)
                else:
                    renderer.color = sdl2.ext.Color(255, 255, 255)

            if RENDER_NORMAL_LINES:
                renderer.draw_line(line)
            else:
                sdl2.sdlgfx.thickLineRGBA(renderer.renderer,
                                        line[0], line[1], line[2], line[3],
                                        self.roadLineWidth,
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)

        for obj in self.losm.nodes + self.losm.landmarks:
            r = self._camera(obj.x, obj.y) + [int(self.markerSize), int(self.markerSize)]

            if max(r[0], r[0] + r[2]) < 0 or \
                    max(r[1], r[1] + r[3]) < 0 or \
                    min(r[0], r[0] + r[2]) >= self.width or \
                    min(r[1], r[1] + r[3]) >= self.height:
                continue

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
                # Amherst Dead End Loop = 66688624 <-> 66600236
                # Amherst Strange Conflicting States But Kinda OK = 66672799 <-> 66642836
                # Boston Strange Conflicting States But OK = 61340671 <-> 61340704
                if obj in self.selectedNodes or \
                        (self.showStateNodes and obj in self.stateNodes): # or \
                        #obj.uid == 66688624 or obj.uid == 66600236 or \
                        #obj.uid == 66672799 or obj.uid == 66642836 or \
                        #obj.uid == 61340671 or obj.uid == 61340704:
                    r[0] -= int(self.markerSize / 2)
                    r[1] -= int(self.markerSize / 2)
                    r[2] = r[0] + self.markerSize
                    r[3] = r[1] + self.markerSize

                    sdl2.sdlgfx.boxRGBA(renderer.renderer,
                                        r[0], r[1], r[2], r[3],
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)


    def _render_policy(self, renderer):
        """ Render the policy on the map.

            Parameters:
                renderer -- The renderer object.
        """

        if self.policy == None:
            return

        for action in self.policy[self.tiredness][self.autonomy]:
            enableAutonomy = action[1]
            valueVector = action[2]

            node = [np.array(self._camera(self.uidToNode[uid].x, self.uidToNode[uid].y), \
                        dtype=float) \
                        for uid in action[0]]

            alpha = node[0] - node[1]
            if alpha[0] != 0 or alpha[1] != 0:
                alpha /= np.linalg.norm(alpha)
                alpha *= self.policyOffset * self.camera['scale']

            beta = node[2] - node[1]
            if beta[0] != 0 or beta[1] != 0:
                beta /= np.linalg.norm(beta)
                beta *= self.policyOffset * self.camera['scale']

            line = list(node[1] + alpha * 2) + \
                    list(node[1] + alpha) + \
                    list(node[1] + alpha + beta)
            line = [int(l) for l in line]

            renderer.color = self.highlight['policyColor']


            # ----- BEZIER CURVE VERSION -----

            #xvals = [line[0], line[2], line[4]]
            #xvals = (ctypes.c_short * len(xvals))(*xvals)

            #yvals = [line[1], line[3], line[5]]
            #yvals = (ctypes.c_short * len(yvals))(*yvals)

            #sdl2.sdlgfx.bezierRGBA(renderer.renderer,
            #                        ctypes.cast(xvals, ctypes.POINTER(ctypes.c_short)),
            #                        ctypes.cast(yvals, ctypes.POINTER(ctypes.c_short)),
            #                        3,
            #                        10,
            #                        renderer.color.r,
            #                        renderer.color.g,
            #                        renderer.color.b,
            #                        renderer.color.a)

            # ----- BEZIER CURVE VERSION -----


            line = list(node[1] + alpha * 2) + list(node[1] + alpha)
            line = [int(l) for l in line]

            if enableAutonomy:
                renderer.color = self.highlight['policyColorAutonomy']
            else:
                renderer.color = self.highlight['policyColor']

            if RENDER_NORMAL_LINES:
                renderer.draw_line(line)
            else:
                sdl2.sdlgfx.thickLineRGBA(renderer.renderer,
                                        line[0], line[1], line[2], line[3],
                                        self.policyLineWidth,
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)

            line = list(node[1] + alpha) + list(node[1] + alpha + beta)
            line = [int(l) for l in line]

            #renderer.color = sdl2.ext.Color(0, 0, 150)

            if RENDER_NORMAL_LINES:
                renderer.draw_line(line)
            else:
                sdl2.sdlgfx.thickLineRGBA(renderer.renderer,
                                        line[0], line[1], line[2], line[3],
                                        self.policyLineWidth,
                                        renderer.color.r,
                                        renderer.color.g,
                                        renderer.color.b,
                                        renderer.color.a)

            trigon = node[1] - node[2]
            if trigon[0] != 0 or trigon[1] != 0:
                trigon /= np.linalg.norm(trigon)
                trigon *= self.camera['scale']
            t1 = math.pi / 4.0
            t2 = -t1

            a = node[1] + alpha + beta
            a = [int(a[0]), int(a[1])]

            b = [0, 0]
            b[0] = int(a[0] + self.policyTrigonWidth * math.cos(t1) * trigon[0] - \
                                self.policyTrigonWidth * math.sin(t1) * trigon[1])
            b[1] = int(a[1] + self.policyTrigonWidth * math.sin(t1) * trigon[0] + \
                                self.policyTrigonWidth * math.cos(t1) * trigon[1])

            c = [0, 0]
            c[0] = int(a[0] + self.policyTrigonWidth * math.cos(t2) * trigon[0] - \
                                self.policyTrigonWidth * math.sin(t2) * trigon[1])
            c[1] = int(a[1] + self.policyTrigonWidth * math.sin(t2) * trigon[0] + \
                                self.policyTrigonWidth * math.cos(t2) * trigon[1])

            a = [int(a[0] + beta[0]), int(a[1] + beta[1])]

            sdl2.sdlgfx.filledTrigonRGBA(renderer.renderer,
                                    a[0], a[1], b[0], b[1], c[0], c[1],
                                    renderer.color.r,
                                    renderer.color.g,
                                    renderer.color.b,
                                    renderer.color.a)

            try:
                renderer.color = self.highlight['policyValueColor']
            except KeyError:
                renderer.color = sdl2.ext.Color(0, 0, 0)

            fontManager = sdl2.ext.FontManager("/usr/share/fonts/TTF/DejaVuSans.ttf",
                                                size=self.fontSize,
                                                color=renderer.color)
            spriteFactory = sdl2.ext.SpriteFactory(renderer=renderer)

            for i, v in enumerate(valueVector):
                textSprite = spriteFactory.from_text("%.3f" % (v), fontmanager=fontManager)
                renderer.copy(textSprite, dstrect=(int((node[1] + alpha * 2)[0]),
                                        int((node[1] + alpha * 2)[1]) + \
                                                    i * (self.fontSize + 5),
                                        textSprite.size[0],
                                        textSprite.size[1]))

            fontManager.close()


if __name__ == "__main__":
    # --- DEBUG ---
    h = dict()

    h['policyColor'] = sdl2.ext.Color(25, 150, 25)
    h['policyColorAutonomy'] = sdl2.ext.Color(150, 25, 150)
    h['policyValueColor'] = sdl2.ext.Color(100, 100, 100)
    h['policyAutonomyCapableColor'] = sdl2.ext.Color(240, 240, 255)

    # Boston Commons Highlights
    h['Mount Vernon Place'] = sdl2.ext.Color(200, 0, 0)

    # Amherst (Small) Highlights
    #h['Gray Street'] = sdl2.ext.Color(200, 0, 0)
    #h['Rao\'s cafe'] = sdl2.ext.Color(0, 200, 0)
    #h['Amherst coffee'] = sdl2.ext.Color(0, 0, 200)

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

