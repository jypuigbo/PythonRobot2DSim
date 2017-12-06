import numpy as np
import Box2D
from Box2DWorld import (world, arm, createBox, createCircle, createTri, createRope,
                        myCreateRevoluteJoint, myCreateDistanceJoint,
                        myCreateLinearJoint, collisions, RayCastCallback)

from VectorFigUtils import vnorm, dist
from Robots import NaoRobot, CartPole, Epuck
import random
from predatorPrey import prey

# put some walls independant of the screen; beacuse screen is defined in PyGame
def addWalls(pos, dx=3, dh=0, h=2.8, th=0, bHoriz=True, bVert=True, damping = 0): 
    """ Also defined locally in ExpSetupDualCartPole!!! """
    x, y = pos
    wl = 0.2
    yh = (5 + 1) / 2.0
    if(bHoriz):
        createBox((x, y - 1 - dh + th), w=h + dh + wl + th, h=wl, bDynamic=False, damping=damping, name="wall_top")
        createBox((x, y + 5 + dh + th), w=h + dh + wl + th, h=wl, bDynamic=False, damping=damping, name="wall_bottom")

    if(bVert):
        createBox((x - dx - wl, y + yh - 1 + dh / 2 + th), w=wl, h=h + dh, bDynamic=False, damping=damping, friction=0, name="wall_left")
        createBox((x + dx + wl, y + yh - 1 + dh / 2 + th), w=wl, h=h + dh, bDynamic=False, damping=damping, friction=0, name="wall_right")

def addStandardWall(pos,L,W,name):
    createBox(pos, w=W, h=L, bDynamic=False, damping=-1, name=name)

def addReward(who, pos=(0,0), vel=(0,0), reward_type=0, bDynamic=True, bCollideNoOne=False):
    if(reward_type == 0):
        name, r = "reward", 0.27
    else:
        name, r = "reward_small", 0.2

    obj = createCircle(position=pos, bDynamic=bDynamic, bCollideNoOne=bCollideNoOne, density=5, damping=0, friction=0, name=name, r=r)
    obj.userData["energy"] = 1.0
    obj.userData["visible"] = 1.0
    obj.linearVelocity = vel
    who.objs.append(obj)


# *****************************************************************
# Experimental Setup Epuck Preys
# *****************************************************************

class ExpSetupPreys(object):
    """Exp setup class with two epucks and two reward sites."""

    def __init__(self, n=1, debug=False):
        """Create the two epucks, two rewards and walls."""
        global bDebug
        bDebug = debug
        print "-------------------------------------------------"
        th = .2
        positions = [(-3, 2 + th), (3, 2 + th)]
        angles = [2 * np.pi, np.pi]
        self.epucks = [prey(position=positions[i], angle=angles[i], frontIR=0, nother=2, nrewsensors=4) for i in range(n)]
        addWalls((0, 0), dx=3.75, dh=0.1, h=5, th=th)
        addStandardWall((0,0),5,.1,'wall_1')
        self.objs = []
        addReward(self, pos=(0, 4 + th), vel=(0, 0), bDynamic=False, bCollideNoOne=True)
        addReward(self, pos=(0, 0 + th), vel=(0, 0), reward_type=1, bDynamic=False, bCollideNoOne=True)

    def update(self):
        """Update of epucks positions and gradient sensors: other and reward."""
        for e in self.epucks:
            e.update()
            pos = e.getPosition()

            for g in e.GradSensors:
                if(g.name == "other"):
                    centers = [o.getPosition() for o in self.epucks if o != e]
                    g.update(pos, e.getAngle(), centers)
                elif(g.name == "reward"):
                    centers = [o.position for o in self.objs[:1]]
                    g.update(pos, e.getAngle(), centers)
                    centers = [o.position for o in self.objs[-1:]]
                    g.update(pos, e.getAngle(), centers, extremes=1)



    def setMotors(self, epuck=0, motors=[10, 10]):
        self.epucks[epuck].motors = motors


# *****************************************************************
# Experimental Setup MultiAgent
# *****************************************************************

class ExpSetupMultiAgent(object):
    """Exp setup class with two epucks and two reward sites."""

    def __init__(self, n=1, debug=False):
        """Create the two epucks, two rewards and walls."""
        global bDebug
        bDebug = debug
        print "-------------------------------------------------"
        th = .2

        positions = [ (random.uniform(-5,5), random.uniform(-1,3)) for i in range(n)]

        angles = [random.uniform(0,2*np.pi) for i in range(n)]
        self.epucks = [Epuck(position=positions[i], angle=angles[i], frontIR=0, nother=2, nrewsensors=4) for i in range(n)]

        self.objs = []
        addReward(self, pos=(0, 4 + th), vel=(0, 0), bDynamic=False, bCollideNoOne=True)
        addReward(self, pos=(0, 0 + th), vel=(0, 0), reward_type=1, bDynamic=False, bCollideNoOne=True)

    def update(self):
        """Update of epucks positions and gradient sensors: other and reward."""
        for e in self.epucks:
            e.update()
            pos = e.getPosition()

            for g in e.GradSensors:
                if(g.name == "other"):
                    centers = [o.getPosition() for o in self.epucks if o != e]
                    g.update(pos, e.getAngle(), centers)
                elif(g.name == "reward"):
                    centers = [o.position for o in self.objs[:1]]
                    g.update(pos, e.getAngle(), centers)
                    centers = [o.position for o in self.objs[-1:]]
                    g.update(pos, e.getAngle(), centers, extremes=1)



    def setMotors(self, epuck=0, motors=[10, 10]):
        self.epucks[epuck].motors = motors




