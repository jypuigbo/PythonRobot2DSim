import numpy as np
from Box2DWorld import (world, step, createBox, createBoxFixture, createCircle,
                        myCreateRevoluteJoint, vrotate, vangle, RayCastCallback,Box2D)
from Arm import Arm
from VectorFigUtils import dist


class GradSensor(object):
    """ Gradient Sensor used by EPuck."""

    def __init__(self, ngrad=1, name="grad",maxdist=3.):
        """Init Gradient Sensor."""
        self.name = name
        self.ngrad = ngrad
        self.maxd = maxdist
        if(ngrad < 4):
            m, da = (1 + ngrad) % 2, np.pi / (2 + ngrad)
        elif(ngrad == 4):
            m, da = (1 + ngrad) % 2, np.pi / ngrad
        else:
            m, da = (1 + ngrad) % 2, np.pi / (ngrad - 1)
        self.GradAngles = [k * da - ((ngrad - m) / 2) * da - m * da / 2 for k in range(ngrad)]
        self.GradValues = [0 for i in range(ngrad)]

    def update(self, pos, angle, centers=[], extremes=0):
        """Update passing agnet pos, angle and list of positions of gradient emmiters."""
        sensors = range(self.ngrad)
        if(extremes):
            sensors = [0, self.ngrad - 1]

        if(len(centers) == 0): return

        for k in sensors:
            v = vrotate((1, 0), angle + self.GradAngles[k])
            vals = [0 for i in range(len(centers))]
            for i, c in enumerate(centers):
                vc = (c[0] - pos[0], c[1] - pos[1])
                d = dist(pos, c)
                if(d > self.maxd):
                    d = self.maxd
                vals[i] = ((self.maxd - d) / self.maxd) * (1 - abs(vangle(v, vc)) / np.pi)
            self.GradValues[k] = 1 - max(vals)


class IR(object):
    """Infraread sensors class implemented as RayCast used by EPuck, CartPole."""

    def __init__(self, nir=1,ignoreList=[]):
        """Init IRAngles and IRValues and RayCast."""
        self.nir = nir
        self.maxdist = 1
        self.callback = RayCastCallback()
        if(nir < 4):
            m, da = (1 + nir) % 2, np.pi / (2 + nir)
        else:
            m, da = (1 + nir) % 2, np.pi / (nir - 1)
        self.IRAngles = [k * da - ((nir - m) / 2) * da - m * da / 2 for k in range(nir)]
        self.IRValues = [1 for i in range(nir)]
        self.ignoreList=ignoreList

    def update(self, pos, angle, r=0.1):
        """Udpate casting ray."""
        for k in range(self.nir):
            v = vrotate((1, 0), angle + self.IRAngles[k])
            c = pos + [0.9 * r * v[0], 0.9 * r * v[1]]
            cdist = pos + [self.maxdist * v[0], self.maxdist * v[1]]
            self.callback.fixture = None
            world.RayCast(self.callback, c, cdist)
            if(self.callback.fixture is not None):
                if 'ignore' in self.callback.fixture.body.userData.keys():
                    self.IRValues[k] = 1
                elif any([ig in self.callback.fixture.body.userData['name'] for ig in self.ignoreList]):
                    self.IRValues[k] = 1
                else:
                    self.IRValues[k] = dist(c, self.callback.point) / self.maxdist
            else:
                self.IRValues[k] = 1

class VisualSensor(object):
    """Infraread sensors class implemented as RayCast used by EPuck, CartPole."""

    def __init__(self, retinaSize=10,span=90,maxdist=20):
        """Init IRAngles and IRValues and RayCast."""
        self.retinaSize = retinaSize
        self.maxdist = maxdist
        self.callback = RayCastCallback()
        if(retinaSize < 4):
            m, da = (1 + retinaSize) % 2, 2*np.pi*span/360. / (2 + retinaSize)
        else:
            m, da = (1 + retinaSize) % 2, 2*np.pi*span/360. / (retinaSize - 1)
        self.VSAngles = [k * da - ((retinaSize - m) / 2) * da - m * da / 2 for k in range(retinaSize)]
        self.RGB = [[0,0,0] for i in range(retinaSize)]

    def update(self, pos, angle, r=0.1):
        """Udpate casting ray."""
        for k in range(self.retinaSize):
            v = vrotate((1, 0), angle + self.VSAngles[k])
            c = pos + [0.9 * r * v[0], 0.9 * r * v[1]]
            cdist = pos + [self.maxdist * v[0], self.maxdist * v[1]]
            self.callback.fixture = None
            world.RayCast(self.callback, c, cdist)
            if(self.callback.fixture is not None):
                if 'RGB' in self.callback.fixture.body.userData.keys():
                    self.RGB[k] = [int(col-100*dist(c, self.callback.point) / self.maxdist) for col in self.callback.fixture.body.userData['RGB']]
                else:
                    self.RGB[k] = [255,255,255]
            else:
                self.RGB[k] = [0,0,0]


# *****************************************************************
# Epuck class

class Epuck(object):
    """Epuck robot class: two motors and infrared sensors."""

    def __init__(self, position=(0, 0), angle=np.pi / 2, r=0.48, bHorizontal=False, frontIR=6, nother=0, nrewsensors=0,
                 RGB=[255,0,0],bodyType='circle',categoryBits=0x0001,name='epuck',maskBits=0x0009):
        """Init of userData map with relevant values."""

        self.ini_pos = position
        if bodyType=='circle':
            self.body = createCircle(position, r=r, bDynamic=True, restitution=0, name=name, categoryBits=categoryBits, maskBits=maskBits)
        elif bodyType=='square':
            self.body = createBox(position, w=r, h=r, wdiv=1, hdiv=1, bDynamic=True, restitution=0, name=name,categoryBits=categoryBits, maskBits=maskBits)
        self.body.angle = angle
        self.r = r

        self.motors = [0, 0]
        self.bHorizontal = bHorizontal
        self.bForceMotors = True

        self.frontIR = frontIR
        self.IR = IR(frontIR)

        self.RGB=RGB

        self.body.userData["nIR"] = frontIR
        self.body.userData["IRAngles"] = self.IR.IRAngles
        self.body.userData["IRValues"] = self.IR.IRValues
        self.body.userData["RGB"] = self.RGB
        self.body.userData["radius"] = self.r

        self.GradSensors = []

        self.body.userData["nOtherSensors"] = nother
        self.nother = nother
        if(nother > 0):
            otherGrad = GradSensor(nother, name="other")
            self.GradSensors.append(otherGrad)
            self.body.userData["OtherAngles"] = otherGrad.GradAngles
            self.body.userData["OtherValues"] = otherGrad.GradValues

        self.nrewardsensors = nrewsensors
        self.body.userData["nRewardSensors"] = nrewsensors
        if(nrewsensors > 0):
            rewGrad = GradSensor(nrewsensors, name="reward")
            self.GradSensors.append(rewGrad)
            self.body.userData["RewardAngles"] = rewGrad.GradAngles
            self.body.userData["RewardValues"] = rewGrad.GradValues

        self.userData = self.body.userData

    def getPosition(self):
        """Return position."""
        return self.body.position

    def setPosition(self,p):
        """Set position."""
        self.body.position = p

    def getAngle(self):
        """Return position."""
        return self.body.angle

    def getVelocity(self):
        """Return linear velocity."""
        return self.body.linearVelocity

    def getIRs(self):
        """Return IR list."""
        return self.body.userData["IRValues"]

    def stop(self):
        self.body.linearVelocity = [0, 0]
        self.body.angularVelocity = 0
        step()


    def update(self):
        """update of position applying forces and IR."""
        body, angle, pos = self.body, self.body.angle, self.body.position
        mLeft, mRight = self.motors
        fangle, fdist = 50 * (mRight - mLeft), 1000 * (mLeft + mRight)
        d = (fdist * np.cos(angle), fdist * np.sin(angle))

        if(not self.bHorizontal):
            self.body.linearVelocity = [d[0]/50,d[1]/50]
            self.body.angularVelocity = fangle/2

        #if(self.bForceMotors):
        #    body.ApplyTorque(fangle, wake=True)
        #    body.ApplyForce(force=d, point=body.worldCenter, wake=False)

        if(self.bHorizontal):
            body.angularVelocity = 0
            body.angle = np.pi / 2

        nir = self.frontIR
        self.IR.update(pos, angle, self.r)


# ********************************************************
# Simple Robot class with two arms of any number of joints


class NaoRobot:
    """Two Arm robot top view."""

    def __init__(self, position=(0,0), name="simple", bTwoArms=True, collisionGroup=None, bOppositeArms=False):
        """Init body and arms of the robot."""
        global nao
        nao = self
        self.ini_pos = position
        x, y = position[0], position[1]
        self.salient = []
        self.bTwoArms = bTwoArms
        self.body_pos = (x, y)
        w = 0.4
        if(bOppositeArms):
            w = 0.5

        if(not bOppositeArms and bTwoArms):
            createBox((x - w / 2.0, y), w=w / 2.0, h=w / 1.8, bDynamic=False, collisionGroup=-1)
            createBox((x + w / 2.0, y), w=w / 2.0, h=w / 1.8, bDynamic=False, collisionGroup=-1)

        bShrink = False
        length = 1
        self.nparts = 2
        if(name == "human"):
            self.nparts = 3
            length = 1.0 / 1.5
            bShrink = True

        self.arms = []

        if(not bOppositeArms):
            self.arms.append(Arm(bLateralize=1, hdiv=1, nparts=self.nparts, position=(x - w, y), length=length, name=name, bShrink=bShrink, collisionGroup=collisionGroup))
            if(bTwoArms):
                self.arms.append(Arm(bLateralize=2, hdiv=1, nparts=self.nparts, position=(x+w,y), length=length, name=name, bShrink=bShrink, collisionGroup=collisionGroup))
        else:
            arm1 = Arm(position=(x, y), bLateralize=0, hdiv=1, nparts=self.nparts, length = length, name=name, bShrink=bShrink, collisionGroup=collisionGroup)
            arm2 = Arm(position=(x, y + 3), signDir=-1, bLateralize=0, hdiv=1, nparts=self.nparts, length=length, name=name, bShrink=bShrink, collisionGroup=collisionGroup)
            self.arms.append(arm1)
            self.arms.append(arm2)


    def getMotorSpeeds(self):
        speeds = []
        for arm in self.arms:
            speeds += arm.getMotorSpeeds()
        return speeds

    def getJointLimits(self, iarm=-1):
        if(iarm == -1):
            joints = []
            for arm in self.arms:
                joints += arm.getJointLimits()
            return joints

        if(iarm >= 2):
            print "getJointLimits Invalid iarm", iarm
        return self.arms[iarm].getJointLimits()

    def getJointAngles(self, iarm=-1):
        m = []
        if(iarm < 0):
            m += self.arms[0].getJointAngles()
            if(self.bTwoArms):
                m += self.arms[1].getJointAngles()
            return m
        thearm = self.arms[iarm]
        return thearm.getJointAngles()

    def getFinalPos(self, iarm=-1):
        if(iarm < 0):
            return self.arms[0].getFinalPos() + self.arms[1].getFinalPos()
        else:
            return self.arms[iarm].getFinalPos()

    def gotoTargetJoints(self, t=[0, 0, 0, 0, 0, 0], iarm=-1):
        l = len(t)/2
        if(iarm==1 and not self.bTwoArms): return []
        elif(iarm<0):
            self.arms[0].gotoTargetJoints(t[:l])
            if(self.bTwoArms):
                self.arms[1].gotoTargetJoints(t[l:])
        else:
            self.arms[iarm].gotoTargetJoints(t)
        self.update()
        return self.getFinalPos(iarm=iarm)


    def setTargetJoints(self, t=[1, -1], iarm=-1):
        if(iarm < 0):
            if(self.bTwoArms):
                l = len(t) / 2
                self.arms[0].setTargetJoints(t[0:l])
                self.arms[1].setTargetJoints(t[l:])
            else:             
                self.arms[0].setTargetJoints(t)
        else:
            if(not self.bTwoArms and iarm == 1):
                return
            self.arms[iarm].setTargetJoints(t)

    def restPosition(self, online=True, iarms=[0, 1], otherarm=-1):
        if(otherarm >= 0): 
            if(otherarm == 0):
                iarms = [1]
            if(otherarm == 1):
                iarms = [0]
        da = self.m_maxs()[0]
        for iarm in iarms:
            if(online):
                if(iarm == 1):
                    self.setTargetJoints([-da] + [0] * (self.nparts - 1), iarm=1) 
                if(iarm == 0):
                    self.setTargetJoints([da] + [0] * (self.nparts - 1), iarm=0)
            else:
                if(iarm == 1):
                    self.gotoTargetJoints([-da] + [0] * (self.nparts - 1), iarm=1) 
                if(iarm == 0):
                    self.gotoTargetJoints([da] + [0] * (self.nparts - 1), iarm=0)
            self.stop(iarm)

    def stop(self, iarm=-1):
        iarms = [0, 1]
        if(iarm >= 0):
            iarms = [iarm]
        for iarm in iarms:
            if(not self.bTwoArms and iarm == 1):
                return
            self.arms[iarm].stop()


    def deltaMotorUpdate(self, dm=[]):
        self.deltaMotor(dm)
        for i in range(25):
            self.update()
            world.Step(TIME_STEP, vel_iters, pos_iters)
            world.ClearForces()
        return self.getFinalPos()

    def deltaMotor(self, dm=[], iarm=-1):
        if(iarm < 0):
            if(len(dm) == 0):
                l = 2 * self.arms[0].nparts
                dm = [round(2 * r - 1, 2) for r in np.random.rand(l)]
            i = len(dm) / 2
            self.arms[0].deltaMotor(dm[0:i])
            self.arms[1].deltaMotor(dm[i:])
        else:
            if(len(dm) == 0):
                l = self.arms[iarm].nparts
                dm = [round(2 * r - 1, 2) for r in np.random.rand(l)]
            self.arms[iarm].deltaMotor(dm)

    def update(self, iarm=-1):
        sum = 0
        if(iarm < 0):
            self.salient = []
            for iarm, a in enumerate(self.arms):
                sum += self.arms[iarm].update()
                self.salient += self.arms[iarm].getSalient()
        else:
            sum = self.arms[iarm].update()
        return sum

    def getSalient(self):
        return self.salient

    def m_mins(self): 
        joint_limits = self.getJointLimits()
        return [j[0] for j in joint_limits]

    def m_maxs(self): 
        joint_limits = self.getJointLimits()
        return [j[1] for j in joint_limits]

    def s_mins(self):
        return [-3.0, -2.0]

    def s_maxs(self):
        return [3.0, 3.0]


    def dm_mins(self):
        return [-1] * len(self.getJointLimits())

    def dm_maxs(self):
        return [1] * len(self.getJointLimits())

    def dm_bounds(self):
        return tuple([(self.dm_mins()[d], self.dm_maxs()[d]) for d in range(len(self.dm_mins()))])

    def ds_mins(self):
        return [-1] * 4

    def ds_maxs(self):
        return [1] * 4

    def rest_position(self):
        return self.m_mins()


# *****************************************************************
# Arm class of any parts and adding a joint extra hand if wanted

class CartPole:
    """Cartpole self balancing robot class."""

    def __init__(self, position=(0, 0), name="simple", d=1, bHand=0, collisionGroup=None):
        """Init using IR class."""
        global bDebug
        self.name = name
        self.ini_pos = position
        self.salientMode = "all"
        self.circle = createCircle(position, r=d * 0.6, bDynamic=True, density=1, name="wheel")
        self.box = createBox((position[0], position[1] + d * 1.9), d * 0.2, d * 2, bDynamic=True, density=1)
        self.joint = myCreateRevoluteJoint(self.circle, self.box, position, iswheel=True)
        self.bHand = bHand

        self.IR = IR(1)
        self.box.userData["name"] = name
        self.box.userData["nIR"] = 1

        if(name == "cartLeft"):
            self.IR.IRAngles = [0]
        else:
            self.IR.IRAngles = [np.pi]

        self.box.userData["IRAngles"] = self.IR.IRAngles
        self.box.userData["IRValues"] = self.IR.IRValues

        if(bHand > 0):
            body = self.box
            h = d * 0.15
            w = d * 0.4
            pos = (2 * w - d * 0.2, 0)
            if(bHand == 2):
                pos = (-2 * w + d * 0.2, 0)
            fixtureDef = createBoxFixture(pos, width=w, height=h, collisionGroup = collisionGroup)
            body.CreateFixture(fixtureDef)

        self.motor_speed = 0
        self.angle = 0

    def resetPosition(self):
        ipos = self.ini_pos
        self.motor_speed = 0
        self.angle = 0
        self.joint.motorSpeed = 0
        self.joint.torque = 0
        self.box.linearVelocity = [0, 0]
        self.box.angularVelocity = 0
        self.box.angle = 0
        self.box.position = (ipos[0], ipos[1] + 1.5)

        self.circle.linearVelocity = [0, 0]
        self.circle.angularVelocity = 0
        self.circle.angle = 0
        self.circle.position = (ipos[0], ipos[1])

    def getChestPos(self):
        body = self.box
        shape = body.fixtures[0].shape
        vertices = [body.transform * v for v in shape.vertices]
        if(body.userData["name"] == "cartLeft"):
            a, b = vertices[2], vertices[1]
            chestpos = a + (b - a) / 3.8
        else:
            a, b = vertices[3], vertices[0]
            chestpos = a + (b - a) / 3.8
        return chestpos

    def getBodyPos(self):
        body = self.box
        shape = body.fixtures[1].shape
        verticesHand = [body.transform * v for v in shape.vertices]

        shape = body.fixtures[0].shape
        vertices=[body.transform * v for v in shape.vertices]

        if(self.bHand == 2):
            d = vertices[2] - vertices[1]
            d = (d[0] / 12., d[1] / 12.)
            p = (verticesHand[1] + verticesHand[2]) / 2.0 + d
        else:
            d = vertices[2] - vertices[1]
            d = (d[0] / 12., d[1] / 12.)
            p = (verticesHand[3] + verticesHand[0]) / 2.0 + d

        ret = [p[0], p[1]]
        ret = [round(e, 2) for e in ret]
        return ret


    def setMotorSpeed(self,speed):  
        self.joint.motorSpeed = speed

    def getIR(self):
        return self.IR.IRValues[0]

    def getAngle(self):
        return self.box.angle

    def getPosition(self):
        return self.circle.position[0]

    def getVelocity(self):
        return self.circle.linearVelocity

    def update(self):
        self.angle = self.getAngle()
        self.IR.update(self.getChestPos(), self.angle)
