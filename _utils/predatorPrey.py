from Robots import *

class prey(Epuck):
	def __init__(self,position=(0, 0), angle=np.pi / 2, r=0.48, bHorizontal=False, frontIR=6, nother=0, nrewsensors=0):
		Epuck.__init__(self,position=position,angle=angle,nother=nother,nrewsensors=nrewsensors,r=.3,frontIR=15)
	def update(self):
		Epuck.update(self)
		IRs=self.IR
		IR= IRs
		amounts = np.cos(IR.IRAngles)*(1-np.array(IR.IRValues))**2.
		amount=np.sum(amounts)
		self.motors[0] = -amount+.5
		self.motors[1] = amount+.5
