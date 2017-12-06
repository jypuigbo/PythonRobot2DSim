from Robots import *

class prey(Epuck):
	def __init__(self,position=(0, 0), angle=np.pi / 2, r=0.48, bHorizontal=False, frontIR=6, nother=0, nrewsensors=0):
		Epuck.__init__(self,position=position,angle=angle,nother=nother,nrewsensors=nrewsensors,r=.3,frontIR=15)
	def update(self):
		Epuck.update(self)
		IRs=self.IR
		IR= IRs
		amounts = np.cos(IR.IRAngles)*np.exp(1-np.array(IR.IRValues))*.1
		amount=np.sum(amounts*(amounts>.09))
		self.motors[0] = -amount+.7
		self.motors[1] = amount+.7
