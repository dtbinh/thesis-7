import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *
from random import randrange
	
#--------------------------------------------- control law gains 
kdx = 10   	# proportional term gain
kpx = 30	# derivative term gain	 
kix = 10**-10	# integral term gain

kdy = 10
kpy = 30
kiy = 10**-10

kdz = 6 
kpz = 8	

kdpsi = 2
kppsi = 1

sphi1 = 3
sphi2 = 3                           
sphi3 = 2
sphi4 = 0.1
sthe1 = 3
sthe2 = 3
sthe3 = 2
sthe4 = 0.1
#---------------------------------------------#  other constants
g = -9.8
m = 2
xError = 0
yError = 0

#--------------------------------------------------------

xSetpoint = 1		
ySetpoint = 1
zSetpoint = 1               #  desired z position
psiSetpoint = 0				#  Desired yaw position	

maximumIterations =100
h=0.001


#----------------------------------------------Initial conditions
x = [0,0]
y = [0,0]
z = [0,0]
psi = [0,0]      
phi = [0,0]
the = [0,0]

vx = [0,0] # 
vy = [0,0]
vz = [0,0]
vpsi = [0,0]
vphi = [0,0]
vthe = [0,0]

xdd = [0,0]#---------------------------- initial accelerations
ydd = [0,0]
zdd = [0,0]
psidd = [0,0]      
phidd = [0,0]
thedd = [0,0]

xdd = [0,0]#---------------------------- initial sums of accelerations
ydd = [0,0]
zdd = [0,0]
psidd = [0,0]      
phidd = [0,0]
thedd = [0,0]
#---------------------------------------------------------
def xd():#------------------------Backward finite difference approximation of the derivitive of y at time k
	global x
	return (x[-1] - x[-2])/h 
def yd():#------------------------Backward finite difference approximation of the derivitive of y at time k
	global y
	return (y[-1] - y[-2])/h 
def zd():#------------------------ derivitive of z at time k
	global z
	return (z[-1] - z[-2])/h 
def psid():#------------------------- derivitive of psi, the yaw velocity at time k
	global ps
	return (psi[-1] - psi[-2])/h 
def phid():#------------------------- derivitive of phi, the ___ velocity at time k
	global ph
	return (phi[-1] - phi[-2])/h 
def thed():#------------------------- derivitive of Theta, the ___ velocity at time k
	global the
	return (the[-1] - the[-2])/h 

def xint():
	global xError
	global x
	global xSetpoint
	xError += x[-1] - xSetpoint# the integral term accounts for an accumulation of error
	return xError
def yint():
	global yError
	global y
	global ySetpoint
	yError += (y[-1]-ySetpoint)
	return yError

def systemIterate():#------ this function is called at each time step and returns the next Acceleration 
	global x
	global y
	global z
	global psi
	global phi
	global the

	global h
	
	global xdd
	global ydd
	global zdd
	global psidd      
	global phidd
	global thedd

	
	#x.append( -g*(h**2)*Th[-1] + 2*x[-1]-x[-2]+ ((h**2)/m)*(-ax1*Xdot() - ax2*(x[-1] - xd)) + ax3*Xint() + noise())
	
	# the control laws prescribe accelerations based on measurements
	xdd.append( -g*the[-1] - kdx*xd() - kpx*(x[-1] - xSetpoint) + kix*xint())
	ydd.append( g*phi[-1] - kdy*yd() - kpy*(y[-1] - ySetpoint) + kiy*yint())
	zdd.append( (1/m)*(-kdz*zd() - kpz*(z[-1] - zSetpoint)))
	psidd.append( -kdpsi*psid() - kppsi*(psi[-1] - psiSetpoint))
	phidd.append( -sphi1*(phid() + sphi2*(phi[-1] + phid() + sphi3*(2*phi[-1] + phid() + (yd()/g) + sphi4*(phid() + 3*phi[-1] + 3*(yd()/g) + y[-1]/g )))))
	thedd.append( -sthe1*(thed() + sthe2*(the[-1] + thed() + sthe3*(2*the[-1] + thed() - (xd()/g) + sthe4*(thed() + 3*the[-1] - 3*(xd()/g) - x[-1]/g )))))
	
	vx.append((xdd[-1]+xdd[-2])*h)
	x.append((vx[-1]+vx[-2])*h)


for i in range(maximumIterations):
	systemIterate()
	
# print 'the list of acceleration values are \n\n',xdd,"\n\n"
# print 'the list of velocity values are \n\n',vx,"\n\n"
# print 'the list of position values are \n\n',x,"\n\n"
#-------------------------------------------------------------defintions for plotting

timeSeries = [i for i in range(len(x))]

import matplotlib.gridspec as gridspec
gs = gridspec.GridSpec(1,3)#3, 5)

fig = plt.figure()
#ax = fig.add_subplot(gs[0:2,3:5], projection='3d')
#ax.scatter(x[0:len(x):30], y[0:len(y):30], z[0:len(z):30])
#ax.set_xlabel('X')
#ax.set_ylabel('Y')
#ax.set_zlabel('Z')

xx = fig.add_subplot(gs[0,0])#----------------------linear displacements
plt.plot(timeSeries, x)
title('x',fontsize=10)

yy = fig.add_subplot(gs[0,1])
plt.plot(timeSeries, vx)
title('vx',fontsize=10)

zz = fig.add_subplot(gs[0,2])
plt.plot(timeSeries, xdd)
title('xdd',fontsize=10)
'''
thth = fig.add_subplot(gs[1,0])#---------------------angles
plt.plot(timeSeries, the)
title('Theta = pitch',fontsize=10)

phph = fig.add_subplot(gs[1,1])
plt.plot(timeSeries, phi)
title('Phi = roll',fontsize=10)

psps = fig.add_subplot(gs[1,2])
plt.plot(timeSeries, psi)
title('Psi = yaw',fontsize=10)
'''
plt.show()

