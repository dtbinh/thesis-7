import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *
from random import randrange
"""
	1) measure accelerations
		-send request for data to msp430
		-handle read timeout if data is lost
		-retrieve data from rx buffer
	2) calculate velocities and accelerations from acc data
		-use finite sum to calculate velocities from accelerations
		-"              " to calculate positions from velocities
		-??? how to measure error in actual position, how do we know it's in the right place???
		-??? will drift of measurements occur, how to measure drift???
		- might need to confirm presence or absence of inertial measurement drift with ultrasonic range measurements  
	3) Calculate the forces (x,y,z) necessary to move the system to the set point position and orientation
		-use the values for velocities and positions (derived from the measured accelerations) as inputs 
		to the control laws.
		-The control laws return prescribed linear accelerations as measured from the inertial 
		frame 
		-calculate the linear forces as measured from the inertial frame
		-map linear forces (inertial frame) to the thrust  the props can produce (quad-rotor frame)
		-map thrust to motor speed
	4) solve system of nonlinear equations for motor speeds in terms of prescribed liner accelerations (inertial Frame)
		-update motor speeds

	REPEAT
"""

kpx = 10   # proportional term gain
kdx = 30	# derivative term gain	 
kix = 10**-10	# integral term gain

kpy = 10
kdy = 30
kiy = 10**-10

kdz = 6 
kpz = 8	

aPs1 = 2
aPs2 = 1

sPh1 = 3
sPh2 = 3                          # control law gains  
sPh3 = 2
sPh4 = 0.1
sTh1 = 3
sTh2 = 3
sTh3 = 2
sTh4 = 0.1
#-------------------------------#  other constants
g = -9.8
	
xd = 1		
yd = 1
zd = 1                 		#  desired z position
Psd = 0				#  Desired yaw position	h = 0.001
m = 2
maximumIterations =5000
h=0.001

xError = 0
yError = 0
#---------------------------------------------------Initial conditions
x = [0,0]
y = [0,0]
z = [0,0]
Ps = [0,0]      
Ph = [0,0]
Th = [0,0]

ax = [0,0]
ay = [0,0]
az = [0,0]
aPs = [0,0]      
aPh = [0,0]
aTh = [0,0]

def Xdot():#------------------------Backward finite difference approximation of the derivative of y at time k
	global x
	return (x[-1] - x[-2])/h 
def Ydot():#------------------------Backward finite difference approximation of the derivative of y at time k
	global y
	return (y[-1] - y[-2])/h 
def Zdot():#------------------------ derivative of z at time k
	global z
	return (z[-1] - z[-2])/h 
def Psdot():#------------------------- derivative of psi, the yaw velocity at time k
	global Ps
	return (Ps[-1] - Ps[-2])/h 
def Phdot():#------------------------- derivative of phi, the ___ velocity at time k
	global Ph
	return (Ph[-1] - Ph[-2])/h 
def Thdot():#------------------------- derivative of Theta, the ___ velocity at time k
	global Th
	return (Th[-1] - Th[-2])/h 

def Xint():
	global xError
	global x
	xError += (x[-1]-xd)
	return xError
def Yint():
	global yError
	global y
	yError += (y[-1]-yd)
	return yError

def noise():
	return (10**-7)*randrange(-100,100,1)


def systemIterate():#--------------------- this function is called at each time step and returns the next value of each [x,y,z,ps,ph,th]
	global x
	global y
	global z
	global Ps
	global Ph
	global Th
	
	global ax
	global ay
	global az
	global aPs
	global aPh
	global aTh
	
	global h

	x.append( -g*(h**2)*Th[-1] + 2*x[-1]-x[-2]+ ((h**2)/m)*(-kpx*Xdot() - kdx*(x[-1] - xd) + kix*Xint()) + noise())
 
	y.append( g*(h**2)*Ph[-1] + 2*y[-1]-y[-2] + ((h**2)/m)*(-kpy*Ydot() - kdy*(y[-1] - yd)) + kiy*Yint() + noise())
    
	z.append( ((h**2)/m)*(-kdz*Zdot() - kpz*(z[-1] - zd)) + 2*z[-1] - z[-2] + noise() )
    
	Ps.append( (h**2)*(-aPs1*Psdot() - aPs2*(Ps[-1] - Psd)) + 2*Ps[-1] - Ps[-2] + noise() )
    
	Ph.append( -(h**2)*sPh1*(Phdot() + sPh2*(Ph[-1] + Phdot() + sPh3*(2*Ph[-1] + Phdot() + (Ydot()/g) + sPh4*(Phdot() + 3*Ph[-1] + 3*(Ydot()/g) + y[-1]/g )))) + 2*Ph[-1] - Ph[-2] + noise())

	Th.append( -(h**2)*sTh1*(Thdot() + sTh2*(Th[-1] + Thdot() + sTh3*(2*Th[-1] + Thdot() - (Xdot()/g) + sTh4*(Thdot() + 3*Th[-1] - 3*(Thdot()/g) - x[-1]/g )))) + 2*Th[-1] -Th[-2] + noise())
	
	
	
	#for the sake of debugging modify a copy of the system to give acceleration values
	
	ax.append( -g*Th[-1] + ((h**2)/m)*(-kpx*Xdot() - kdx*(x[-1] - xd)) + kix*Xint())
 
	ay.append( g*Ph[-1] + ((h**2)/m)*(-kpy*Ydot() - kdy*(y[-1] - yd)) + kiy*Yint())
    
	az.append( (1/m)*(-kdz*Zdot() - kpz*(z[-1] - zd)) )
    
	aPs.append( (-aPs1*Psdot() - aPs2*(Ps[-1] - Psd)) )
    
	aPh.append( -sPh1*(Phdot() + sPh2*(Ph[-1] + Phdot() + sPh3*(2*Ph[-1] + Phdot() + (Ydot()/g) + sPh4*(Phdot() + 3*Ph[-1] + 3*(Ydot()/g) + y[-1]/g )))) )

	aTh.append( -sTh1*(Thdot() + sTh2*(Th[-1] + Thdot() + sTh3*(2*Th[-1] + Thdot() - (Xdot()/g) + sTh4*(Thdot() + 3*Th[-1] - 3*(Thdot()/g) - x[-1]/g )))) )
	

def go():
	for k in range(maximumIterations-1):#-------------the magic happens here------------------
		systemIterate()
#-----------------------------------------------------------------------
go()
xd = 5
yd = 3
zd = 4
go()
xd = 10
yd = 5
zd = 12
go()

timeSeries = [i for i in range(len(x))]# this is just a time domain list to plot against

##----------------------------------------------------the rest is woohaa for plotting
import matplotlib.gridspec as gridspec
gs = gridspec.GridSpec(3, 5)

fig = plt.figure()
axes = fig.add_subplot(gs[0:2,3:5], projection='3d')
axes.scatter(x[0:len(x):30], y[0:len(y):30], z[0:len(z):30])
axes.set_xlabel('X')
axes.set_ylabel('Y')
axes.set_zlabel('Z')

xx = fig.add_subplot(gs[0,0])#----------------------linear displacements
plt.plot(timeSeries, x)
title('x',fontsize=10)

yy = fig.add_subplot(gs[0,1])
plt.plot(timeSeries, y)
title('y',fontsize=10)

zz = fig.add_subplot(gs[0,2])
plt.plot(timeSeries, z)
title('z',fontsize=10)

thth = fig.add_subplot(gs[1,0])#--------------------- tilt angles
plt.plot(timeSeries, Th)
title('Theta = pitch',fontsize=10)

phph = fig.add_subplot(gs[1,1])
plt.plot(timeSeries, Ph)
title('Phi = roll',fontsize=10)

psps = fig.add_subplot(gs[1,2])
plt.plot(timeSeries, Ps)
title('Psi = yaw',fontsize=10)

axp = fig.add_subplot(gs[2,0])#---------------------plotting prescribed accelerations
plt.plot(timeSeries, ax)
title('ax',fontsize=10)

ayp = fig.add_subplot(gs[2,1])
plt.plot(timeSeries, ay)
title('ay',fontsize=10)

azp = fig.add_subplot(gs[2,2])
plt.plot(timeSeries, az)
title('az',fontsize=10)

plt.show()






