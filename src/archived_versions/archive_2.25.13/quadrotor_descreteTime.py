import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *
from random import randrange
import pprint
pp = pprint.PrettyPrinter(indent=2)


ax1 = 10   # proportional term gain
ax2 = 30	# derivative term gain	 
ax3 = 10**-10	# integral term gain

ay1 = 10
ay2 = 30
ay3 = 10**-10

az1 = 6 
az2 = 8	

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
	
#xd = 1		
#yd = 1
#zd = 1                 		#  desired z position
Psd = 0				#  Desired yaw position	h = 0.001


m = 2
#maximumIterations =1000
#h=0.05

xError = 0
yError = 0

#---------------------------------------------------Initial conditions
x = [0,0]
y = [0,0]
z = [0,0]
Ps = [0,0]      
Ph = [0,0]
Th = [0,0]

def Xdot(h):#------------------------Backward finite difference approximation of the derivitive of y at time k
	global x
	return (x[-1] - x[-2])/h 
def Ydot(h):#------------------------Backward finite difference approximation of the derivitive of y at time k
	global y
	return (y[-1] - y[-2])/h 
def Zdot(h):#------------------------ derivitive of z at time k
	global z
	return (z[-1] - z[-2])/h 
def Psdot(h):#------------------------- derivitive of psi, the yaw velocity at time k
	global Ps
	return (Ps[-1] - Ps[-2])/h 
def Phdot(h):#------------------------- derivitive of phi, the ___ velocity at time k
	global Ph
	return (Ph[-1] - Ph[-2])/h 
def Thdot(h):#------------------------- derivitive of Theta, the ___ velocity at time k
	global Th
	return (Th[-1] - Th[-2])/h 

def Xint(xd,h):
	global xError
	global x
	xError += (x[-1]-xd)*h
	return xError
def Yint(yd,h):
	global yError
	global y
	yError += (y[-1]-yd)*h
	return yError

def noise():
	return (10**-15)*randrange(-100,100,1)


def systemIterate(xd,yd,zd,h):#--------------------- this function is called at each time step and returns the next value of each [x,y,z,ps,ph,th]
	global x
	global y
	global z
	global Ps
	global Ph
	global Th

	#global h


	'''
	use non zero desited veloity setpoint to set up waypoints that the qr must pass through but not hover at!!!	
	'''



	x.append( -g*(h**2)*Th[-1] + 2*x[-1]-x[-2]+ ((h**2)/m)*(-ax1*Xdot(h) - ax2*(x[-1] - xd)) + ax3*Xint(xd,h) + noise())
 
	y.append( g*(h**2)*Ph[-1] + 2*y[-1]-y[-2] + ((h**2)/m)*(-ay1*Ydot(h) - ay2*(y[-1] - yd)) + ay3*Yint(yd,h) + noise())
    
	z.append( ((h**2)/m)*(-az1*Zdot(h) - az2*(z[-1] - zd)) + 2*z[-1] - z[-2] + noise() )
    
	Ps.append( (h**2)*(-aPs1*Psdot(h) - aPs2*(Ps[-1] - Psd)) + 2*Ps[-1] - Ps[-2] + noise() )
    
	Ph.append( -(h**2)*sPh1*(Phdot(h) + sPh2*(Ph[-1] + Phdot(h) + sPh3*(2*Ph[-1] + Phdot(h) + (Ydot(h)/g) + sPh4*(Phdot(h) + 3*Ph[-1] + 3*(Ydot(h)/g) + y[-1]/g )))) + 2*Ph[-1] - Ph[-2] + noise() )

	Th.append( -(h**2)*sTh1*(Thdot(h) + sTh2*(Th[-1] + Thdot(h) + sTh3*(2*Th[-1] + Thdot(h) - (Xdot(h)/g) + sTh4*(Thdot(h) + 3*Th[-1] - 3*(Thdot(h)/g) - x[-1]/g )))) + 2*Th[-1] - Th[-2] + noise() )


#-----------------------------in the client script where this function is used there should be a known 3D setpoint for the control input
def go(xd,yd,zd, step_size , maximumIterations): 
    h = step_size    
    for k in range(maximumIterations-1):#-------------the magic happens here
		systemIterate(xd,yd,zd,h)
#--------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    
    #these calls of the go() function form the three legs of the flight path... 
    h = 0.001 
    go(1,1,1,h,5000)
    
    go(5,4,3,h,5000)
    
    go(10,5,12,h, 5000)
    
    print 'the final position is:'
    print [x[-1],y[-1],z[-1]]
    print 'the relative steady state error in position is:'
#    print [abs(x[-1]-xd)/xd,abs(y[-1]-yd)/yd,abs(z[-1]-zd)/zd]
    print 'the steady state error in pitch, roll and yaw angles:'
    print [Th[-1],Ph[-1],Ps[-1]]
    
    #note that h will have to be updated, or the the statements below will have to be made fuctions to pass h as a variable
       
    
    vx = [(x[i]-x[i-1])/h for i in range(len(x)-1)]
    vy = [(y[i]-y[i-1])/h for i in range(len(y)-1)]
    vz = [(z[i]-z[i-1])/h for i in range(len(z)-1)]
    
    fx = [m*(vx[i]-vx[i-1])/h for i in range(len(vx)-1)]# the forces as measured from the inertial frame
    fy = [m*(vy[i]-vy[i-1])/h for i in range(len(vy)-1)]
    fz = [m*(vz[i]-vz[i-1])/h for i in range(len(vz)-1)]
    '''    
    for i in range(len(fx)):# this is a temporary fix to get rid of the few giant numbers that appear in the forces
        if abs(fx[i]) > 100:			#need figure out whats going on here
            fx[i] = 0
    for i in range(len(fy)):
        if abs(fy[i]) > 100:
            fy[i] = 0
    for i in range(len(fz)):
        if abs(fz[i]) > 100:
            fz[i] = 0
    '''    
    timeSeries = [h*i for i in range(len(x))]
    
    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(3, 5)
    
    fig = plt.figure()
    ax = fig.add_subplot(gs[0:2,3:5], projection='3d')
    ax.scatter(x[0:len(x):30], y[0:len(y):30], z[0:len(z):30])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    xx = fig.add_subplot(gs[0,0])#----------------------linear displacements
    plt.plot(timeSeries, x)
    title('x',fontsize=10)
    
    yy = fig.add_subplot(gs[0,1])
    plt.plot(timeSeries, y)
    title('y',fontsize=10)
    
    zz = fig.add_subplot(gs[0,2])
    plt.plot(timeSeries, z)
    title('z',fontsize=10)
    
    thth = fig.add_subplot(gs[1,0])#---------------------angles
    plt.plot(timeSeries, Th)
    title('Theta = pitch',fontsize=10)
    
    phph = fig.add_subplot(gs[1,1])
    plt.plot(timeSeries, Ph)
    title('Phi = roll',fontsize=10)
    
    psps = fig.add_subplot(gs[1,2])
    plt.plot(timeSeries, Ps)
    title('Psi = yaw',fontsize=10)
    
    fxp = fig.add_subplot(gs[2,0])#-----------------------plotting forces
    plt.plot(timeSeries[2:-2], fx[2:])
    title('fx',fontsize=10)

    fyp = fig.add_subplot(gs[2,1])
    plt.plot(timeSeries[4:-2], fy[3:-1])
    title('fy',fontsize=10)
    
    fzp = fig.add_subplot(gs[2,2])
    plt.plot(timeSeries[2:-2], fz[2:])
    title('fz',fontsize=10)
    
    plt.show()






