#this control law is taken from 'Teppo Luukkonen'

#   the form of the controller is on page 12

'''
here is an outline of the program flow:

    1) initialize lists for state variables and appropriate derivatives. also constants
    2) calculate total thrust T and the torques using the state variables from the [k]th 
       time step
    3) using T[k] and tao_[k] calculate new motor speed vector for time [k]
    4) using T[k] and tao_[k] calculate new state variables at time [k+1]
    5) repeat 
'''

from numpy import cos as c, sin as s , sqrt, array, dot
from numpy.linalg import inv 
from coriolis_matrix import coriolis_matrix

#-------------------------------- physical constants
g = 9.81 #[m/s^2]
m = 0.468 #[kg]
L = 0.22  #[m]
b = 1.0 * 10**-7
k = 3.0*10**-6

#--------------------------------moments of inertia
Ixx = 5.0*10**-3
Iyy = 5.0*10**-3
Izz = 10.0*10**-3

#--------------------------------PID derivative gain values
kdz     = 2.5
kdphi   = 1.75
kdtheta = 1.75
kdpsi   = 1.75

# -------------------------------PID proportional gain values
kpz     = 3.
kpphi   = 8.
kptheta = 6.
kppsi   = 6.

#--------------------------------position set points        
z_des = 1.
phi_des = 0
theta_des = 0
psi_des = 0
#--------------------------------velocity setpoints
zdot_des = 0
phidot_des = 0
thetadot_des = 0
psidot_des = 0

#--------------------------------directional drag coeficients

Ax = 0.25
Ay = 0.25
Az = 0.25


def J(ph,th):

    global Ixx 
    global Iyy 
    global Izz 

    return array([
    [Ixx        ,                               0  , -Ixx * s(th)                ],
    [0          , Iyy*(c(ph)**2) + Izz * s(ph)**2  , (Iyy-Izz)*c(ph)*s(ph)*c(th) ],
    [-Ixx*s(th) , (Iyy-Izz)*c(ph)*s(ph)*c(th)      , Ixx*(s(th)**2) + Iyy*(s(th)**2)*(c(th)**2) + Izz*(c(ph)**2)*(c(th)**2)]    
    ])    

'''
usage for coriolis matrix:

coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz):

'''

######################################################################################################

#--------------------------------state vector and  derivative time series'
x = [0]
y = [0]
z = [0]

xdot = [0]
ydot = [0]
zdot = [0]

xddot = [0]
yddot = [0]
zddot = [0]

phi = [1]
theta = [0.25]
psi = [0.25]

phidot = [0]
thetadot = [0]
psidot = [0] 

phiddot = [0]
thetaddot = [0]
psiddot = [0]

#--------------------------------- force, torque, and motor speed list initializations

T = []
tao_phi = []
tao_theta = []
tao_psi = []

w1 = []
w2 = []
w3 = []
w4 = []

etaddot = []
#----------------------------------

max_iterations = 5000
h = 0.001

for i in range(max_iterations):
    
    #--------------------------------control expressions  eq 23

    T.append( (g + kdz*( zdot_des - zdot[-1] ) + kpz*( z_des - z[-1] ))* m/float( c(phi[-1]) * c(theta[-1])) )

    tao_phi.append( ( kdphi*( phidot_des - phidot[-1] ) + kpphi*( phi_des - phi[-1] ) ) * Ixx )

    tao_theta.append( ( kdtheta*( thetadot_des - thetadot[-1] ) + kptheta*( theta_des - theta[-1] ) )*Iyy)

    tao_psi.append( ( kdpsi*( psidot_des - psidot[-1] ) + kppsi*( psi_des - psi[-1] ) ) * Izz )

    #--------------------------------solve for motor speeds, eq 24

    w1.append( sqrt( (T[-1] / (4.0*k)) - ( tao_theta[-1] / (2.0*k*L) )  - ( tao_psi[-1] / (4.0*b) ) )  )
    w2.append( sqrt( (T[-1] / (4.0*k)) - ( tao_phi[-1]   / (2.0*k*L) )  + ( tao_psi[-1] / (4.0*b) ) )  )
    w3.append( sqrt( (T[-1] / (4.0*k)) + ( tao_theta[-1] / (2.0*k*L) )  - ( tao_psi[-1] / (4.0*b) ) )  )
    w4.append( sqrt( (T[-1] / (4.0*k)) + ( tao_phi[-1]   / (2.0*k*L) )  + ( tao_psi[-1] / (4.0*b) ) )  )

    #---------------------------------calculate new linear accelerations

    xddot.append(   ( T[-1] / m ) * ( c(psi[-1]) * s(theta[-1]) * c(phi[-1]) + s(psi[-1]) * s(phi[-1]) ) - Ax * xdot[-1] / m   )

    yddot.append(   ( T[-1] / m ) * ( s(psi[-1]) * s(theta[-1]) * c(phi[-1]) - c(psi[-1]) * s(phi[-1]) ) - Ay * ydot[-1] / m   )

    zddot.append(   -g + ( T[-1] / m ) * ( c(theta[-1]) * c(phi[-1]) ) - Az * zdot[-1] / m   )
 
    #-------------------------------- calculate new angular accelerations

    tao = array( [tao_phi[-1], tao_theta[-1], tao_psi[-1] ] )  # must build vectors of kth timestep quantities for the matrix math evaluations

    etadot = array( [phidot[-1], thetadot[-1], psidot[-1] ] )

    etaddot =  dot(  
                   inv( J(phi[-1],theta[-1])  ),  
                   tao  -  dot(  
                               coriolis_matrix(phi[-1],theta[-1],phidot[-1],thetadot[-1],psidot[-1] ,Ixx,Iyy,Izz) ,  
                               etadot
                              )         
                  )

    phiddot.append(etaddot[0])       # parse the etaddot vector of the new accelerations into the appropriate time series'

    thetaddot.append(etaddot[1])

    psiddot.append(etaddot[2])

    #------------------------------ integrate new acceleration values to obtain velocity values

    xdot.append(  xdot[-1] + xddot[-1] * h  )
    ydot.append(  ydot[-1] + yddot[-1] * h  )
    zdot.append(  zdot[-1] + zddot[-1] * h  )

    phidot.append(  phidot[-1] + phiddot[-1] * h  )
    thetadot.append(  thetadot[-1] + thetaddot[-1] * h  )
    psidot.append(  psidot[-1] + psiddot[-1] * h  )

   
    #------------------------------ integrate new velocity values to obtain position / angle values

    x.append(  x[-1] + xdot[-1] * h  )
    y.append(  y[-1] + ydot[-1] * h  )
    z.append(  z[-1] + zdot[-1] * h  )

    phi.append(  phi[-1] + phidot[-1] * h  )
    theta.append(  theta[-1] + thetadot[-1] * h  )
    psi.append(  psi[-1] + psidot[-1] * h  )

###########################################################################---------PLOTS...
#print 'len(x) = ',len(x)
#print 'len(wi) = ',len(w1)

timeSeries = [h*i for i in range(len(x))]
 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *   
import matplotlib.gridspec as gridspec
gs = gridspec.GridSpec(8, 6)

fig = plt.figure()
ax = fig.add_subplot(gs[0:6,3:6], projection='3d')
ax.scatter(x[0:len(x):30], y[0:len(y):30], z[0:len(z):30])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

#----------------------------------------------------------------------------linear displacements
xx = fig.add_subplot(gs[0:2,0:3])
plt.plot(timeSeries, x,'r',  timeSeries, y,'g',  timeSeries, z,'b')
title('x,y,z',fontsize=10)

#-----------------------------------------------------------------------------angles
thth = fig.add_subplot(gs[2:4,0:3 ])
plt.plot(timeSeries, phi,'r',  timeSeries, theta,'g',  timeSeries, psi,'b')
title('Phi = roll, Theta = pitch, Psi = yaw',fontsize=10)

#------------------------------------------------------------------------------plotting motor speeds
spd = fig.add_subplot(gs[4:6,0:3])
plt.plot(timeSeries[:-1], w1,'r',  timeSeries[:-1], w2,'g',  timeSeries[:-1], w3,'b',  timeSeries[:-1], w4,'k',)
title('motor speeds',fontsize=10)

#------------------------------------------------------------------------------torque
spd = fig.add_subplot(gs[6:8,0:3])
plt.plot(timeSeries[:-1], tao_phi,'r',  timeSeries[:-1], tao_theta,'g',  timeSeries[:-1], tao_psi,'b')
title('torques ',fontsize=10)

#------------------------------------------------------------------------------thrust
spd = fig.add_subplot(gs[6:8,3:6])
plt.plot(timeSeries[:-1], T,'r',)
title('Total Thrust',fontsize=10)


plt.show()            




