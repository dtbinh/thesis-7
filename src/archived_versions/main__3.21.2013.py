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
m = 1 #[kg]
L = 0.4  #[m]
b = 10**-7
k = 3.0*10**-6

#--------------------------------moments of inertia
Ixx = 5.0*10**-3
Iyy = 5.0*10**-3
Izz = 10.0*10**-3


# -------------------------------PID proportional gain values
kpx     = 10.0
kpy     = 10.0
kpz     = 8.0

#kpphi   = 8.0
#kptheta = 6.0
kppsi   = 6.0


#--------------------------------PID derivative gain values
kdx     = 5.0
kdy     = 5.0
kdz     = 5.0

kdphi   = 1.75
kdtheta = 1.75
kdpsi   = 1.75

#-------------------------------- PID integral gain values

kix = 10**-10
kiy = 10**-10


#--------------------------------nonlinear roll/pitch control law gains
sPh1 = 3
sPh2 = 3                       
sPh3 = 2
sPh4 = .1
sTh1 = 3
sTh2 = 3
sTh3 = 2
sTh4 = .1


#--------------------------------position set points        
x_des = 1.0
y_des = 2.0
z_des = 10.0


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

xError = 0
yError = 0
phiError = 0

def Xint(x_des,h):
	global xError
	global x
	xError += (x[-1]-x_des)*h
	return xError
def Yint(y_des,h):
	global yError
	global y
	yError += (y[-1]-y_des)*h
	return yError
def phiint(phi_des,h):
	global phiError
	global phi
	phiError += (phi[-1]-phi_des)*h
	return phiError


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

phi = [0]
theta = [0]
psi = [0]

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

max_total_thrust = 50.0  # [newtons]
min_total_thrust = 1.0
for i in range(max_iterations):
    
    #--------------------------------control expressions  eq 23

    # the following two expressions for total thrust and torque about z axis are from 'teppo_luukkenon'

    T_k = (g + kdz*( zdot_des - zdot[-1] ) + kpz*( z_des - z[-1] ))* m/float( c(phi[-1]) * c(theta[-1]))

    if T_k <= min_total_thrust:
        T.append( min_total_thrust ) 

    elif T_k >= max_total_thrust:
        T.append( max_total_thrust )
        
    elif T_k > 0 and T_k <= max_total_thrust :    
        T.append( T_k )



    tao_psi.append( ( kdpsi*( psidot_des - psidot[-1] ) + kppsi*( psi_des - psi[-1] ) ) * Izz )


    #equation 61 in 'real-time stablization and tracking'
    tao_phi.append( - sPh1 * (phidot[-1] + sPh2 * (phi[-1] + phidot[-1] + 

                        sPh3 * ( 2 * phi[-1] + phidot[-1] + ( ydot[-1]/g ) + 

                        sPh4 * (phidot[-1] + 3 * phi[-1] + 3 * ( ydot[-1]/(g) ) + y[-1]/(g) )))) * Ixx 

                 )


    #equation 66 in 'real-time stablization and tracking'
    tao_theta.append( - sTh1 * ( thetadot[-1] + sTh2 * ( theta[-1] + thetadot[-1] + 

                    sTh3 * ( 2 * theta[-1] + thetadot[-1] - ( xdot[-1]/(g) ) + 

                    sTh4 * ( thetadot[-1] + 3 * theta[-1] - 3 * ( thetadot[-1]/(g) ) - x[-1]/(g) )))) * Iyy
                )


# original pd contol expressions for roll and pitch

#    tao_phi.append( ( kdphi*( phidot_des - phidot[-1] ) + kpphi*( phi_des - phi[-1] ) ) * Ixx )
#    tao_theta.append( ( kdtheta*( thetadot_des - thetadot[-1] ) + kptheta*( theta_des - theta[-1] ) )*Iyy)


    #--------------------------------solve for motor speeds, eq 24

    w1.append( sqrt( (T[-1] / (4.0*k)) - ( tao_theta[-1] / (2.0*k*L) )  - ( tao_psi[-1] / (4.0*b) ) )  )
    w2.append( sqrt( (T[-1] / (4.0*k)) - ( tao_phi[-1]   / (2.0*k*L) )  + ( tao_psi[-1] / (4.0*b) ) )  )
    w3.append( sqrt( (T[-1] / (4.0*k)) + ( tao_theta[-1] / (2.0*k*L) )  - ( tao_psi[-1] / (4.0*b) ) )  )
    w4.append( sqrt( (T[-1] / (4.0*k)) + ( tao_phi[-1]   / (2.0*k*L) )  + ( tao_psi[-1] / (4.0*b) ) )  )

    #---------------------------------calculate new linear accelerations

    # the following expressions for new x an y accelerations account for the forces:
        # the total thrust imparted by the rotors
        # aerodynamic drag
        # pid control 
        
    '''
    QUESTION : WILL THE PID CONTROL HAVE TO GO INTO A DIFFERENT EXPRESSION IN A REAL IMPLEMENTATION?
    DOES IT AMOUNT TO A PHYSICAL FORCE IN THIS SETUP?
    HAVE I INTRODUCED A LAG OF ONE TIME-STEP?; THE PID INPUT TO THE SYSTEM AT TIME K INFLUENCES THE THRUST AND TORQUES AT TIME K+1?
    '''    
    xddot.append(    (T[-1]/m)  * ( c(psi[-1]) * s(theta[-1]) * c(phi[-1]) + s(psi[-1]) * s(phi[-1]) ) 
                    - (Ax * xdot[-1]/m )  
                    + (-kdx * xdot[-1] - kpx * ( x[-1] - x_des ) + kix * Xint(x_des,h) )
                )


    yddot.append(   ( T[-1] / m ) * ( s(psi[-1]) * s(theta[-1]) * c(phi[-1]) - c(psi[-1]) * s(phi[-1]) )
                    - ( Ay * ydot[-1] / m )  
                    + (-kdy * ydot[-1] - kpy * ( y[-1] - y_des ) + kiy * Yint(y_des,h) )

                )
#    xddot.append(   ( T[-1] / m ) * ( c(psi[-1]) * s(theta[-1]) * c(phi[-1]) + s(psi[-1]) * s(phi[-1]) ) - Ax * xdot[-1] / m   )

#    yddot.append(   ( T[-1] / m ) * ( s(psi[-1]) * s(theta[-1]) * c(phi[-1]) - c(psi[-1]) * s(phi[-1]) ) - Ay * ydot[-1] / m   )

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


time_integral_of_motor_speeds = h * ( sum(w1) + sum(w2) + sum(w3) + sum(w4) )
print 'time_integral_of_motor_speeds = ',time_integral_of_motor_speeds

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




