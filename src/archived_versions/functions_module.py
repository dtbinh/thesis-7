from numpy import cos as c, sin as s , sqrt, array, dot
from numpy.linalg import inv 



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


#-------------------------------angular setpoints
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


#--------------------------------the jacobian for transforming from body frame to inertial frame

def J(ph,th):

    global Ixx 
    global Iyy 
    global Izz 

    return array([
    [Ixx        ,                               0  , -Ixx * s(th)                ],
    [0          , Iyy*(c(ph)**2) + Izz * s(ph)**2  , (Iyy-Izz)*c(ph)*s(ph)*c(th) ],
    [-Ixx*s(th) , (Iyy-Izz)*c(ph)*s(ph)*c(th)      , Ixx*(s(th)**2) + Iyy*(s(th)**2)*(c(th)**2) + Izz*(c(ph)**2)*(c(th)**2)]    
    ])    


#------------------------------------------------------------------coriolis matrix
'''
usage for coriolis matrix:

coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz):

'''
def coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz): 
    
    c11 = 0

    c12 = (iyy-izz) * ( thd*c(ph)*s(ph) + psd*c(th)*s(ph)**2 )  + (izz-iyy)*psd*(c(ph)**2)*c(th) - ixx*psd*c(th)

    c13 = (izz-iyy) * psd * c(ph) * s(ph) * c(th)**2

    c21 = (izz-iyy) * ( thd*c(ph)*s(ph) + psd*s(ph)*c(th) ) + (iyy-izz) * psd * (c(ph)**2) * c(th) + ixx * psd * c(th)

    c22 = (izz-iyy)*phd*c(ph)*s(ph)

    c23 = -ixx*psd*s(th)*c(th) + iyy*psd*(s(ph)**2)*s(th)*c(th)

    c31 = (iyy-izz)*phd*(c(th)**2)*s(ph)*c(ph) - ixx*thd*c(th)

    c32 = (izz-iyy)*( thd*c(ph)*s(ph)*s(th) + phd*(s(ph)**2)*c(th) ) + (iyy-izz)*phd*(c(ph)**2)*c(th) + ixx*psd*s(th)*c(th) - iyy*psd*(s(ph)**2)*s(th)*c(th) - izz*psd*(c(ph)**2)*s(th)*c(th)

    c33 = (iyy-izz) * phd *c(ph)*s(ph)*(c(th)**2) - iyy * thd*(s(ph)**2) * c(th)*s(th) - izz*thd*(c(ph)**2)*c(th)*s(th) + ixx*thd*c(th)*s(th)

    return array([
                   [c11,c12,c13],
                   [c21,c22,c23],
                   [c31,c32,c33]
    	              ])


#--------------------------------------------------------------------------------------------------------------

# perform one step in the simulation , ie go to a single setpoint and hover until 'max_iteraions
'''
def go(
        x_des,
        y_des,
        z_des,
        h,
        x,y,z,xdot,ydot,zdot,xddot,yddot,zddot,
        phi,theta,psi,phidot,thetadot,psidot,phiddot,thetaddot,psiddot,
        w1,w2,w3,w4,
        tao_phi,tao_theta,tao_psi, 
        T,
        max_total_thrust,
        min_total_thrust,
        wind_x,
        wind_y,
        wind_z
      ):
    for i in range(max_iterations):
        
        system_iteration(
                         x_des,
                         y_des,
                         z_des,
                         h,
                         x,y,z,xdot,ydot,zdot,xddot,yddot,zddot,
                         phi,theta,psi,phidot,thetadot,psidot,phiddot,thetaddot,psiddot,
                         w1,w2,w3,w4,
                         tao_phi,tao_theta,tao_psi, 
                         T,
                         max_total_thrust,
                         min_total_thrust,
                         wind_x[i],
                         wind_y[i],
                         wind_z[i]
                         )

'''
#--------------------------------------------------------------------------------------------------------------


def system_iteration(x_des,
                     y_des,
                     z_des,
                     h,
                     x,y,z,xdot,ydot,zdot,xddot,yddot,zddot,
                     phi,theta,psi,phidot,thetadot,psidot,phiddot,thetaddot,psiddot,
                     w1,w2,w3,w4,
                     tao_phi,tao_theta,tao_psi, 
                     T,
                     max_total_thrust,
                     min_total_thrust,
                     ith_wind_x,
                     ith_wind_y,
                     ith_wind_z,
                     kpx = 10.0,
                     kpy = 10.0,
                     kpz = 8.0,
                     kdx = 5.0,
                     kdy = 5.0,
                     kdz = 5.0,
                     kix = 1, 
                     kiy = 1,
                     kiz = 1,
                     xError,
                     yError,
                     zError
                     ):


    kpphi   = 8.0
    kptheta = 6.0
    kppsi   = 6.0


    kdphi   = 1.75
    kdtheta = 1.75
    kdpsi   = 1.75


    #--------------------------------nonlinear roll/pitch control law gains
    sPh1 = 3
    sPh2 = 3                       
    sPh3 = 2
    sPh4 = .1
    sTh1 = 3
    sTh2 = 3
    sTh3 = 2
    sTh4 = .1



    #--------------------------------control expressions  eq 23


    # the following two expressions for total thrust and torque about z axis are from 'teppo_luukkenon'

    T_k = (g + kdz*( zdot_des - zdot[-1] ) + kpz*( z_des - z[-1] ) + kiz*zError )* m/float( c(phi[-1]) * c(theta[-1]))

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

    #note the "wind" is introduced as an acceleration which imparts exogenous forces in the position control law expressions 



    xddot.append(    (T[-1]/m)  * ( c(psi[-1]) * s(theta[-1]) * c(phi[-1]) + s(psi[-1]) * s(phi[-1]) ) 
                    - (Ax * xdot[-1]/m )  
                    + (-kdx * xdot[-1] - kpx * ( x[-1] - x_des ) + kix * xError )
                    + ith_wind_x * Ax / m
                )



    yddot.append(   ( T[-1] / m ) * ( s(psi[-1]) * s(theta[-1]) * c(phi[-1]) - c(psi[-1]) * s(phi[-1]) )
                    - ( Ay * ydot[-1] / m )  
                    + (-kdy * ydot[-1] - kpy * ( y[-1] - y_des ) + kiy * yError )
                    + ith_wind_y * Ay / m
                )
#    xddot.append(   ( T[-1] / m ) * ( c(psi[-1]) * s(theta[-1]) * c(phi[-1]) + s(psi[-1]) * s(phi[-1]) ) - Ax * xdot[-1] / m   )

#    yddot.append(   ( T[-1] / m ) * ( s(psi[-1]) * s(theta[-1]) * c(phi[-1]) - c(psi[-1]) * s(phi[-1]) ) - Ay * ydot[-1] / m   )

    zddot.append(   -g + ( T[-1] / m ) * ( c(theta[-1]) * c(phi[-1]) ) - Az * ( zdot[-1] / m ) + ith_wind_z * Az /m)
 
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

    return [x,y,z,xdot,ydot,zdot,xddot,yddot,zddot,
            phi,theta,psi,phidot,thetadot,psidot,phiddot,thetaddot,psiddot,
            w1,w2,w3,w4,
            tao_phi,tao_theta,tao_psi, 
            T
           ]



'''
#----------------------------------------------------------------PID integral terms


#phiError = 0

def Xint(x,xError,x_des,h):
	xError += (x[-1]-x_des)*h
	return xError

def Yint(y,yError,y_des,h):
	yError += (y[-1]-y_des)*h
	return yError

def phiint(phi,phi_Error,phi_des,h):
	phiError += (phi[-1]-phi_des)*h
	return phiError
'''
#----------------------------------------------------------------------PLOTTING FUNCTION


def plot_results(h,x,y,z,phi,theta,psi,w1,w2,w3,w4,tao_phi,tao_theta,tao_psi,T,wind_x,wind_y,wind_z):

    timeSeries = [h*i for i in range(len(x))]

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from pylab import title   
    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(10, 6)

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
    plt.plot(timeSeries[:-1], w1,'r',  timeSeries[:-1], w2,'g',  timeSeries[:-1], w3,'b',  timeSeries[:-1], w4,'k')
    title('motor speeds',fontsize=10)

    #------------------------------------------------------------------------------torque
    spd = fig.add_subplot(gs[6:8,0:3])
    plt.plot(timeSeries[:-1], tao_phi,'r',  timeSeries[:-1], tao_theta,'g',  timeSeries[:-1], tao_psi,'b')
    title('torques ',fontsize=10)

    #------------------------------------------------------------------------------thrust
    spd = fig.add_subplot(gs[6:8,3:6])
    plt.plot(timeSeries[:-1], T,'r',)
    title('Total Thrust',fontsize=10)

    #------------------------------------------------------------------------------wind velocities

    wind_plot = fig.add_subplot( gs[ 8:10 ,0:3 ] )

    plt.plot(timeSeries, wind_x[:len(timeSeries)],'-r',
             timeSeries, wind_y[:len(timeSeries)],'-g',
             timeSeries, wind_z[:len(timeSeries)],'-b'  )


    title('wind velocities, rgb = xyz', fontsize=10)

    fig.tight_layout()


    plt.show()            
