from agent_module import *

from numpy import sin, cos, array


def control_block(a):

    # using the comm linear accellerations, calc  theta_c, phi_c and T_c 


    theta_numerator = (a.xacc_comm * c(a.psi[-1]) + a.yacc_comm * s(a.psi[-1]) )


    theta_denominator = float( a.zacc_comm + a.g )

    if theta_denominator <= 0:
        
        theta_denominator = 0.1           # don't divide by zero !!!


    a.theta_comm.append(arctan2( theta_numerator ,  theta_denominator ))


    a.phi_comm.append(arcsin( (a.xacc_comm * s(a.psi[-1]) - a.yacc_comm * c(a.psi[-1]) ) / float(sqrt( a.xacc_comm**2 + 
                                                                                                       a.yacc_comm**2 +    
                                                                                                       (a.zacc_comm + a.g)**2  )) ))


    a.T_comm.append(a.m * ( a.xacc_comm * ( s(a.theta[-1])*c(a.psi[-1])*c(a.phi[-1]) + s(a.psi[-1])*s(a.phi[-1]) ) +
                            a.yacc_comm * (  s(a.theta[-1])*s(a.psi[-1])*c(a.phi[-1]) - c(a.psi[-1])*s(a.phi[-1]) ) +
                            (a.zacc_comm + a.g) * ( c(a.theta[-1])*c(a.phi[-1])  ) 
                                ))

    if a.T_comm[-1] < 1.0:
        a.T_comm = a.T_comm[:-1] 
        a.T_comm.append(1.0)

    # we will need the derivatives of the comanded angles for the torque control laws.
    a.phidot_comm = (a.phi_comm[-1] - a.phi_comm[-2])/a.h

    a.thetadot_comm = (a.theta_comm[-1] - a.theta_comm[-2])/a.h

    # solve for torques based on theta_c, phi_c and T_c , also psi_des , and previous values of theta, phi, and psi

    a.tao_phi_comm.append(( a.kpphi*(a.phi_comm[-1] - a.phi[-1]) + a.kdphi*(a.phidot_comm - a.phidot[-1]) )*a.Ixx) 
    a.tao_theta_comm.append(( a.kptheta*(a.theta_comm[-1] - a.theta[-1]) + a.kdtheta*(a.thetadot_comm - a.thetadot[-1]) )*a.Iyy )
    a.tao_psi_comm.append(( a.kppsi*(a.psi_des - a.psi[-1]) + a.kdpsi*( a.psidot_des - a.psidot[-1] ) )*a.Izz )

    #--------------------------------solve for motor speeds, eq 24

    a.w1_arg.append( (a.T_comm[-1] / (4.0*a.k)) - ( a.tao_theta_comm[-1] / (2.0*a.k*a.L) )  - ( a.tao_psi_comm[-1] / (4.0*a.b) ) ) 
    a.w2_arg.append( (a.T_comm[-1] / (4.0*a.k)) - ( a.tao_phi_comm[-1]   / (2.0*a.k*a.L) )  + ( a.tao_psi_comm[-1] / (4.0*a.b) ) )
    a.w3_arg.append( (a.T_comm[-1] / (4.0*a.k)) + ( a.tao_theta_comm[-1] / (2.0*a.k*a.L) )  - ( a.tao_psi_comm[-1] / (4.0*a.b) ) )
    a.w4_arg.append( (a.T_comm[-1] / (4.0*a.k)) + ( a.tao_phi_comm[-1]   / (2.0*a.k*a.L) )  + ( a.tao_psi_comm[-1] / (4.0*a.b) ) )


    a.w1.append( sqrt( a.w1_arg[-1] )  )
    a.w2.append( sqrt( a.w2_arg[-1] )  )
    a.w3.append( sqrt( a.w3_arg[-1] )  )
    a.w4.append( sqrt( a.w4_arg[-1] )  )



###############################################################

if __name__ == "__main__":


    a = agent(x_0 = 0,    
                  y_0 = 0,       
                  z_0 = 0,     
                  initial_setpoint_x = 1,     
                  initial_setpoint_y = 1,     
                  initial_setpoint_z = 1,     
                  agent_priority = 1)

    time_series = [a.h*i for i in range(a.max_iterations)]

    r1 = 1
    r2 = 1

    x_t = [ r1*sin(t) for t in time_series ]

    y_t = [ r1*cos(t) for t in time_series ]

    z_t = [ 1 for t in time_series ] 


    vx = [ (x_t[i+1] - x_t[i]) / a.h for i in range(len(x_t)-1)  ]
    vy = [ (y_t[i+1] - y_t[i]) / a.h for i in range(len(y_t)-1)  ]
    vz = [ (z_t[i+1] - z_t[i]) / a.h for i in range(len(z_t)-1)  ]

    ax = [ (vx[i+1] - vx[i] ) / a.h for i in range(len(vx)-1) ]  
    ay = [ (vy[i+1] - vy[i] ) / a.h for i in range(len(vy)-1) ]
    az = [ (vz[i+1] - vz[i] ) / a.h for i in range(len(vz)-1) ]


    a.x[-1] = 1
    a.y[-1] = 1
    a.z[-1] = 1

    a.xdot[-1] = vx[-1]
    a.ydot[-1] = vy[-1]
    a.zdot[-1] = vz[-1]

    a.xddot[-1] = ax[-1]
    a.yddot[-1] = ay[-1]
    a.zddot[-1] = az[-1]

    for acc in range(len(ax)):

        a.xacc_comm = ax[acc]
        a.yacc_comm = ay[acc]
        a.zacc_comm = az[acc]

        control_block(a)

    for w in range(len(a.w1)):

        

        # BELOW ARE THE EQUATIONS THAT MODEL THE SYSTEM,
        # FOR THE PURPOSE OF SIMULATION, GIVEN THE MOTOR SPEEDS WE CAN CALCULATE THE STATES OF THE SYSTEM

        a.tao_qr_frame.append( array([ 
                                a.L*a.k*( -a.w2[-1]**2 + a.w4[-1]**2 ) , 
                                a.L*a.k*( -a.w1[-1]**2 + a.w3[-1]**2 ) ,
                                a.b* ( -a.w1[-1]**2 + a.w2[-1]**2 - a.w3[-1]**2 + a.w4[-1]**2 )
                             ]) )

        a.tao_phi.append(a.tao_qr_frame[-1][0])
        a.tao_theta.append(a.tao_qr_frame[-1][1])    
        a.tao_psi.append(a.tao_qr_frame[-1][2])

        a.T.append(a.k*( a.w1[-1]**2 + a.w2[-1]**2 + a.w3[-1]**2 + a.w4[-1]**2 ) )


        # use the previous known angles and the known thrust to calculate the new resulting linear accelerations

        # remember this would be measured ,

        # for the purpose of modeling the measurement error and testing a kalman filter, inject noise here...
        # perhaps every 1000ms substitute an artificial gps measurement (and associated uncertianty) for the double integrated imu value

        a.xddot.append(  (a.T[-1]/a.m)*( c(a.psi[-1])*s(a.theta[-1])*c(a.phi[-1]) 
                            + s(a.psi[-1])*s(a.phi[-1]) ) 
                            - a.Ax * a.xdot[-1] / a.m  )
        
        a.yddot.append(  (a.T[-1]/a.m)*( s(a.psi[-1])*s(a.theta[-1])*c(a.phi[-1]) 
                            - c(a.psi[-1])*s(a.phi[-1]) ) 
                            - a.Ay * a.ydot[-1] / a.m  )        

        a.zddot.append(  a.g + (a.T[-1]/a.m)*( c(a.theta[-1])*c(a.phi[-1]) ) - a.Az * a.zdot[-1] / a.m  )

        # calculate the new angular accelerations based on the known values
        a.etadot.append( array( [a.phidot[-1], a.thetadot[-1], a.psidot[-1] ] ) )

        a.etaddot.append( dot(inv( a.J() ), a.tao_qr_frame[-1]  -  dot(a.coriolis_matrix() , a.etadot[-1]) ) )

        # parse the etaddot vector of the new accelerations into the appropriate time series'
        a.phiddot.append(a.etaddot[-1][0])       

        a.thetaddot.append(a.etaddot[-1][1])

        a.psiddot.append(a.etaddot[-1][2])

        #------------------------------ integrate new acceleration values to obtain velocity values

        a.xdot.append(  a.xdot[-1] + a.xddot[-1] * a.h  )
        a.ydot.append(  a.ydot[-1] + a.yddot[-1] * a.h  )
        a.zdot.append(  a.zdot[-1] + a.zddot[-1] * a.h  )

        a.phidot.append(    a.phidot[-1]   + a.phiddot[-1] * a.h  )
        a.thetadot.append(  a.thetadot[-1] + a.thetaddot[-1] * a.h  )
        a.psidot.append(    a.psidot[-1]   + a.psiddot[-1] * a.h  )
    
        #------------------------------ integrate new velocity values to obtain position / angle values

        a.x.append(  a.x[-1] + a.xdot[-1] * a.h  )
        a.y.append(  a.y[-1] + a.ydot[-1] * a.h  )
        a.z.append(  a.z[-1] + a.zdot[-1] * a.h  )

        a.phi.append(  a.phi[-1] + a.phidot[-1] * a.h  )
        a.theta.append(  a.theta[-1] + a.thetadot[-1] * a.h  )
        a.psi.append( a.psi[-1] + a.psidot[-1] * a.h  )




    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from pylab import title   

    fig1 = plt.figure()

    axeees = fig1.add_subplot(111, projection='3d')

    axeees.scatter(a.x,a.y,a.z)

    axeees.set_xlabel('X')
    axeees.set_ylabel('Y')
    axeees.set_zlabel('Z')

    plt.show()


