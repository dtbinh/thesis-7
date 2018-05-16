
'''
here is an outline of the program flow:

    1) initialize lists for state variables and appropriate derivatives 
       as well as constants.
    2) calculate total thrust T and the torques using the state variables from
       the [k]th time step
    3) using T[k] and tao_[k] calculate new motor speed vector for time [k]
    4) using T[k] and tao_[k] calculate new state variables at time [k+1]
    5) repeat 
'''

from numpy import cos as c, sin as s , sqrt, array, dot, arctan2, arcsin, sign , around
from numpy.linalg import inv 
from wind import *
import sys
import math

################################################################################

class agent:

    def __init__(self, x_0,   y_0,   z_0,
                 initial_setpoint_x,  initial_setpoint_y, initial_setpoint_z, 
                 agent_priority = 10 ):

    


        #-------------------------------- physical constants
        self.g = -9.81    #[m/s^2]
        self.m = 1       #[kg]
        self.L = 1     #[m]
        self.b = 10**-6
        self.k = self.m*abs(self.g)/(4*(1000**2))
        #print 'self.k = ',self.k
        #raw_input()
        #--------------------------------moments of inertia
        self.Ixx = 5.0*10**-3
        self.Iyy = 5.0*10**-3
        self.Izz = 10.0*10**-3

        #--------------------------------directional drag coefficients
        self.Ax = 0.25
        self.Ay = 0.25
        self.Az = 0.25

        self.hover_at_setpoint = True

        # a list of distances to each other agent sorted nearest to farthest
        self.distance_vectors = []
        
        self.agent_priority = agent_priority

        # state vector and  derivative time series' list initializations
        self.x = [x_0]
        self.y = [y_0]
        self.z = [z_0]

        self.xdot = [0]
        self.ydot = [0]
        self.zdot = [0]

        self.xddot = [0]
        self.yddot = [0]
        self.zddot = [0]

        self.phi   = [0]
        self.theta = [0]
        self.psi   = [0]

        self.phidot   = [0]
        self.thetadot = [0]
        self.psidot   = [0] 

        self.phiddot   = [0]
        self.thetaddot = [0]
        self.psiddot   = [0]

        self.tao_qr_frame = []
        self.etadot = []
        self.etaddot =[]


        #-------------------------------------------------------------CONTROLLER GAINS

        self.kpx = 0.          # PID proportional gain values
        self.kpy = 0.
        self.kpz = 0.

        self.kdx = 0.           # PID derivative gain values
        self.kdy = 0.
        self.kdz = 0.

        self.kddx = 0.
        self.kddy = 0.
        self.kddz = 0.

        self.kix = 0.         # PID integral gain values
        self.kiy = 0.
        self.kiz = 0.
     
        self.kpphi   = 4  # gains for the angular pid control laws
        self.kptheta = 4
        self.kppsi   = 4

        self.kiphi   = 0.
        self.kitheta = 0.
        self.kipsi   = 0.

        self.kdphi   = 5
        self.kdtheta = 5
        self.kdpsi   = 5

        self.xdd_des = 0
        self.ydd_des = 0
        self.zdd_des = 0

        self.x_integral_error = [0]
        self.y_integral_error = [0]
        self.z_integral_error = [0]

        # angular set points 
        self.phi_des = 0
        self.theta_des = 0
        self.psi_des = 0

        self.psidot_des = 0

       

        self.phi_comm = [0,0]
        self.theta_comm = [0,0]
        self.T_comm = [0,0]

        self.tao_phi_comm = [0,0]
        self.tao_theta_comm = [0,0]
        self.tao_psi_comm = [0,0]


        # force, torque, and motor speed list initializations

        self.T = [9.81]
        self.tao_phi   = [0]
        self.tao_theta = [0]
        self.tao_psi   = [0]

        self.w1 = [1000]
        self.w2 = [1000]
        self.w3 = [1000]
        self.w4 = [1000]

        self.etaddot = []
        #----------------------------------

        self.max_iterations = 10000
        self.h = 0.01
        self.ending_iteration = 0

        #-----------------------------------------------------------------wind!!
        self.wind_duration = self.max_iterations
        self.max_gust = 0.1  

        # generate the wind for the entire simulation beforehand
        self.wind_data = wind_vector_time_series(self.max_gust,self.wind_duration)

        self.wind_x = self.wind_data[0] 
        self.wind_y = self.wind_data[1]
        self.wind_z = self.wind_data[2] 


        #----------------------------------------------------------------------

        #TODO: NEED TO SORT OUT THE MIN AND MAX THRUST PARAMETERS AND CORRELATE 
        #      THIS PHYSICAL LIMITATION WITH THE MAX VALUES FOR THE PROPORTIONAL
        #      GAIN TERMS IN EACH CONTROL LAW.

        #self.max_total_thrust = 50.0  # [newtons] 
        #self.min_total_thrust = 1.0

        self.x_des = initial_setpoint_x
        self.y_des = initial_setpoint_y
        self.z_des = initial_setpoint_z

        self.xdot_des = 0
        self.ydot_des = 0
        self.zdot_des = 0

        self.initial_setpoint_x = initial_setpoint_x 
        self.initial_setpoint_y = initial_setpoint_y
        self.initial_setpoint_z = initial_setpoint_z

        #----------------------------------------------------------------------


        self.w1_arg = [0,0]
        self.w2_arg = [0,0]
        self.w3_arg = [0,0]
        self.w4_arg = [0,0]


        self.xacc_comm = [0]
        self.yacc_comm = [0]
        self.zacc_comm = [19.62]




    ######################################################  END   " __INIT__() "





    # the Jacobian for transforming from body frame to inertial frame

    def J(self):

        ixx = self.Ixx 
        iyy = self.Iyy
        izz = self.Izz

        th = self.theta[-1]
        ph = self.phi[-1]
   

        j11 = ixx

        j12 = 0

        j13 = -ixx * s(th)

        j21 = 0

        j22 = iyy*(c(ph)**2) + izz * s(ph)**2

        j23 = (iyy-izz)*c(ph)*s(ph)*c(th)

        j31 = -ixx*s(th)

        j32 = (iyy-izz)*c(ph)*s(ph)*c(th)

        j33 = ixx*(s(th)**2) + iyy*(s(th)**2)*(c(th)**2) + izz*(c(ph)**2)*(c(th)**2)          

        return array([
                     [j11, j12, j13],
                     [j21, j22, j23],
                     [j31, j32, j33]
        	         ])





    #-------------------Coriolis matrix

    def coriolis_matrix(self): 

        ph = self.phi[-1]
        th = self.theta[-1]

        phd = self.phidot[-1]
        thd = self.thetadot[-1]
        psd = self.psidot[-1]

        ixx = self.Ixx 
        iyy = self.Iyy
        izz = self.Izz

        c11 = 0

        # break up the large elements in to bite size chunks and then add each term ...

        c12_term1 = (iyy-izz) * ( thd*c(ph)*s(ph) + psd*c(th)*s(ph)**2 )

        c12_term2and3 = (izz-iyy)*psd*(c(ph)**2)*c(th) - ixx*psd*c(th)

        c12 = c12_term1  + c12_term2and3



        c13 = (izz-iyy) * psd * c(ph) * s(ph) * c(th)**2



        c21_term1 = (izz-iyy) * ( thd*c(ph)*s(ph) + psd*s(ph)*c(th) )

        c21_term2and3 = (iyy-izz) * psd * (c(ph)**2) * c(th) + ixx * psd * c(th)

        c21 = c21_term1 + c21_term2and3



        c22 = (izz-iyy)*phd*c(ph)*s(ph)

        c23 = -ixx*psd*s(th)*c(th) + iyy*psd*(s(ph)**2)*s(th)*c(th)

        c31 = (iyy-izz)*phd*(c(th)**2)*s(ph)*c(ph) - ixx*thd*c(th)



        c32_term1     = (izz-iyy)*( thd*c(ph)*s(ph)*s(th) + phd*(s(ph)**2)*c(th) )

        c32_term2and3 = (iyy-izz)*phd*(c(ph)**2)*c(th) + ixx*psd*s(th)*c(th)

        c32_term4 = - iyy*psd*(s(ph)**2)*s(th)*c(th)
         
        c32_term5 = - izz*psd*(c(ph)**2)*s(th)*c(th)

        c32 = c32_term1 + c32_term2and3 + c32_term4 + c32_term5



        c33_term1 = (iyy-izz) * phd *c(ph)*s(ph)*(c(th)**2)

        c33_term2 = - iyy * thd*(s(ph)**2) * c(th)*s(th)

        c33_term3and4 = - izz*thd*(c(ph)**2)*c(th)*s(th) + ixx*thd*c(th)*s(th)

        c33 = c33_term1 + c33_term2 + c33_term3and4


        return array([
                       [c11,c12,c13],
                       [c21,c22,c23],
                       [c31,c32,c33]
        	              ])


    #-----------------------------------------------------------------------


    def control_block(self):
       

        # calculate the integral of the error in position for each direction
        
        self.x_integral_error.append( self.x_integral_error[-1] + (self.x_des - self.x[-1])*self.h )
        self.y_integral_error.append( self.y_integral_error[-1] + (self.y_des - self.y[-1])*self.h )
        self.z_integral_error.append( self.z_integral_error[-1] + (self.z_des - self.z[-1])*self.h )


        # computte the comm linear accelerations needed to move the system from present location to the desired location
    
        self.xacc_comm.append( self.kdx * (self.xdot_des - self.xdot[-1]) 
                             + self.kpx * ( self.x_des - self.x[-1] ) 
                             + self.kddx * (self.xdd_des - self.xddot[-1] ) 
                             + self.kix * self.x_integral_error[-1]  )


        self.yacc_comm.append( self.kdy * (self.ydot_des - self.ydot[-1]) 
                                  + self.kpy * ( self.y_des - self.y[-1] ) 
                                  + self.kddy * (self.ydd_des - self.yddot[-1] ) 
                                  + self.kiy * self.y_integral_error[-1]  )


        self.zacc_comm.append( self.kdz * (self.zdot_des - self.zdot[-1]) 
                                  + self.kpz * ( self.z_des - self.z[-1] ) 
                                  + self.kddz * (self.zdd_des - self.zddot[-1] )  
                                  + self.kiz * self.z_integral_error[-1] )


        # need to limit the max linear acceleration that is perscribed by the control law

        # as a meaningful place to start, just use the value '10m/s/s' , compare to g = -9.8 ... 

        max_latt_acc = 5

        max_z_acc = 30    

        if abs(self.xacc_comm[-1]) > max_latt_acc: self.xacc_comm[-1] = max_latt_acc * sign(self.xacc_comm[-1])
        if abs(self.yacc_comm[-1]) > max_latt_acc: self.yacc_comm[-1] = max_latt_acc * sign(self.yacc_comm[-1])            
        if abs(self.zacc_comm[-1]) > max_z_acc: self.zacc_comm[-1] = max_z_acc * sign(self.zacc_comm[-1])

        min_z_acc = 12

        if self.zacc_comm[-1] < min_z_acc: self.zacc_comm[-1] = min_z_acc

        # using the comm linear accellerations, calc  theta_c, phi_c and T_c 

        theta_numerator = (self.xacc_comm[-1] * c(self.psi[-1]) + self.yacc_comm[-1] * s(self.psi[-1]) )

        theta_denominator = float( self.zacc_comm[-1] + self.g )

        if theta_denominator <= 0:
            
            theta_denominator = 0.1           # don't divide by zero !!!
    
        self.theta_comm.append(arctan2( theta_numerator ,  theta_denominator ))

        self.phi_comm.append(arcsin( (self.xacc_comm[-1] * s(self.psi[-1]) - self.yacc_comm[-1] * c(self.psi[-1]) ) / float(sqrt( self.xacc_comm[-1]**2 + 
                                                                                                                     self.yacc_comm[-1]**2 +    
                                                                                                                    (self.zacc_comm[-1] + self.g)**2  )) ))

        self.T_comm.append(self.m * ( self.xacc_comm[-1] * ( s(self.theta[-1])*c(self.psi[-1])*c(self.phi[-1]) + s(self.psi[-1])*s(self.phi[-1]) ) +
                                      self.yacc_comm[-1] * (  s(self.theta[-1])*s(self.psi[-1])*c(self.phi[-1]) - c(self.psi[-1])*s(self.phi[-1]) ) +
                                     (self.zacc_comm[-1] + self.g) * ( c(self.theta[-1])*c(self.phi[-1])  ) 
                                    ))

        if self.T_comm[-1] < 1.0:
            self.T_comm = self.T_comm[:-1] 
            self.T_comm.append(1.0)


        # we will need the derivatives of the comanded angles for the torque control laws.
        self.phidot_comm = (self.phi_comm[-1] - self.phi_comm[-2])/self.h

        self.thetadot_comm = (self.theta_comm[-1] - self.theta_comm[-2])/self.h


        # solve for torques based on theta_c, phi_c and T_c , also psi_des , and previous values of theta, phi, and psi


        tao_phi_comm_temp = ( self.kpphi*(self.phi_comm[-1] - self.phi[-1]) + self.kdphi*(self.phidot_comm - self.phidot[-1]) )*self.Ixx
                
        tao_theta_comm_temp = ( self.kptheta*(self.theta_comm[-1] - self.theta[-1]) + self.kdtheta*(self.thetadot_comm - self.thetadot[-1]) )*self.Iyy

        tao_psi_comm_temp = ( self.kppsi*(self.psi_des - self.psi[-1]) + self.kdpsi*( self.psidot_des - self.psidot[-1] ) )*self.Izz

        self.tao_phi_comm.append(tao_phi_comm_temp ) 
        self.tao_theta_comm.append(tao_theta_comm_temp )
        self.tao_psi_comm.append(tao_psi_comm_temp )

        #--------------------------------solve for motor speeds, eq 24

        self.w1_arg.append( (self.T_comm[-1] / (4.0*self.k)) - ( self.tao_theta_comm[-1] / (2.0*self.k*self.L) )  - ( self.tao_psi_comm[-1] / (4.0*self.b) ) ) 
        self.w2_arg.append( (self.T_comm[-1] / (4.0*self.k)) - ( self.tao_phi_comm[-1]   / (2.0*self.k*self.L) )  + ( self.tao_psi_comm[-1] / (4.0*self.b) ) )
        self.w3_arg.append( (self.T_comm[-1] / (4.0*self.k)) + ( self.tao_theta_comm[-1] / (2.0*self.k*self.L) )  - ( self.tao_psi_comm[-1] / (4.0*self.b) ) )
        self.w4_arg.append( (self.T_comm[-1] / (4.0*self.k)) + ( self.tao_phi_comm[-1]   / (2.0*self.k*self.L) )  + ( self.tao_psi_comm[-1] / (4.0*self.b) ) )

        self.w1.append( sqrt( self.w1_arg[-1] )  )
        self.w2.append( sqrt( self.w2_arg[-1] )  )
        self.w3.append( sqrt( self.w3_arg[-1] )  )
        self.w4.append( sqrt( self.w4_arg[-1] )  )

        # IMPORTANT!!! THIS ENDS THE 'CONTROLLER BLOCK' IN A REAL IMPLEMENTATION, WE WOULD NOW TAKE MEASUREMENTS AND ESTIMATE THE STATE and then start over...

    def system_model_block(self):

        # BELOW ARE THE EQUATIONS THAT MODEL THE SYSTEM,
        # FOR THE PURPOSE OF SIMULATION, GIVEN THE MOTOR SPEEDS WE CAN CALCULATE THE STATES OF THE SYSTEM

        self.tao_qr_frame.append( array([ 
                                self.L*self.k*( -self.w2[-1]**2 + self.w4[-1]**2 ) , 
                                self.L*self.k*( -self.w1[-1]**2 + self.w3[-1]**2 ) ,
                                self.b* ( -self.w1[-1]**2 + self.w2[-1]**2 - self.w3[-1]**2 + self.w4[-1]**2 )
                             ]) )

        self.tao_phi.append(self.tao_qr_frame[-1][0])
        self.tao_theta.append(self.tao_qr_frame[-1][1])    
        self.tao_psi.append(self.tao_qr_frame[-1][2])

        self.T.append(self.k*( self.w1[-1]**2 + self.w2[-1]**2 + self.w3[-1]**2 + self.w4[-1]**2 ) )

        # use the previous known angles and the known thrust to calculate the new resulting linear accelerations
        # remember this would be measured ...
        # for the purpose of modeling the measurement error and testing a kalman filter, inject noise here...
        # perhaps every 1000ms substitute an artificial gps measurement (and associated uncertianty) for the double integrated imu value

        self.xddot.append(  (self.T[-1]/self.m)*( c(self.psi[-1])*s(self.theta[-1])*c(self.phi[-1]) 
                            + s(self.psi[-1])*s(self.phi[-1]) ) 
                            - self.Ax * self.xdot[-1] / self.m  )
        
        self.yddot.append(  (self.T[-1]/self.m)*( s(self.psi[-1])*s(self.theta[-1])*c(self.phi[-1]) 
                            - c(self.psi[-1])*s(self.phi[-1]) ) 
                            - self.Ay * self.ydot[-1] / self.m  )        

        self.zddot.append(  self.g + (self.T[-1]/self.m)*( c(self.theta[-1])*c(self.phi[-1]) ) - self.Az * self.zdot[-1] / self.m  )

        # calculate the new angular accelerations based on the known values
        self.etadot.append( array( [self.phidot[-1], self.thetadot[-1], self.psidot[-1] ] ) )

        self.etaddot.append( dot(inv( self.J() ), self.tao_qr_frame[-1]  -  dot(self.coriolis_matrix() , self.etadot[-1]) ) )

        # parse the etaddot vector of the new accelerations into the appropriate time series'
        self.phiddot.append(self.etaddot[-1][0])       

        self.thetaddot.append(self.etaddot[-1][1])

        self.psiddot.append(self.etaddot[-1][2])

        #------------------------------ integrate new acceleration values to obtain velocity values

        self.xdot.append(  self.xdot[-1] + self.xddot[-1] * self.h  )
        self.ydot.append(  self.ydot[-1] + self.yddot[-1] * self.h  )
        self.zdot.append(  self.zdot[-1] + self.zddot[-1] * self.h  )

        self.phidot.append(    self.phidot[-1]   + self.phiddot[-1] * self.h  )
        self.thetadot.append(  self.thetadot[-1] + self.thetaddot[-1] * self.h  )
        self.psidot.append(    self.psidot[-1]   + self.psiddot[-1] * self.h  )
    
        #------------------------------ integrate new velocity values to obtain position / angle values

        self.x.append(  self.x[-1] + self.xdot[-1] * self.h  )
        self.y.append(  self.y[-1] + self.ydot[-1] * self.h  )
        self.z.append(  self.z[-1] + self.zdot[-1] * self.h  )

        self.phi.append(  self.phi[-1] + self.phidot[-1] * self.h  )
        self.theta.append(  self.theta[-1] + self.thetadot[-1] * self.h  )
        self.psi.append( self.psi[-1] + self.psidot[-1] * self.h  )



    ############################################################################
       

    def plot_results(self,show_plot = True, save_plot = False, fig1_file_path = None,fig2_file_path = None):

        timeSeries = [self.h*i for i in range(len(self.x))]

        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt
        from pylab import title   
        import matplotlib.gridspec as gridspec


        fig0 = plt.figure()


        ax = fig0.add_subplot(111, projection='3d') 

        ax.scatter(
                    self.x[0:len(self.x):5], 
                    self.y[0:len(self.y):5], 
                    self.z[0:len(self.z):5])

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')


        fig1 = plt.figure()
        gs1 = gridspec.GridSpec(3, 2)

        #---------------------------------------------------linear displacements
        xx = fig1.add_subplot(gs1[0,0])
        plt.plot(timeSeries, self.x,'r',  
                 timeSeries, self.y,'g',  
                 timeSeries, self.z,'b')
        title('x,y,z',fontsize=10)
        plt.xlabel('time (s)',fontsize=10)
        plt.ylabel('dist. (m)',fontsize=10)

        #-----------------------------------------------------------------angles
        thth = fig1.add_subplot(gs1[1,0])
        plt.plot(timeSeries, self.phi,'r',  
                 timeSeries, self.theta,'g',  
                 timeSeries, self.psi,'b')
        title('Phi = roll, Theta = pitch, Psi = yaw',fontsize=10)
        plt.xlabel('time (s)',fontsize=10)
        plt.ylabel('(rad)',fontsize=10)
        #-----------------------------------------------------------motor speeds
       

        spd = fig1.add_subplot(gs1[2,0])
        plt.plot([self.h*i for i in range(len(self.w1))], self.w1,'r',  
                 [self.h*i for i in range(len(self.w2))], self.w2,'g',  
                 [self.h*i for i in range(len(self.w3))], self.w3,'b',  
                 [self.h*i for i in range(len(self.w4))], self.w4,'k')
        title('motor speeds',fontsize=10)
        plt.xlabel('time (s)',fontsize=10)
        plt.ylabel('(rad/sec)',fontsize=10)
        #-----------------------------------------------------------------torque
        spd = fig1.add_subplot(gs1[0,1])
        plt.plot([self.h*i for i in range(len(self.tao_phi))], self.tao_phi,'r',  
                 [self.h*i for i in range(len(self.tao_theta))], self.tao_theta,'g',  
                 [self.h*i for i in range(len(self.tao_psi))], self.tao_psi,'b')
        title('torques ',fontsize=10)
        plt.xlabel('time (s)',fontsize=10)
        plt.ylabel('(Nm)',fontsize=10)
        #-----------------------------------------------------------------thrust
        spd = fig1.add_subplot(gs1[1,1])
        plt.plot([self.h*i for i in range(len(self.T))], self.T,'r',)
        title('Total Thrust',fontsize=10)
        plt.xlabel('time (s)',fontsize=10)
        plt.ylabel('(N)',fontsize=10)
        

        #----------------------------------------------------------------commanded total thrust
        t_comm = fig1.add_subplot(gs1[2,1]) 
        plt.plot([self.h*i for i in range(len(self.T_comm))], self.T_comm,'r')
        title('T_comm',fontsize=10)



        '''
        #--------------------------------------------------------wind velocities
        wind_plot = fig1.add_subplot( gs1[ 8:10 ,0:3 ] )

        plt.plot([self.h*i for i in range(len(self.wind_x))], self.wind_x,'-r',
                 [self.h*i for i in range(len(self.wind_y))], self.wind_y,'-g',
                 [self.h*i for i in range(len(self.wind_z))], self.wind_z,'-b'  )

        title('wind velocities, rgb = xyz', fontsize=10)
        '''
        fig1.tight_layout()

        #############################################

        

        fig2 = plt.figure()
        gs2 = gridspec.GridSpec(4, 2)

        #-----------------------------------------------------------------linear velocities
        lin_vel = fig2.add_subplot(gs2[0,0])
        plt.plot(timeSeries, self.xdot,'r',  
                 timeSeries, self.ydot,'g',  
                 timeSeries, self.zdot,'b')
        title('xdot, ydot, self.zdot',fontsize=10)

        #-----------------------------------------------------------------linear accelerations
        lin_acc = fig2.add_subplot(gs2[1,0])
        plt.plot(timeSeries, self.xddot,'r',  
                 timeSeries, self.yddot,'g',  
                 timeSeries, self.zddot,'b')
        title('xddot, yddot, self.zddot',fontsize=10)


        #-----------------------------------------------------------------angular velocities
        ang_vel = fig2.add_subplot(gs2[2,0])
        plt.plot(timeSeries, self.phidot,'r',  
                 timeSeries, self.thetadot,'g',  
                 timeSeries, self.psidot,'b')
        title('phidot, thetadot, self.psidot',fontsize=10)

        
        #----------------------------------------------------------------commanded torques
        t_comm  = fig2.add_subplot(gs2[3,0])
        plt.plot([self.h*i for i in range(len(self.tao_phi_comm))], self.tao_phi_comm,'r',
                 [self.h*i for i in range(len(self.tao_theta_comm))], self.tao_theta_comm,'g',
                 [self.h*i for i in range(len(self.tao_psi_comm))], self.tao_psi_comm,'b',
                 )
        title('tao_phi_comm,tao_theta_comm,tao_psi_comm',fontsize=10)
        


        #-----------------------------------------------------------------angular velocities
        ang_acc = fig2.add_subplot(gs2[0,1])
        plt.plot(timeSeries, self.phiddot,'r',  
                 timeSeries, self.thetaddot,'g',  
                 timeSeries, self.psiddot,'b')
        title('phiddot, thetaddot, self.psiddot',fontsize=10)


        #-----------------------------------------------------------------integral errors
        integral_errors = fig2.add_subplot(gs2[1,1])
        plt.plot(timeSeries, self.x_integral_error,'r',  
                 timeSeries, self.y_integral_error,'g',  
                 timeSeries, self.z_integral_error,'b')
        title('x_integral_error, y_integral_error, z_integral_error',fontsize=10)


        #-----------------------------------------------------------------w_args
        integral_errors = fig2.add_subplot(gs2[2,1])
        plt.plot([self.h*i for i in range(len(self.w1_arg))], self.w1_arg,'r',  
                 [self.h*i for i in range(len(self.w2_arg))], self.w2_arg,'g',  
                 [self.h*i for i in range(len(self.w3_arg))], self.w3_arg,'b',
                 [self.h*i for i in range(len(self.w4_arg))], self.w4_arg,'k'                
                )
        title('w1_arg, w2_arg, w3_arg, w4_arg ',fontsize=10)

        #-----------------------------------------------------------------commanded phi and theta
        integral_errors = fig2.add_subplot(gs2[3,1])
        plt.plot([self.h*i for i in range(len(self.theta_comm))], self.theta_comm,'r',  
                 [self.h*i for i in range(len(self.phi_comm))], self.phi_comm,'g'                
                )
        title('theta_com ,phi_com  ',fontsize=10)



        fig2.tight_layout()

        if save_plot == True:
        
            fig1.savefig(fig1_file_path)
            fig2.savefig(fig2_file_path)

            

        if show_plot == True:

            plt.show()            

#-------------------------------------------------------------------------------

    def print_dump(self,n=5):
       
        print '\n\nself.xacc_comm[-n:] = ',around(self.xacc_comm[-n:], decimals=5)
        print '\n\nself.yacc_comm[-n:] = ',around(self.yacc_comm[-n:], decimals=5)
        print '\n\nself.zacc_comm[-n:] = ',around(self.zacc_comm[-n:], decimals=5)

        print '\n\ntheta_com[-n:] = ',around(self.theta_comm[-n:], decimals=5)

        print '\n\nphi_com[-n:] = ',around(self.phi_comm[-n:], decimals=5)

        print '\n\nT_comm[-n:] = ',around(self.T_comm[-n:], decimals=5)

        print '\n\nself.tao_phi_comm = ',around(self.tao_phi_comm[-n:], decimals=5)
        print '\n\nself.tao_theta_comm = ',around(self.tao_theta_comm[-n:], decimals=5)
        print '\n\nself.tao_psi_comm = ',around(self.tao_psi_comm[-n:], decimals=5)

        print '\n\nw1_arg[-n:] = ',around(self.w1_arg[-n:], decimals=0)
        print '\n\nw2_arg[-n:] = ',around(self.w2_arg[-n:], decimals=0)
        print '\n\nw3_arg[-n:] = ',around(self.w3_arg[-n:], decimals=0)
        print '\n\nw4_arg[-n:] = ',around(self.w4_arg[-n:], decimals=0)

        print '\n\nw1[-n:] = ',around(self.w1[-n:] , decimals=1)
        print '\n\nw2[-n:] = ',around(self.w2[-n:], decimals=1)
        print '\n\nw3[-n:] = ',around(self.w3[-n:] , decimals=1)
        print '\n\nw4[-n:] = ',around(self.w4[-n:], decimals=1)

        print '\n\nself.tao_qr_frame[-n:] = ',around(self.tao_qr_frame[-n:], decimals=5)

        print '\n\nself.T[-n:] = ',around(self.T[-n:], decimals=5)

        print '\n\nself.phi[-n:] = ',around(self.phi[-n:], decimals=5) 
        print '\n\nself.theta[-n:] = ',around(self.theta[-n:], decimals=5)
        print '\n\nself.psi[-n:] = ',around(self.psi[-n:], decimals=5)

        print '\n\nself.phidot[-n:] = ',around(self.phidot[-n:], decimals=5) 
        print '\n\nself.thetadot[-n:] = ',around(self.thetadot[-n:], decimals=5)
        print '\n\nself.psidot[-n:] = ',around(self.psidot[-n:], decimals=5)

        print '\n\nself.phiddot[-n:] = ',around(self.phiddot[-n:], decimals=5)
        print '\n\nself.thetaddot[-n:] = ',around(self.thetaddot[-n:], decimals=5)
        print '\n\nself.psiddot[-n:] = ',around(self.psiddot[-n:], decimals=5)

        print '\n\nself.x[-n:] = ',around(self.x[-n:], decimals=5)
        print '\n\nself.y[-n:] = ',around(self.y[-n:], decimals=5)
        print '\n\nself.z[-n:] = ',around(self.z[-n:], decimals=5)

        print '\n\nself.xdot[-n:] = ',around(self.xdot[-n:], decimals=5)
        print '\n\nself.ydot[-n:] = ',around(self.ydot[-n:], decimals=5)
        print '\n\nself.zdot[-n:] = ',around(self.zdot[-n:], decimals=5)

        print '\n\nself.xddot[-n:] = ',around(self.xddot[-n:], decimals=5)
        print '\n\nself.yddot[-n:] = ',around(self.yddot[-n:], decimals=5)
        print '\n\nself.zddot[-n:] = ',around(self.zddot[-n:], decimals=5)

        print '\n\nself.x_integral_error[-n:] = ',around(self.x_integral_error[-n:], decimals=5)
        print '\n\nself.y_integral_error[-n:] = ',around(self.y_integral_error[-n:], decimals=5)
        print '\n\nself.z_integral_error[-n:] = ',around(self.z_integral_error[-n:], decimals=5)

        print '\n\n'
#-------------------------------------------------------------------------------

if __name__ == "__main__":

    a = agent(x_0 = 0,    
              y_0 = 0,       
              z_0 = 0,     
              initial_setpoint_x = 1,     
              initial_setpoint_y = 1,     
              initial_setpoint_z = 1,     
              agent_priority = 1)



    #the following gains worked well for a setpoint of (1,1,1)

    a.kpx = 40     # ----------------------PID proportional gain values
    a.kpy = 40
    a.kpz = 40

    a.kdx = 20         #----------------- -----PID derivative gain values
    a.kdy = 20
    a.kdz = 10

    a.kix = .2
    a.kiy = .2
    a.kiz = 40

    a.kpphi   = 4  # gains for the angular pid control laws
    a.kptheta = 4
    a.kppsi   = 4

    a.kdphi   = 10
    a.kdtheta = 10
    a.kdpsi   = 5






    for i in range(a.max_iterations):


        a.ending_iteration = i  # preemptively...

        a.system_model_block()

        a.control_block()

        x_ave = sum(a.x[-100:])/100.0
        y_ave = sum(a.y[-100:])/100.0
        z_ave = sum(a.z[-100:])/100.0

        xerr = a.x_des - x_ave
        yerr = a.y_des - y_ave
        zerr = a.z_des - z_ave
     
        
        #if i%50 == 0:
        print 'x_ave = ',x_ave  
        print 'y_ave = ',y_ave
        print 'z_ave = ',z_ave
  
        print 'i = ',i
        print 'xerr, yerr, zerr = ',xerr,',',yerr,',',zerr
        print 'sqrt( xerr**2 + yerr**2 + zerr**2 ) = ',sqrt( xerr**2 + yerr**2 + zerr**2 )
        #a.print_dump(3)        
        
 
        

        # Stopping Criteria: if the agent is within a 5 cm error sphere for 200 time steps ( .2 sec )  
        
        if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) < 10**-2) and (i>50):

            print 'set point reached!!'

            print 'i = ', i

            break
        
        if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) > 200) and (i>50):

            print 'you are lost!!'

            print 'i = ', i

            break

        k_th_variable_list = [
                                a.xacc_comm[-1],
                                a.yacc_comm[-1],
                                a.zacc_comm[-1],
                                a.theta_comm[-1],
                                a.phi_comm[-1],
                                a.T_comm[-1],
                                a.tao_phi_comm[-1],
                                a.tao_theta_comm[-1],
                                a.tao_psi_comm[-1],
                                a.w1_arg[-1],
                                a.w2_arg[-1],
                                a.w3_arg[-1],
                                a.w4_arg[-1],
                                a.w1[-1],
                                a.w2[-1],
                                a.w3[-1],
                                a.w4[-1],
                                a.tao_qr_frame[-1][0],
                                a.tao_qr_frame[-1][1],
                                a.tao_qr_frame[-1][2],
                                a.T[-1],
                                a.phi[-1],
                                a.theta[-1],
                                a.psi[-1],
                                a.phidot[-1],
                                a.thetadot[-1],
                                a.psidot[-1],
                                a.phiddot[-1],
                                a.thetaddot[-1],
                                a.psiddot[-1],
                                a.x[-1],
                                a.y[-1],
                                a.z[-1],
                                a.xdot[-1],
                                a.ydot[-1],
                                a.zdot[-1],
                                a.xddot[-1],
                                a.yddot[-1],
                                a.zddot[-1],
                                a.x_integral_error[-1],
                                a.y_integral_error[-1],
                                a.z_integral_error[-1],
                                ]  

        for k in k_th_variable_list: 

            if math.isnan(k):

                a.print_dump(1)     
        
                break


        if a.phi[-1] > 5: break
        if a.theta[-1] > 5: break
        if a.psi[-1] > 5: break

        if math.isnan(xerr):

            break

    print '##############################################\n\n'

    a.print_dump(10)

    fig1_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig1_agent_module.png'
    fig2_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig2_agent_module.png'

    a.plot_results()#False, True, fig1_file_path, fig2_file_path)






