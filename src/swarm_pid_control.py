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

from functions_module import *
import numpy as n
from wind import *
import sys
######################################################################################################

class agent:

    def __init__(self, x_0,   y_0,   z_0,
                       initial_setpoint_x,  initial_setpoint_y, initial_setpoint_z, 
                       agent_priority = 10 ):

    
        # a list of distances to each other agent sorted nearest to farthest
        self.distance_vectors = []
        
        self.agent_priority = agent_priority

        #--------------------------------state vector and  derivative time series'
        self.x = [x_0]
        self.y = [y_0]
        self.z = [z_0]

        self.xdot = [0]
        self.ydot = [0]
        self.zdot = [0]

        self.xddot = [0]
        self.yddot = [0]
        self.zddot = [0]

        self.phi = [0]
        self.theta = [0]
        self.psi = [0]

        self.phidot = [0]
        self.thetadot = [0]
        self.psidot = [0] 

        self.phiddot = [0]
        self.thetaddot = [0]
        self.psiddot = [0]



        self.kpx = 10          # -------------------------------PID proportional gain values
        self.kpy = 10
        self.kpz = 8

        self.kdx = 5           #--------------------------------PID derivative gain values
        self.kdy = 5
        self.kdz = 5

        self.kix = 1
        self.kiy = 1
        self.kiz = 1

        self.xError = 0
        self.yError = 0
        self.zError = 0
        #--------------------------------- force, torque, and motor speed list initializations

        self.T = []
        self.tao_phi = []
        self.tao_theta = []
        self.tao_psi = []

        self.w1 = []
        self.w2 = []
        self.w3 = []
        self.w4 = []

        self.etaddot = []
        #----------------------------------

        self.max_iterations = 50000
        self.h = 0.001

        #-----------------------------------------------------------------wind!!
        
        self.wind_duration = self.max_iterations
        self.max_gust = 0  #  m/s !!  = 111 mph

        # generate the wind for the entire simulation beforehand
        self.wind_data = wind_vector_time_series( self.max_gust , self.wind_duration )

        self.wind_x = self.wind_data[0] 
        self.wind_y = self.wind_data[1]
        self.wind_z = self.wind_data[2] 
        #----------------

        self.max_total_thrust = 50.0  # [newtons]
        self.min_total_thrust = 1.0

        self.x_des = initial_setpoint_x
        self.y_des = initial_setpoint_y
        self.z_des = initial_setpoint_z

        self.initial_setpoint_x = initial_setpoint_x 
        self.initial_setpoint_y = initial_setpoint_y
        self.initial_setpoint_z = initial_setpoint_z

    def iteration(self,i):

        system_iteration(
	             self.x_des,
	             self.y_des,
	             self.z_des,
	             self.h,
	             self.x,self.y,self.z,self.xdot,self.ydot,self.zdot,self.xddot,self.yddot,self.zddot,
	             self.phi,self.theta,self.psi,self.phidot,self.thetadot,self.psidot,self.phiddot,self.thetaddot,self.psiddot,
	             self.w1,self.w2,self.w3,self.w4,
	             self.tao_phi,self.tao_theta,self.tao_psi, 
	             self.T,
	             self.max_total_thrust,
	             self.min_total_thrust,
	             self.wind_x[i],
	             self.wind_y[i],
	             self.wind_z[i]
	             )




    #----------------------------------------------------
        
    def iteration_gain_input(self,i):

        system_iteration(
	             self.x_des,
	             self.y_des,
	             self.z_des,
	             self.h,
	             self.x,self.y,self.z,self.xdot,self.ydot,self.zdot,self.xddot,self.yddot,self.zddot,
	             self.phi,self.theta,self.psi,self.phidot,self.thetadot,self.psidot,self.phiddot,self.thetaddot,self.psiddot,
	             self.w1,self.w2,self.w3,self.w4,
	             self.tao_phi,self.tao_theta,self.tao_psi, 
	             self.T,
	             self.max_total_thrust,
	             self.min_total_thrust,
	             self.wind_x[i],
	             self.wind_y[i],
	             self.wind_z[i],
                 self.kpx,    # -------------------------------PID proportional gain values
                 self.kpy,
                 self.kpz,
                 self.kdx,    #--------------------------------PID derivative gain values
                 self.kdy,
                 self.kdz,
                 self.kix,
                 self.kiy,
                 self.kiz,    
                 self.xError,
                 self.yError,
                 self.zError         
	             )


    # ==========================================================================end class def


def avoid_collisions(agents):
    '''
    take position of each agent

    form a list of lists of inter-agent distances for each agent, sorted nearest to farthest
    
    distance *  agent priority = the coefficient for the imaginary force that each agent 'feels' from each other agent
    '''
    positions = [ n.array( [a.x[-1], a.y[-1], a.z[-1]] ) for a in agents ]

    #print '\n\n----------------------------------------------------------------------------'
    
    #print '\npositions = ',positions

    for p in positions:

        if n.isnan(p.any()):

            print 'INVALID POSITION'

            sys.exit()


    for a in agents:

        myPosition = n.array( [a.x[-1], a.y[-1], a.z[-1]] )
        #print '\n\nmyPosition = ',myPosition

        a.distance_vectors = n.array([ p - myPosition for p in positions])
        #print '\na.distance_vectors = ',a.distance_vectors    

        #---------------------------------------------
        filtered_distance_vectors = []

        for d in a.distance_vectors:
            
            if 0.5 < n.linalg.norm(d) < 6 :   # only care if two agents are less than 4 m apart 

                #print 'n.linalg.norm(d) = ',n.linalg.norm(d)

                filtered_distance_vectors.append( d )

            elif  (all(d) == 0.0 ):         # in this case d is the distance from the ith agent to itself , which is not interesting

                filtered_distance_vectors.append(None)
    
            elif (n.linalg.norm(d) <= .5) :
            
                    print '\n\n(n.linalg.norm(d) < .5) = ',(n.linalg.norm(d) < .5)
                    print 'CRASH!CRASH!CRASH!CRASH!CRASH!CRASH!\n\n'                                
                    sys.exit()

            elif (n.linalg.norm(d) >= 6 ) :     # dont care

                filtered_distance_vectors.append(None)


            elif n.isnan(any(d)):

                print 'd in a.distance_vectors  = ',d
                sys.exit()


            else:
        
                print '\n\n EXCEPTION IN CALCULATING DISTANCE VECTOR '

                print 'd in a.distance_vectors  = ',d
                sys.exit()
            #-------------------------------------------------------------------

        

        a.distance_vectors = filtered_distance_vectors           # the filtered distance vectors only includes the 
                                                                 # distances to other agents that are  bewteen 0.5 m and 4 m 
 
        #print '\na.distance_vectors = ',a.distance_vectors

        a.direction_coeffs = []

        for m in range(len(a.distance_vectors)):

            coef_vect = []

            if a.distance_vectors[m] != None:

                for d in a.distance_vectors[m]:             # testing each component of the vector d

                    if (d >= 0.1) or (d <= -0.1):

                        coef_vect.append( - ( 2 / d ) * agents[m].agent_priority )

                    elif 0.1 > d > -0.1 :

                        coef_vect.append(0)
                        
                    else: 
                        print 'ERROR COMPUTING DIRECTION COEFF'  
                        sys.exit()                        
                           




            #print 'coef_vect = ',coef_vect


            if len(coef_vect) != 0:

                a.direction_coeffs.append(coef_vect)                



        #if len(a.direction_coeffs) > 0 : print '\na.direction_coeffs = ',a.direction_coeffs





        for dc in a.direction_coeffs:

            if any(n.isnan(dc)):

                print 'INVALID DIRECTION COEFFICIENT'
                sys.exit()    






        if (len( a.direction_coeffs ) > 0):

            #print 'n.linalg.norm( a.direction_coeffs ) > 0 = ',n.linalg.norm( a.direction_coeffs )

            temp_x = a.x[-1]
            temp_y = a.y[-1]
            temp_z = a.z[-1]

            for coef in a.direction_coeffs:

                temp_x += coef[0] 
                temp_y += coef[1]
                temp_z += coef[2]

            
            a.x_des = temp_x
            #print 'a.x_des = ',a.x_des
            
            a.y_des = temp_y 
            #print 'a.y_des = ',a.y_des

            a.z_des = temp_z
            #print 'a.z_des = ',a.z_des
                      
    
        else:

            a.x_des = a.initial_setpoint_x
            a.y_des = a.initial_setpoint_y 
            a.z_des = a.initial_setpoint_z   
        


#-------------------------------------------------------------------  
      
    

            
            


###############################################################################################################
if __name__ == '__main__':

    number_of_agents = 4

    agents = [ agent(x_0 = 0,    y_0 = 0,   z_0 = 0, initial_setpoint_x = 5, initial_setpoint_y = 10, initial_setpoint_z = 15, agent_priority = 1),
               agent(x_0 = 0,    y_0 = 5,   z_0 = 0, initial_setpoint_x = 20, initial_setpoint_y = 70, initial_setpoint_z = 20, agent_priority = 10), 
               agent(x_0 = 0,    y_0 = 10,   z_0 = 0, initial_setpoint_x = 30, initial_setpoint_y = 60, initial_setpoint_z = 20, agent_priority = 20),
               agent(x_0 = 0,    y_0 = 15,   z_0 = 0, initial_setpoint_x = 40, initial_setpoint_y = 50,  initial_setpoint_z = 20, agent_priority = 30),]
    '''
               agent(x_0 = 0,    y_0 = 20,   z_0 = 0, initial_setpoint_x = 50, initial_setpoint_y = 40, initial_setpoint_z = 20, agent_priority = 40),
               agent(x_0 = 0,    y_0 = 25,   z_0 = 0, initial_setpoint_x = 60, initial_setpoint_y = 30, initial_setpoint_z = 20, agent_priority = 50), 
               agent(x_0 = 0,    y_0 = 30,   z_0 = 0, initial_setpoint_x = 70, initial_setpoint_y = 20, initial_setpoint_z = 20, agent_priority = 60),
               agent(x_0 = 0,    y_0 = 35,   z_0 = 0, initial_setpoint_x = 80, initial_setpoint_y = 10,  initial_setpoint_z = 20, agent_priority = 70)
            ]
    '''
    for i in range(agents[0].max_iterations):
        #inp = raw_input('press enter to continue...' )
        if i%1000 == 0: print 'i = ',i
        #print 'i = ',i
        for a in agents:

            a.iteration(i)
        

        avoid_collisions(agents)
        
    for a in agents:
        print '-----------------------------------'
        print 'a.x[-1] = ',a.x[-1]
        print 'a.y[-1] = ',a.y[-1]
        print 'a.z[-1] = ',a.z[-1]
        print 'a.x_des = ',a.x_des
        print 'a.y_des = ',a.y_des
        print 'a.z_des = ',a.z_des
    timeSeries = [agents[0].h*i for i in range(len(agents[0].x))]#

    swarm_plot(timeSeries, agents) 
    




