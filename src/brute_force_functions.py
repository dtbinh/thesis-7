
from agent_module import *

from numpy import mean

import json
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

def run(agent,plots = False):

    for i in range(agent.max_iterations):
      
        agent.ending_iteration = i

        agent.system_model_block()

        agent.control_block()


        x_ave = sum(agent.x[-100:])/100.0
        y_ave = sum(agent.y[-100:])/100.0
        z_ave = sum(agent.z[-100:])/100.0

        xerr = agent.x_des - x_ave
        yerr = agent.y_des - y_ave
        zerr = agent.z_des - z_ave

        # Stopping Criteria: if the agent is within a n cm error sphere for 200 time steps ( .2 sec )  
        
        if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) <10**-2) and (i>50):

            print 'i = ', i

            print 'set point reached'
        
            return 1

        if ( abs(zerr) > 200) and (i>50):

            print 'i = ', i

            print 'you are lost'

            return 'err'

        k_th_variable_list = [  agent.xacc_comm[-1],agent.yacc_comm[-1],agent.zacc_comm[-1],
                                agent.theta_comm[-1],agent.phi_comm[-1],agent.T_comm[-1],
                                agent.tao_phi_comm[-1],agent.tao_theta_comm[-1],agent.tao_psi_comm[-1],
                                agent.w1_arg[-1],agent.w2_arg[-1],agent.w3_arg[-1],agent.w4_arg[-1],
                                agent.w1[-1],agent.w2[-1],agent.w3[-1],agent.w4[-1],
                                agent.tao_qr_frame[-1][0],agent.tao_qr_frame[-1][1],agent.tao_qr_frame[-1][2],
                                agent.T[-1],
                                agent.phi[-1],agent.theta[-1],agent.psi[-1],
                                agent.phidot[-1],agent.thetadot[-1],agent.psidot[-1],
                                agent.phiddot[-1],agent.thetaddot[-1],agent.psiddot[-1],
                                agent.x[-1],agent.y[-1],agent.z[-1],
                                agent.xdot[-1],agent.ydot[-1],agent.zdot[-1],
                                agent.xddot[-1],agent.yddot[-1],agent.zddot[-1],
                                agent.x_integral_error[-1],agent.y_integral_error[-1],agent.z_integral_error[-1],
                                ]  

        for k in k_th_variable_list: 

            if math.isnan(k):

                agent.print_dump(1)     
        
                return 'err'

            if agent.phi[-1] > 5: break
            if agent.theta[-1] > 5: break
            if agent.psi[-1] > 5: break

            if math.isnan(xerr):
                print 'math.isnan(xerr) = True'

                return 'err'

#-------------------------------------------------------------------------------    
#-------------------------------------------------------------------------------

def take_off():

    agent0 = agent(x_0 = 0,    
              y_0 = 0,       
              z_0 = 0,     
              initial_setpoint_x = 0,     
              initial_setpoint_y = 0,     
              initial_setpoint_z = 1,     
              agent_priority = 1)

    agent0.max_iterations = 800

    #the following gains worked well for a setpoint of (1,1,1)

    agent0.kpx = 40     # ----------------------PID proportional gain values
    agent0.kpy = 40
    agent0.kpz = 40

    agent0.kdx = 25         #----------------- -----PID derivative gain values
    agent0.kdy = 25
    agent0.kdz = 40

    agent0.kix = .2
    agent0.kiy = .2
    agent0.kiz = 40

    run(agent0)
    
    return agent0   # return the agent instance hovering at (0,0,1)

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def test_gain_vector(a0, set_point, gain_dictionary):

    a0.x_des = set_point[0]     
    a0.y_des = set_point[1]     
    a0.z_des = set_point[2]     

    a0.max_iterations = 1000

    a0.kpx = gain_dictionary['kpxy']
    a0.kix = gain_dictionary['kixy']
    a0.kdx = gain_dictionary['kdxy']

    a0.kpy = gain_dictionary['kpxy']
    a0.kiy = gain_dictionary['kixy']
    a0.kdy = gain_dictionary['kdxy']

    a0.kpz = gain_dictionary['kpz']
    a0.kiz = gain_dictionary['kiz']
    a0.kdz = gain_dictionary['kdz']
 
    return_val2 = run(a0)
    '''
    #-----------------------------------------------------
    variable_dictionary = {
                'a0.xacc_comm' : a0.xacc_comm,'a0.yacc_comm' : a0.yacc_comm,'a0.zacc_comm' : a0.zacc_comm,
                'a0.theta_comm' : a0.theta_comm,'a0.phi_comm' : a0.phi_comm,'a0.T_comm' : a0.T_comm,
                'a0.tao_phi_comm' : a0.tao_phi_comm,'a0.tao_theta_comm' : a0.tao_theta_comm,'a0.tao_psi_comm' : a0.tao_psi_comm,
                'a0.w1_arg' : a0.w1_arg,'a0.w2_arg' : a0.w2_arg,'a0.w3_arg' : a0.w3_arg,'a0.w4_arg' : a0.w4_arg,
                'a0.w1' : a0.w1,'a0.w2' : a0.w2,'a0.w3' : a0.w3,'a0.w4' : a0.w4,
                'a0.tao_qr_frame[0]' : a0.tao_qr_frame[0].tolist(),'a0.tao_qr_frame[1]' : a0.tao_qr_frame[1].tolist(),'a0.tao_qr_frame[2]' : a0.tao_qr_frame[2].tolist(),
                'a0.T' : a0.T,
                'a0.phi' : a0.phi,'a0.theta' : a0.theta,'a0.psi' : a0.psi,
                'a0.phidot' : a0.phidot,'a0.thetadot' : a0.thetadot,'a0.psidot' : a0.psidot,
                'a0.phiddot' : a0.phiddot,'a0.thetaddot' : a0.thetaddot,'a0.psiddot' : a0.psiddot,
                'a0.x' : a0.x,'a0.y' : a0.y,'a0.z' : a0.z,
                'a0.xdot' : a0.xdot,'a0.ydot' : a0.ydot,'a0.zdot' : a0.zdot,
                'a0.xddot' : a0.xddot,'a0.yddot' : a0.yddot,'a0.zddot' : a0.zddot,
                'a0.x_integral_error' : a0.x_integral_error,
                'a0.y_integral_error' : a0.y_integral_error,
                'a0.z_integral_error' : a0.z_integral_error,
                }
    #-----------------------------------------------------
    '''
    if return_val2 != 1:


        test_run_dictionary = {'setpoint':set_point,
                                'a0.kpx' : a0.kpx,
                                'a0.kix' : a0.kix,
                                'a0.kdx' : a0.kdx,
                                'a0.kpy' : a0.kpy,
                                'a0.kiy' : a0.kiy,
                                'a0.kdy' : a0.kdy,
                                'a0.kpz' : a0.kpz,
                                'a0.kiz' : a0.kiz,
                                'a0.kdz' : a0.kdz,
                                'return_val2' : return_val2,
                                'a0.ending_iteration' : a0.ending_iteration
                                }#'variable_dictionary':variable_dictionary
                                #} 
            
        return test_run_dictionary        



    elif return_val2 ==1:


        #need to calculate the number of times the state crosses the setpoint value:

        x_crossings = 0
        y_crossings = 0
        z_crossings = 0

        for i in range(len(a0.x)-1):

            if sign( a0.x[i] - a0.x_des ) != sign( a0.x[i+1] - a0.x_des ) :

                x_crossings += 1


            if sign( a0.y[i] - a0.y_des ) != sign( a0.y[i+1] - a0.y_des ) :                 

                y_crossings += 1


            if sign( a0.z[i] - a0.z_des ) != sign( a0.z[i+1] - a0.z_des ) :  

                z_crossings += 1




        #-----------------------------------------------------
        

        if (max(a0.x) - a0.x_des) > 0: x_over_shoot = max(a0.x) - a0.x_des

        if (max(a0.y) - a0.y_des) > 0: y_over_shoot = max(a0.y) - a0.y_des

        if (max(a0.z) - a0.z_des) > 0: z_over_shoot = max(a0.z) - a0.z_des


       #-----------------------------------------------------



        test_run_dictionary = {'setpoint':set_point,
                                'a0.kpx' : a0.kpx,
                                'a0.kix' : a0.kix,
                                'a0.kdx' : a0.kdx,
                                'a0.kpy' : a0.kpy,
                                'a0.kiy' : a0.kiy,
                                'a0.kdy' : a0.kdy,
                                'a0.kpz' : a0.kpz,
                                'a0.kiz' : a0.kiz,
                                'a0.kdz' : a0.kdz,
                                'return_val2' : return_val2,
                                'a0.ending_iteration' : a0.ending_iteration,
                                'total_thrust' : sum(a0.T),
                                'x_over_shoot':x_over_shoot,
                                'y_over_shoot':y_over_shoot,                               
                                'z_over_shoot':z_over_shoot,
                                'x_crossings':x_crossings,
                                'y_crossings':y_crossings,
                                'z_crossings':z_crossings
                                } 
            
        return test_run_dictionary

#-------------------------------------------------------------------------------
'''
{   u'a0.ending_iteration': 266,
    u'a0.kdx': 5,
    u'a0.kdy': 5,
    u'a0.kdz': 20,
    u'a0.kix': 0.8,
    u'a0.kiy': 0.8,
    u'a0.kiz': 15,
    u'a0.kpx': 5,
    u'a0.kpy': 5,
    u'a0.kpz': 30,
    u'ith_runtime': 7.14291787147522,
    u'return_val2': 1,
    u'setpoint': [1, 1, 2],
    u'total_thrust': 4973.812742102159,
    u'x_crossings': 1,
    u'x_over_shoot': 0.06537601013649552,
    u'y_crossings': 1,
    u'y_over_shoot': 0.0706587304875288,
    u'z_crossings': 1,
    u'z_over_shoot': 0.03570385076740079}

'''
#-------------------------------------------------------------------------------

if __name__ == '__main__':

    gain_dictionary = {
                    'kpxy' : 5,     
                    'kpz'  : 30,
                    'kdxy' : 5,
                    'kdz'  : 20,
                    'kixy' : 0.8,
                    'kiz'  : 15}

    agent = take_off() # ----> returns the agent instance hovering at (0,0,1)      
    
    set_point = [1,1,2]

    test_run_dictionary = test_gain_vector(agent, set_point, gain_dictionary)

    print 'test_run_dictionary = ',test_run_dictionary

    agent.plot_results()

