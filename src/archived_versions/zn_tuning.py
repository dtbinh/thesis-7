

from agent_module import *
from numpy import mean

import json


def calc_tu(agent, tuning_variable_list):  # note tuning_variable_list must be in the form 'agent.z'
 
    snake_length = 11

    maxima_list = []

    for i in range(len(tuning_variable_list)-snake_length):

        snake = tuning_variable_list[i:i+snake_length]

        #print snake   

        #print snake[5] 

        if snake[5] == max(snake):

            maxima_list.append( [ (i+5)*agent.h ,snake[5] ] )

    #print 'maxima_list = ',maxima_list

    tu =  mean([abs(maxima_list[i][0] - maxima_list[i+1][0])   for i in range( len(maxima_list)-1 ) ] )

    return tu

    #maxima_height_differences = [abs(maxima_list[i][1] - maxima_list[i+1][1])   for i in range( len(maxima_list)-1 ) ]

    #print 'maxima_height_differences = ',maxima_height_differences

#---------------------------------------------------------------------------------

def tune(agent,set_point,kux,kuy,kuz):

    print 'tuning the controllers for the setpoint = ', set_point


    agent.x_des = set_point[0]
    agent.y_des = set_point[1]
    agent.z_des = set_point[2]

    agent.max_iterations = 1000

    agent.kpx = kux
    agent.kpy = kuy
    agent.kpz = kuz

    agent.kix = 0
    agent.kiy = 0
    agent.kiz = 0

    agent.kdx = 0
    agent.kdy = 0
    agent.kdz = 0

    print 'agent.kpx = ',agent.kpx
    print 'agent.kpy = ',agent.kpy
    print 'agent.kpz = ',agent.kpz

    # first run the sim with only the ku to measure tu then calculate and run with all the gains in place

    return_val = run(agent)



    tux = calc_tu(agent,agent.x)
    tuy = calc_tu(agent,agent.y)                       
    tuz = calc_tu(agent,agent.z)
          

    print 'tux = ',tux
    print 'tuy = ',tuy
    print 'tuz = ',tuz


    # calc the Zeigler - Nichols gains for each controller

    kpx = kux
    kix = 0.2 #2.0 * kpx / float(tux)  
    kdx = kpx * tux * 100


    kpy = kuy
    kiy = 0.2 #2.0 * kpy / float(tuy)
    kdy = kpy * tuy *100


    kpz = 0.8 * kuz
    kiz = 2.0 * kpz / float(tuz) 
    kdz = 0.8 * kuz




    tuned_gain_dictionary = {'return_val':return_val,
                                'setpoint':set_point,
                                'kux': kux,
                                'kux': kuy,
                                'kux': kuz,
                                'tux': tux,
                                'tuy': tuy,
                                'tuz': tuz,
                                'kpx': kpx,
                                'kix': kix,
                                'kdx': kdx,
                                'kpy': kpy,
                                'kiy': kiy,
                                'kdy': kdy,
                                'kpz': kpz,
                                'kiz': kiz,
                                'kdz': kdz, 
                                'err_msg': None
                                }

    print 'tuned_gain_dictionary = ',tuned_gain_dictionary

    return tuned_gain_dictionary

    #-----------------------------------------------------------------------









def test_gain_vector(a0, set_point, tuned_gain_dictionary):

    a0.x_des = set_point[0]     
    a0.y_des = set_point[1]     
    a0.z_des = set_point[2]     

    a0.max_iterations = 1000

    a0.kpx = tuned_gain_dictionary['kpx']
    a0.kix = tuned_gain_dictionary['kix']
    a0.kdx = tuned_gain_dictionary['kdx']

    a0.kpy = tuned_gain_dictionary['kpy']
    a0.kiy = tuned_gain_dictionary['kiy']
    a0.kdy = tuned_gain_dictionary['kdy']

    a0.kpz = tuned_gain_dictionary['kpz']
    a0.kiz = tuned_gain_dictionary['kiz']
    a0.kdz = tuned_gain_dictionary['kdz']
 
    return_val2 = run(a0)

    #-----------------------------------------------------

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
                                } 
            
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
                                'z_crossings':z_crossings,
                                } 
            
        return test_run_dictionary

#---------------------------------------------------------------------------------
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


        '''
        print 'x_ave = ',x_ave  
        print 'y_ave = ',y_ave
        print 'z_ave = ',z_ave
  
        print 'i = ',i
        print 'xerr, yerr, zerr = ',xerr,',',yerr,',',zerr
        print 'sqrt( xerr**2 + yerr**2 + zerr**2 ) = ',sqrt( xerr**2 + yerr**2 + zerr**2 )
        '''

        # Stopping Criteria: if the agent is within a n cm error sphere for 200 time steps ( .2 sec )  
        
        if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) <10**-3) and (i>50):

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
    '''
    if plots == True:

        #fig1_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig1_zn_module.png'

        #fig2_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig2_zn_module.png'

        agent.plot_results()#False, True, fig1_file_path, fig2_file_path)

        agent.print_dump(10)
    '''

#-------------------------------------------------------------------------------

def get_good_runs(tuning_dictionaries):

    good_runs = []

    for i in tuning_dictionaries:

        if ('nan' not in i.values() ) and ('err' not in i.values() ) and (None not in i.values() ):

            good_runs.append(i)


    min_total_thrust = 1000000  

    optimal_run = None

    for r in good_runs:

        if r['total_thrust'] < min_total_thrust:

            optimal_run = r

            min_total_thrust = r['total_thrust']
 
    print '\n\noptimal run:',optimal_run

    return [good_runs , optimal_run]



#-------------------------------------------------------------------------------    


def take_off():

    agent0 = agent(x_0 = 0,    
              y_0 = 0,       
              z_0 = 0,     
              initial_setpoint_x = 0,     
              initial_setpoint_y = 0,     
              initial_setpoint_z = 1,     
              agent_priority = 1)

    #the following gains worked well for a setpoint of (1,1,1)

    agent0.kpx = 40     # ----------------------PID proportional gain values
    agent0.kpy = 40
    agent0.kpz = 40

    agent0.kdx = 10         #----------------- -----PID derivative gain values
    agent0.kdy = 10
    agent0.kdz = 40

    agent0.kix = .2
    agent0.kiy = .2
    agent0.kiz = 40

    run(agent0)
    
    return agent0   # return the agent instance hovering at (0,0,1)


#-------------------------------------------------------------------------------

if __name__ == '__main__':
    '''
    the starting point 000 requires a slightly different set of gains   
    '''
    

    agent = take_off() # ----> returns the agent instance hovering at (0,0,1)      
    
    set_point = [1,1,2]

    kux = 10
    kuy = 10
    kuz = 40

    tuned_gain_dictionary = tune(agent,set_point,kux,kuy,kuz)

    test_run_dictionary = test_gain_vector(agent, set_point, tuned_gain_dictionary)

    print 'test_run_dictionary = ',test_run_dictionary
    
    agent.plot_results()

    '''
    a.x_des = 10
    a.y_des = 10 
    a.z_des = 20

    a.kpx = 10     # ----------------------PID proportional gain values
    a.kpy = 10
    a.kpz = 40

    a.kdx = 10         #----------------- -----PID derivative gain values
    a.kdy = 10
    a.kdz = 40

    a.kix = .5
    a.kiy = .5
    a.kiz = 15
    '''


    #-----------------------------------------------------------------------



