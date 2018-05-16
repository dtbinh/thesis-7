
from agent_module import *

def go(agent): 

    for i in range(agent.max_iterations):

        a.system_model_block()

        a.control_block()

        retval = stopping_criteria(agent)

        if (retval == 0) or (retval == 1):
                
            print 'i = ', i
        
            break
                
#---------------------------------------------------------------------------------

def stopping_criteria(agent):

    x_ave = sum(agent.x[-100:])/100.0
    y_ave = sum(agent.y[-100:])/100.0
    z_ave = sum(agent.z[-100:])/100.0

    xerr = agent.x_des - x_ave
    yerr = agent.y_des - y_ave
    zerr = agent.z_des - z_ave
    
    #if i%50 == 0:
        #print 'x_ave = ',x_ave  
        #print 'y_ave = ',y_ave
        #print 'z_ave = ',z_ave

    print 'xerr, yerr, zerr = ',xerr,',',yerr,',',zerr
    print 'sqrt( xerr**2 + yerr**2 + zerr**2 ) = ',sqrt( xerr**2 + yerr**2 + zerr**2 )

    if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) < 10**-2) and (len(agent.x) >50):

        print 'set point reached!!'

        #print 'i = ', i

        return 1

    if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) > 200) and (i>50):

        print 'you are lost!!'


        return 0

    k_th_variable_list = [
                            a.xacc_comm[-1],a.yacc_comm[-1],a.zacc_comm[-1],
                            a.theta_comm[-1],a.phi_comm[-1],a.T_comm[-1],
                            a.tao_phi_comm[-1],a.tao_theta_comm[-1],a.tao_psi_comm[-1],
                            a.w1_arg[-1],a.w2_arg[-1],a.w3_arg[-1],a.w4_arg[-1],
                            a.w1[-1],a.w2[-1],a.w3[-1],a.w4[-1],
                            a.tao_qr_frame[-1][0],a.tao_qr_frame[-1][1],a.tao_qr_frame[-1][2],
                            a.T[-1],
                            a.phi[-1],a.theta[-1],a.psi[-1],
                            a.phidot[-1],a.thetadot[-1],a.psidot[-1],
                            a.phiddot[-1],a.thetaddot[-1],a.psiddot[-1],
                            a.x[-1],a.y[-1],a.z[-1],
                            a.xdot[-1],a.ydot[-1],a.zdot[-1],
                            a.xddot[-1],a.yddot[-1],a.zddot[-1],
                            a.x_integral_error[-1],a.y_integral_error[-1],a.z_integral_error[-1],
                            ]  

    for k in k_th_variable_list: 

        if math.isnan(k):

            a.print_dump(1)     
    
            return 0


    if a.phi[-1] > 5: return 0

    if a.theta[-1] > 5: return 0

    if a.psi[-1] > 5: return 0

    if math.isnan(xerr): return 0

###############################################################################

if __name__ == '__main__':

    a = agent(x_0 = 0,    
              y_0 = 0,       
              z_0 = 0,     
              initial_setpoint_x = 1,     
              initial_setpoint_y = 1,     
              initial_setpoint_z = 1,     
              agent_priority = 1)

    #the following gains worked well for a setpoint of (1,1,1)

    a.kpx = 15     # ----------------------PID proportional gain values
    a.kpy = 15
    a.kpz = 50

    a.kdx = 10         #----------------- -----PID derivative gain values
    a.kdy = 10
    a.kdz = 50

    a.kix = 0.8
    a.kiy = 0.8
    a.kiz = 15

    a.kpphi   = 4  # gains for the angular pid control laws
    a.kptheta = 4
    a.kppsi   = 4

    a.kdphi   = 6
    a.kdtheta = 6
    a.kdpsi   = 6
    '''
    u'a0.ending_iteration': 257,                          the  optimal run
    u'a0.kdx': 10,
    u'a0.kdy': 10,
    u'a0.kdz': 50,
    u'a0.kix': 0.8,
    u'a0.kiy': 0.8,
    u'a0.kiz': 20,
    u'a0.kpx': 15,
    u'a0.kpy': 15,
    u'a0.kpz': 40,
    u'ith_runtime': 11.414505958557129,
    u'return_val2': 1,
    u'setpoint': [1, 1, 2],
    u'total_thrust': 4969.888344602483,
    u'x_crossings': 3,
    u'x_over_shoot': 0.024969492375947366,
    u'y_crossings': 1,
    u'y_over_shoot': 0.01858534082026475,
    u'z_crossings': 1,
    u'z_over_shoot': 0.09928387955818296}




    '''

    a.max_iterations = 1000

    position_setpoint_list =  [[0,0,1],[1,1,2]]  #[[1,1,1],[5,5,5],[20,20,20],[100,100,100]] #[[1,1,1],[5,1,1],[5,5,1],[5,5,5],[1,5,5],[1,1,5],[1,1,1]] #

    for ss in range(len(position_setpoint_list)):

        a.x_des = position_setpoint_list[ss][0]
        a.y_des = position_setpoint_list[ss][1]
        a.z_des = position_setpoint_list[ss][2]

        go(a)


    print '##############################################\n\n'

    a.print_dump(10)

    fig1_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig1_agent_module.png'
    fig2_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig2_agent_module.png'

    a.plot_results()


