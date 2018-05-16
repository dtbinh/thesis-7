from agent_module import *
from numpy.linalg import norm
import sys
import pprint
pp = pprint.PrettyPrinter(indent=4)



from datetime import datetime
        
#from PyQt4 import QtGui, QtCore# Python Qt4 bindings for GUI objects
#from matplotlib.figure import Figure# Matplotlib Figure object

# import the Qt4Agg FigureCanvas object, that binds Figure to
# Qt4Agg backend. It also inherits from QWidget
#from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas


from mpl_toolkits.mplot3d import Axes3D
#import matplotlib.pyplot as plt
#from pylab import title   
import matplotlib.gridspec as gridspec

from os import system

from numpy import around


class gain_optimization():

    def __init__(self):


        date_and_time = datetime.now().strftime('%Y-%m-%d__%H.%M.%S')

        # the file path for this run :

        self.plot_dir = '/home/ek/Dropbox/THESIS/python_scripts/run_' + date_and_time

        system('mkdir ' + self.plot_dir)



        self.d = 0.01 # the increment for the finite differences in the gradient

        self.grad_norm_list = []

        self.optimization_step_size = 1.0

        self.normalized_gradient_list = []



        self.kpx_list = [30,30]          # -----------PID proportional gain values
        self.kpy_list = [30,30]
        self.kpz_list = [30,30]

        self.kdx_list = [20,20]          #------------PID derivative gain values
        self.kdy_list = [20,20]
        self.kdz_list = [10,20]

        self.kix_list = [1,1]          #------------PID integral gain values
        self.kiy_list = [1,1]
        self.kiz_list = [40,40]


        self.kpphi   = 4          # gains for the angular pid control laws
        self.kptheta = 4
        self.kppsi   = 4

        self.kdphi   = 10
        self.kdtheta = 10
        self.kdpsi   = 5

        
        self.flight_record = [self.test_flight(None)]
        
        self.xacc_comm  = []
        self.yacc_comm  = []
        self.zacc_comm  = []
        self.theta_comm  = []
        self.phi_comm  = []
        self.T_comm  = []
        self.tao_phi_comm  = []
        self.tao_theta_comm  = []
        self.tao_psi_comm  = []
        self.w1_arg  = []
        self.w2_arg  = []
        self.w3_arg  = []
        self.w4_arg  = []
        self.w1  = []
        self.w2  = []
        self.w3  = []
        self.w4  = []
        self.tao_phi  = []
        self.tao_theta  = []
        self.tao_psi  = []
        self.T  = []
        self.phi  = []
        self.theta  = []
        self.psi  = []
        self.phidot  = []
        self.thetadot  = []
        self.psidot = []
        self.phiddot = []
        self.thetaddot = []
        self.psiddot = []
        self.x = []
        self.y = []
        self.z = []
        self.xdot = []
        self.ydot = []
        self.zdot = []
        self.xddot = []
        self.yddot = []
        self.zddot = []
        self.x_integral_error = []
        self.y_integral_error = []
        self.z_integral_error = []
                           



    #---------------------------------------------------------------------------

    # the argument delta gives an offset to the gain values for use
    # in computing the gradient

    def test_flight(self,WRT):  

        a0 = agent(x_0 = 0,
                    y_0 = 0, 
                    z_0 = 0, 
                    initial_setpoint_x = 1, 
                    initial_setpoint_y = 1, 
                    initial_setpoint_z = 1, 
                    agent_priority = 1)

        a0.h = 0.01

        #------------------------------------------------------------------------

        a0.kpx = self.kpx_list[-1]          # since the instance of the agent class is re-declared each time

        a0.kpy = self.kpy_list[-1]          # test_flight is called, the gains must be re-assigned 

        a0.kpz = self.kpz_list[-1]


        a0.kdx = self.kdx_list[-1]

        a0.kdy = self.kdy_list[-1]

        a0.kdz = self.kdz_list[-1]


        a0.kix = self.kix_list[-1]

        a0.kiy = self.kiy_list[-1]

        a0.kiz = self.kiz_list[-1]


        a0.kpphi = self.kpphi

        a0.kptheta = self.kptheta

        a0.kppsi = self.kppsi 


        a0.kdphi = self.kdphi 

        a0.kdtheta = self.kdtheta 

        a0.kdpsi = self.kdpsi   


        #parse the WRT string arg and add the infinetesimal d to
        # the appropriate gain variable


        if WRT == None : None


        elif WRT == 'dkpx': a0.kpx = self.kpx_list[-1] + self.d 

        elif WRT == 'dkpy': a0.kpy = self.kpy_list[-1] + self.d

        elif WRT == 'dkpz': a0.kpz = self.kpz_list[-1] + self.d


        elif WRT == 'dkdx': a0.kdx = self.kdx_list[-1] + self.d

        elif WRT == 'dkdy': a0.kdy = self.kdy_list[-1] + self.d

        elif WRT == 'dkdz': a0.kdz = self.kdz_list[-1] + self.d


        elif WRT == 'dkix': a0.kix = self.kix_list[-1] + self.d

        elif WRT == 'dkiy': a0.kiy = self.kiy_list[-1] + self.d

        elif WRT == 'dkiz': a0.kiz = self.kiz_list[-1] + self.d


        else: 'Error parsing WRT argument'

        #-----------------------------------------------------------------------


        
        a0.max_iterations = 1000

        final_state = None

        for i in range(a0.max_iterations):
          
            if i  == ( a0.max_iterations -1 ): 
            
                ending_iteration = i
            
                final_state = 'max iterations reached'

                break    
                
            # as a preemptive assignment record the iteration number in case this is the last one
            ending_iteration = i   

            a0.system_model_block()

            a0.control_block()

            x_ave = sum(a0.x[-500:])/500.0
            y_ave = sum(a0.y[-500:])/500.0
            z_ave = sum(a0.z[-500:])/500.0

            xerr = a0.x_des - x_ave
            yerr = a0.y_des - y_ave
            zerr = a0.z_des - z_ave
                    
            # Stopping Criteria: if the agent is within a 5 cm error sphere for 200 time steps ( .2 sec )  
            
            if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) < 10**-2) and (i>50):

                final_state = 'setpoint_reached'

                break
            
            if ( sqrt( xerr**2 + yerr**2 + zerr**2 ) > 200) and (i>50):

                final_state = 'agent_is_lost!'

                break

            k_th_variable_dictionary = {
                'a0.xacc_comm[-1]' : a0.xacc_comm[-1],'a0.yacc_comm[-1]' : a0.yacc_comm[-1],'a0.zacc_comm[-1]' : a0.zacc_comm[-1],
                'a0.theta_comm[-1]' : a0.theta_comm[-1],'a0.phi_comm[-1]' : a0.phi_comm[-1],'a0.T_comm[-1]' : a0.T_comm[-1],
                'a0.tao_phi_comm[-1]' : a0.tao_phi_comm[-1],'a0.tao_theta_comm[-1]' : a0.tao_theta_comm[-1],'a0.tao_psi_comm[-1]' : a0.tao_psi_comm[-1],
                'a0.w1_arg[-1]' : a0.w1_arg[-1],'a0.w2_arg[-1]' : a0.w2_arg[-1],'a0.w3_arg[-1]' : a0.w3_arg[-1],'a0.w4_arg[-1]' : a0.w4_arg[-1],
                'a0.w1[-1]' : a0.w1[-1],'a0.w2[-1]' : a0.w2[-1],'a0.w3[-1]' : a0.w3[-1],'a0.w4[-1]' : a0.w4[-1],
                'a0.tao_qr_frame[-1][0]' : a0.tao_qr_frame[-1][0],'a0.tao_qr_frame[-1][1]' : a0.tao_qr_frame[-1][1],'a0.tao_qr_frame[-1][2]' : a0.tao_qr_frame[-1][2],
                'a0.T[-1]' : a0.T[-1],
                'a0.phi[-1]' : a0.phi[-1],'a0.theta[-1]' : a0.theta[-1],'a0.psi[-1]' : a0.psi[-1],
                'a0.phidot[-1]' : a0.phidot[-1],'a0.thetadot[-1]' : a0.thetadot[-1],'a0.psidot[-1]' : a0.psidot[-1],
                'a0.phiddot[-1]' : a0.phiddot[-1],'a0.thetaddot[-1]' : a0.thetaddot[-1],'a0.psiddot[-1]' : a0.psiddot[-1],
                'a0.x[-1]' : a0.x[-1],'a0.y[-1]' : a0.y[-1],'a0.z[-1]' : a0.z[-1],
                'a0.xdot[-1]' : a0.xdot[-1],'a0.ydot[-1]' : a0.ydot[-1],'a0.zdot[-1]' : a0.zdot[-1],
                'a0.xddot[-1]' : a0.xddot[-1],'a0.yddot[-1]' : a0.yddot[-1],'a0.zddot[-1]' : a0.zddot[-1],
                'a0.x_integral_error[-1]' : a0.x_integral_error[-1],
                'a0.y_integral_error[-1]' : a0.y_integral_error[-1],
                'a0.z_integral_error[-1]' : a0.z_integral_error[-1],
                }  

            for k in k_th_variable_dictionary.keys() : 
    
                if math.isnan(k_th_variable_dictionary[k]):

                    final_state = 'nan' #k + ' = ' + str(k_th_variable_dictionary[k])

                    #a0.print_dump(1)     
            
                    break

            if final_state != None: break  # this is needed to exit the main iteration loop after sensing 'nan' and breaking out of the for loop immdiately above 


        #-----------------------------------------------------

        # after the test_flight, map the variable lists back to the gain_optimization instance for further processing
    

        self.xacc_comm  = a0.xacc_comm
        self.yacc_comm  = a0.yacc_comm
        self.zacc_comm  = a0.zacc_comm
        self.theta_comm  = a0.theta_comm
        self.phi_comm  = a0.phi_comm
        self.T_comm  = a0.T_comm
        self.tao_phi_comm  = a0.tao_phi_comm
        self.tao_theta_comm  = a0.tao_theta_comm
        self.tao_psi_comm  = a0.tao_psi_comm
        self.w1_arg  = a0.w1_arg
        self.w2_arg  = a0.w2_arg
        self.w3_arg  = a0.w3_arg
        self.w4_arg  = a0.w4_arg
        self.w1  = a0.w1
        self.w2  = a0.w2
        self.w3  = a0.w3
        self.w4  = a0.w4
        self.tao_phi  = [i[0] for i in a0.tao_qr_frame]
        self.tao_theta  = [i[1] for i in a0.tao_qr_frame]
        self.tao_psi  = [i[2] for i in a0.tao_qr_frame]
        self.T  = a0.T
        self.phi  = a0.phi
        self.theta  = a0.theta
        self.psi  = a0.psi
        self.phidot  = a0.phidot
        self.thetadot  = a0.thetadot
        self.psidot = a0.psidot
        self.phiddot = a0.phiddot
        self.thetaddot = a0.thetaddot
        self.psiddot = a0.psiddot
        self.x = a0.x
        self.y = a0.y
        self.z = a0.z
        self.xdot = a0.xdot
        self.ydot = a0.ydot
        self.zdot = a0.zdot
        self.xddot = a0.xddot
        self.yddot = a0.yddot
        self.zddot = a0.zddot
        self.x_integral_error = a0.x_integral_error
        self.y_integral_error = a0.y_integral_error
        self.z_integral_error = a0.z_integral_error
                             

        #-----------------------------------------------------

        # calculate the terms that go into the objective function


        thrust_total = sum(a0.T)

        #-----------------------------------------------------

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


        oscillation_total = x_crossings + y_crossings + z_crossings

        #-----------------------------------------------------
        x_over_shoot = max(a0.x) -a0.x_des
        y_over_shoot = max(a0.y) -a0.y_des
        z_over_shoot = max(a0.z) -a0.z_des

        over_shoot_total = 0


        if x_over_shoot > 0: over_shoot_total += x_over_shoot

        if y_over_shoot > 0: over_shoot_total += y_over_shoot

        if z_over_shoot > 0: over_shoot_total += z_over_shoot

       #-----------------------------------------------------


        over_shoot_coeff  = over_shoot_total         # attempt to scale each term so that under 'resonable' cirumstances, 

        oscillation_coeff = oscillation_total       # no one term dominates the value of the objective function

        thrust_coeff = (0.01)*thrust_total

        #-----------------------------------------------------

        objective_function = thrust_coeff + oscillation_coeff + over_shoot_coeff


        #delta_obj_fun = objective_function - self.flight_record[-1]['objective_function']

        #percent_delta_obj_fun = delta_obj_fun / objective_function


        flight_time = ending_iteration * a0.h 

        #each test flight should produce the following information:

        test_flight_results = {
                'a0.kpx' : a0.kpx,
                'a0.kpy' : a0.kpy,
                'a0.kpz' : a0.kpz,
                'a0.kdx' : a0.kdx,
                'a0.kdy' : a0.kdy,
                'a0.kdz' : a0.kdz,
                'a0.kix' : a0.kix,
                'a0.kiy' : a0.kiy,
                'a0.kiz' : a0.kiz,
                'WRT':WRT,
                'flight_time': flight_time,
                'oscillation_coeff' : oscillation_coeff,
                'over_shoot_coeff': over_shoot_coeff,
                'thrust_total' : thrust_total,
                'objective_function':objective_function,
                'ending_iteration': ending_iteration,
                'final_state': final_state
                }  

        '''
        'delta_obj_fun':delta_obj_fun,
        'percent_delta_obj_fun':percent_delta_obj_fun,

        '''





        print '\ntest_flight_results[WRT] = ',test_flight_results['WRT']
        print '\ntest_flight_results[objective_function] = ',test_flight_results['objective_function']
        #pp.pprint(test_flight_results)

        self.timeSeries = [a0.h*i for i in range(len(a0.x))]

        if WRT == None: 

            date_and_time = datetime.now().strftime('%Y-%m-%d__%H.%M.%S')

            fig1_file_path = self.plot_dir + '/test_flight_fig1_' + date_and_time + '.png' 
            fig2_file_path = self.plot_dir + '/test_flight_fig2_' + date_and_time + '.png'

            a0.plot_results(False,True,fig1_file_path,fig2_file_path)

        return test_flight_results

    #---------------------------------------------------------------------------
    def gain_gradient(self):

        t = self.test_flight(None)




        if len(self.flight_record) > 1:

            change_in_objective_function = t['objective_function'] - self.flight_record[-1]['objective_function']
            print '\nchange_in_objective_function = ',change_in_objective_function    

        else:
            
            change_in_objective_function = None

            
            

        if (change_in_objective_function <= 0) or (change_in_objective_function == None): 
    
            self.flight_record.append(t)

            if t['final_state'] == 'nan':

                print 'numerical explosion...'

                self.file_dump()
                
        

            tfd_list = [self.test_flight('dkpx'),
                        self.test_flight('dkpy'),
                        self.test_flight('dkpz'),
                        self.test_flight('dkdx'),
                        self.test_flight('dkdy'),
                        self.test_flight('dkdz'),
                        self.test_flight('dkix'),
                        self.test_flight('dkiy'),
                        self.test_flight('dkiz')]

            # if any of the above test flights did not reach the set point then 
            # simply do the file dump and exit the gain optimization loop


            for tf in tfd_list:

                if tf['final_state'] == 'nan': 

                    print 'numerical explosion...'

                    self.file_dump()



            # compute the finite differences that will populate the gradient

            t_objfun = t['objective_function']

            dthrust_dkpx = (tfd_list[0]['objective_function'] - t_objfun) / self.d
            dthrust_dkpy = (tfd_list[1]['objective_function'] - t_objfun) / self.d
            dthrust_dkpz = (tfd_list[2]['objective_function'] - t_objfun) / self.d       

            dthrust_dkdx = (tfd_list[3]['objective_function'] - t_objfun ) / self.d
            dthrust_dkdy = (tfd_list[4]['objective_function'] - t_objfun) / self.d
            dthrust_dkdz = (tfd_list[5]['objective_function'] - t_objfun) / self.d        
     
            dthrust_dkix = (tfd_list[6]['objective_function'] - t_objfun) / self.d
            dthrust_dkiy = (tfd_list[7]['objective_function'] - t_objfun) / self.d
            dthrust_dkiz = (tfd_list[8]['objective_function'] - t_objfun) / self.d   
 


         
            grad = [dthrust_dkpx, 
                    dthrust_dkpy, 
                    dthrust_dkpz, 
                    dthrust_dkdx, 
                    dthrust_dkdy, 
                    dthrust_dkdz,
                    dthrust_dkix,
                    dthrust_dkiy,
                    dthrust_dkiz
                    ]


     
            return grad

        elif change_in_objective_function > 0:
            print '\nchange_in_objective_function > 0 :',change_in_objective_function 

            return None
    #---------------------------------------------------------------------------



    def file_dump(self):
  
        date_and_time = datetime.now().strftime('%Y-%m-%d__%H.%M.%S')

        #---------------------------------------------------------------------------------------------

        row0 =[
            'kpx','kpy','kpz',
            'kdx','kdy','kdz',
            'kix','kiy','kiz',
            'WRT',
            'ending_iteration',
            'oscillation_coeff',
            'over_shoot_coeff',
            'thrust_total',
            'objective_function',
            'delta_obj_fun',
            'percent_delta_obj_fun',
            'flight_time',
            'final_state']

        rows = [row0]

        for tf in self.flight_record:

            rows.append([
                        around(tf['a0.kpx'],4),
                        around(tf['a0.kpy'],4),
                        around(tf['a0.kpz'],4),
                        around(tf['a0.kdx'],4),
                        around(tf['a0.kdy'],4),
                        around(tf['a0.kdz'],4),
                        around(tf['a0.kix'],4),
                        around(tf['a0.kiy'],4),
                        around(tf['a0.kiz'],4), 
                        tf['WRT'],
                        tf['ending_iteration'],
                        around(tf['oscillation_coeff'],4),
                        around(tf['over_shoot_coeff'],4),
                        around(tf['thrust_total'],4),
                        around(tf['objective_function'],4),                        #around(tf['delta_obj_fun'],4),                        
                        tf['flight_time'],                                         #around(tf['percent_delta_obj_fun'],4),
                        tf['final_state']
                        ])

        import csv

        filepath = self.plot_dir + '/gain_optimization_output_' + date_and_time+ '.csv'

        with open(filepath, 'wb') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')

            for row in rows:
                spamwriter.writerow(row)

        from os import system
        system('libreoffice '+filepath+' &')


        # after the file dump simply exit, 
        sys.exit()





                


#################################################################################
if __name__ == '__main__':


    go = gain_optimization()
    

    # need to initialize thrust_sum_list with two values that are in decending order
   

    for j in range(100):

        print '\n-----------------------------------'

        # the gain gradient function returns the grad if successful, 
        # returns None if the change in gradient is positive 
        # if the process blows up it exits after a file dump    

        gg = go.gain_gradient()
        
        if gg != None:

            go.grad_norm_list.append( float(norm(gg)) )

            go.normalized_gradient_list.append(array(gg) / go.grad_norm_list[-1] )

            print '\ngg = ',gg
            
            print 'go.grad_norm_list[-1] = ',go.grad_norm_list[-1]

            print 'go.flight_record[-1] = ', go.flight_record[-1],'\n'


           
            old_gains = array([
                                go.kpx_list[-1],
                                go.kpy_list[-1], 
                                go.kpz_list[-1],
                                go.kdx_list[-1], 
                                go.kdy_list[-1], 
                                go.kdz_list[-1],
                                go.kix_list[-1],          
                                go.kiy_list[-1], 
                                go.kiz_list[-1]
                              ])


            new_gains = old_gains - go.optimization_step_size * array(go.normalized_gradient_list[-1])

            go.kpx_list.append(new_gains[0])
            go.kpy_list.append(new_gains[1])
            go.kpz_list.append(new_gains[2])

            go.kdx_list.append(new_gains[3])
            go.kdy_list.append(new_gains[4])
            go.kdz_list.append(new_gains[5])


            if new_gains[6] <= 0:

                go.kix_list.append(0)

            elif new_gains[6] > 0:

                go.kix_list.append(new_gains[6])



            if new_gains[7] <= 0:

                go.kiy_list.append(0)

            elif new_gains[7] > 0:

                go.kiy_list.append(new_gains[7])


            if new_gains[8] <= 0:

                go.kiz_list.append(0)

            elif new_gains[8] > 0:

                go.kiz_list.append(new_gains[8])

            #------------------------------------------------------------------------

            percent_diff = 100 * ( go.flight_record[-1]['objective_function'] 
                                 - go.flight_record[-2]['objective_function'] ) / float(go.flight_record[-1]['objective_function'])

            print 'objective function percent change = ',percent_diff

     
            if (abs(percent_diff) <= 0.1) and (len(go.flight_record) > 10) and (percent_diff != 0):

                print '\n\npercent_diff between the total thrust from the last two iterations < 0.1%' 

                print '\neffective minimum of objective function found!'

                go.file_dump()

                break    

    
        if gg == None:                                    

            #if go.flight_record[-1]['delta_obj_fun'] > 0:

                #if go.flight_record[-2]['delta_obj_fun'] > 0:
 
            go.optimization_step_size = go.optimization_step_size * 0.5    

            print '\nobjective function value has increased !!'
            print 'go.optimization_step_size = go.optimization_step_size * 0.5'
            print 'using previous values of gain variables \n'
            
            go.kpx_list = go.kpx_list[:-1]
            go.kpy_list = go.kpy_list[:-1]
            go.kpz_list = go.kpz_list[:-1]

            go.kdx_list = go.kdx_list[:-1]
            go.kdy_list = go.kdy_list[:-1]
            go.kdz_list = go.kdz_list[:-1]

            go.kix_list = go.kix_list[:-1]          
            go.kiy_list = go.kiy_list[:-1] 
            go.kiz_list = go.kiz_list[:-1] 
  
