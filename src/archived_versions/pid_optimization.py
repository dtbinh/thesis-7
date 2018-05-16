
from agent_module import *
from numpy.linalg import norm


class gain_optimization():

    def __init__(self):

        self.d = 0.001 # the increment for the finite differences in the gradient

        

        self.step_size = 1

        self.thrust_sum_list = [10000,9999]

        self.grad_norm_list = []

        self.kpx_list = [35]          # ------------PID proportional gain values
        self.kpy_list = [35]
        self.kpz_list = [35]
        self.kdx_list = [20]          #------------PID derivative gain values
        self.kdy_list = [20]
        self.kdz_list = [20]

        self.ending_iteration = 0

    #---------------------------------------------------------------------------

    # the argument delta gives an offset to the gain values for use
    # in computing the gradient

    def test_flight(self,WRT):  

        self.a0 = agent(x_0 = 0,
                        y_0 = 0, 
                        z_0 = 0, 
                        initial_setpoint_x = 1, 
                        initial_setpoint_y = 1, 
                        initial_setpoint_z = 1, 
                        agent_priority = 1)

        self.a0.h = 0.001


        #parse the WRT string arg and add the infinetesimal d to
        # the appropriate gain variable





        self.a0.kpx = self.kpx_list[-1] 

        self.a0.kpy = self.kpy_list[-1]

        self.a0.kpz = self.kpz_list[-1]

        self.a0.kdx = self.kdx_list[-1]

        self.a0.kdy = self.kdy_list[-1]

        self.a0.kdz = self.kdz_list[-1]



        if WRT == None : None

        elif WRT == 'dkpx': self.a0.kpx =+ self.d 

        elif WRT == 'dkpy': self.a0.kpy =+ self.d

        elif WRT == 'dkpz': self.a0.kpz =+ self.d

        elif WRT == 'dkdx': self.a0.kdx =+ self.d

        elif WRT == 'dkdy': self.a0.kdy =+ self.d

        elif WRT == 'dkdz': self.a0.kdz =+ self.d

        else: 'Error parsing WRT argument'




        #run the sim for the current values of all the gain variables

        for i in range(self.a0.max_iterations):

            #if i%1000 == 0: print 'i = ',i

            self.a0.iteration_gain_input(i)
            
            x_ave = sum(self.a0.x[-20:])/20.0
            y_ave = sum(self.a0.y[-20:])/20.0
            z_ave = sum(self.a0.z[-20:])/20.0

            if (abs(x_ave - self.a0.x_des) < 0.1) and (abs(y_ave - self.a0.y_des) < 0.1) and (abs(z_ave - self.a0.z_des) < 0.1) and (i>50):

                #print 'setpoint reached!!'

                self.ending_iteration = i

                #print 'ending_iteration = ',self.ending_iteration

                break
                #-------------------------------------------------
 
        #print 'self.a0.T = ',self.a0.T

        self.thrust_total = sum(self.a0.T)
        #print 'sum of total thrust = ', self.thrust_total

        return self.thrust_total 
        
 
    #---------------------------------------------------------------------------
    def gain_gradient(self):

        t = self.test_flight(None)
        
        self.thrust_sum_list.append(t)

        dt_dkpx = (t - self.test_flight('dkpx')) / self.d
        dt_dkpy = (t - self.test_flight('dkpy')) / self.d
        dt_dkpz = (t - self.test_flight('dkpz')) / self.d       

        dt_dkdx = (t - self.test_flight('dkdx')) / self.d
        dt_dkdy = (t - self.test_flight('dkdy')) / self.d
        dt_dkdz = (t - self.test_flight('dkdz')) / self.d        
    
        grad = [dt_dkpx, dt_dkpy, dt_dkpz, dt_dkdx, dt_dkdy, dt_dkdz]


 
        return grad

#################################################################################
if __name__ == '__main__':


    go = gain_optimization()
    

    # need to initialize thrust_sum_list with two values that are in decending order
   

    for j in range(100):

        print '\n-----------------------------------'

        gg = go.gain_gradient()
        
        go.grad_norm_list.append( float(norm(gg)) )

        normalized_gradient = array(gg) / go.grad_norm_list[-1]

        print 'go.ending_iteration = ',go.ending_iteration

        print 'gg = ',gg
        
        print 'go.grad_norm_list[-1] = ',go.grad_norm_list[-1]

        print 'go.thrust_sum_list[-1] = ', go.thrust_sum_list[-1]

        print '[kpx,kpy,kpz,kdx,kdy,kdz] = ',[go.kpx_list[-1],
                                              go.kpy_list[-1],
                                              go.kpz_list[-1],
                                              go.kdx_list[-1],
                                              go.kdy_list[-1],
                                              go.kdz_list[-1]]

        print 'go.thrust_sum_list[-1] = ',go.thrust_sum_list[-1]
        print 'go.thrust_sum_list[-2] = ',go.thrust_sum_list[-2]

        if (go.thrust_sum_list[-1] > go.thrust_sum_list[-2]):

            go.step_size = go.step_size * 0.1
            
            #go.d = go.d * 0.1    

            print 'objective function value has increased !!,\n go.step_size = go.step_size * 0.1,\n using previous values of gain variables '
            
            go.kpx_list = go.kpx_list[:-1]
            go.kpy_list = go.kpy_list[:-1]
            go.kpz_list = go.kpz_list[:-1]
            go.kdx_list = go.kdx_list[:-1]
            go.kdy_list = go.kdy_list[:-1]
            go.kdz_list = go.kdz_list[:-1]

        elif (go.thrust_sum_list[-1] <= go.thrust_sum_list[-2]):

            old_gains = array([go.kpx_list[-1],
                               go.kpy_list[-1], 
                               go.kpz_list[-1],
                               go.kdx_list[-1], 
                               go.kdy_list[-1], 
                               go.kdz_list[-1]])

            new_gains = old_gains - go.step_size * array(normalized_gradient)

            go.kpx_list.append(new_gains[0])
            go.kpy_list.append(new_gains[1])
            go.kpz_list.append(new_gains[2])
            go.kdx_list.append(new_gains[3])
            go.kdy_list.append(new_gains[4])
            go.kdz_list.append(new_gains[5])

        percent_diff = 100 * abs( go.thrust_sum_list[-1] - go.thrust_sum_list[-2] ) / float(go.thrust_sum_list[-1])

        print 'thrust sum list percent change = ',percent_diff

 
        if percent_diff < 0.1 :

            print '\n\npercent_diff between the last two iterations < 0.1%' 

            print '\neffective minimum of objective function found!'

            break    

               

    #----------------------------------------------------------------------  
    import matplotlib.pyplot as plt
    from pylab import title   
    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(8, 2)

    fig = plt.figure()

    #----------------------------------------------------------------------------linear displacements
    thrust_sum_plot = fig.add_subplot(gs[0:2,0:2])
    plt.plot(range(len(go.thrust_sum_list[3:])), go.thrust_sum_list[3:],'r')
    title('go.thrust_sum_list',fontsize=10)


    grad_norm_plot = fig.add_subplot(gs[2:4,0:2])
    plt.plot(range(len(go.grad_norm_list)), go.grad_norm_list)
    title('go.grad_norm_list',fontsize=10)
    
    proportional_gain_plot =  fig.add_subplot(gs[4:6,0:2])
    plt.plot(range(len(go.kpx_list)) , go.kpx_list, 'r',
             range(len(go.kpy_list)) , go.kpy_list, 'g',
             range(len(go.kpz_list)) , go.kpz_list, 'b',                
            )
    title('kpx = r, kpy = g, kpz = b',fontsize=10)



    derivative_gain_plot =  fig.add_subplot(gs[6:8,0:2])
    plt.plot(range(len(go.kdx_list)) , go.kdx_list, 'r',
             range(len(go.kdy_list)) , go.kdy_list, 'g',
             range(len(go.kdz_list)) , go.kdz_list, 'b',                
            )
    title('kdx = r, kdy = g, kdz = b',fontsize=10)

    plt.tight_layout()

    plt.show()
                        



