
from agent_module import *

a = agent(x_0 = 0,    y_0 = 0,   z_0 = 0, initial_setpoint_x = 5, initial_setpoint_y = 10, initial_setpoint_z = 15, agent_priority = 1)


a.kpx = 1          # -------------------------------PID proportional gain values
a.kpy = 1
a.kpz = 1
a.kdx = 1           #--------------------------------PID derivative gain values
a.kdy = 1
a.kdz = 1
a.kix = 1
a.kiy = 1
a.kiz = 1

for i in range(a.max_iterations):
  
    a.system_iteration(
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

    a.xError += (a.x[-1]-a.x_des)*a.h

    a.yError += (a.y[-1]-a.y_des)*a.h

    a.zError += (a.z[-1]-a.z_des)*a.h

    if i%1000 == 0:
    
        print 'i = ',i
    
        print 'a.xError, a.yError, a.zError = ',a.xError,',',a.yError,',',a.zError


    if (a.xError < 0.01) and (a.yError < 0.01) and (a.zError < 0.01) and (i>50):

        print 'setpoint reached!!'

        print 'i = ', i

        #print 'ending_iteration = ',self.ending_iteration

        break



plot_results(a.h,
             a.x,
             a.y,
             a.z,
             a.phi,
             a.theta,
             a.psi,
             a.w1,
             a.w2,
             a.w3,
             a.w4,
             a.tao_phi,
             a.tao_theta,
             a.tao_psi,
             a.T,
             a.wind_x,
             a.wind_y,
             a.wind_z)




