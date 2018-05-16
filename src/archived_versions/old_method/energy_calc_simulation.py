# -*- coding: utf-8 -*-
"""
Created on Wed Jan 16 18:13:59 2013

@author: ekemper
"""

from quadrotor_descreteTime import *
import steepest_descent_method___motorSpeeds as sdm
import numpy as n
import pprint
pp = pprint.PrettyPrinter(indent=2)


# the function go() is from 'quadrotor_descreteTime.py
h = 0.005
max_timesteps = 500 #this is the number of iterations in the time series, 

go(1,1,1,h,max_timesteps)
#go(10,20,30,h,max_timesteps)    
#go(30,80,100,h,max_timesteps)
#go(20,30,30,h,max_iter)
#go(30,50,10,h,max_iter)
#go(50,60,10,h,max_iter)    
#go(50,60,20,h,max_iter)


# the simulation of the  
vx = [(x[i+1]-x[i])/h for i in range(len(x)-1)]
vy = [(y[i+1]-y[i])/h for i in range(len(y)-1)]
vz = [(z[i+1]-z[i])/h for i in range(len(z)-1)]
vPs = [(Ps[i+1]-Ps[i])/h for i in range(len(Ps)-1)]
vPh = [(Ph[i+1]-Ph[i])/h for i in range(len(Ph)-1)]
vTh = [(Th[i+1]-Th[i])/h for i in range(len(Th)-1)]


ax = [(vx[i+1]-vx[i])/h for i in range(len(vx)-1)]# the acc as measured from the inertial frame
ay = [(vy[i+1]-vy[i])/h for i in range(len(vy)-1)]
az = [(vz[i+1]-vz[i])/h for i in range(len(vz)-1)]
aPs = [(vPs[i+1]-vPs[i])/h for i in range(len(vPs)-1)]
aPh = [(vPh[i+1]-vPh[i])/h for i in range(len(vPh)-1)]
aTh = [(vTh[i+1]-vTh[i])/h for i in range(len(vTh)-1)]

print '\nlen(ax) =',len(ax)

acc_vec_timeseries = [n.array( [ax[i], ay[i], az[i], aPs[i], aPh[i], aTh[i]] ) for i in range(len(ax))]
print '\nacc_vec_timeseries = '
pp.pprint( acc_vec_timeseries[:10])
print'...'
pp.pprint(acc_vec_timeseries[-10:])

'''
next use the steepest descent method to solve for the motor speeds at each timestep. the integral of this list
is proportional to the total energy used in the manuver


usage example of steepest_descent_method function:
    
call:    solution = solve_for_motor_speeds(h,  ph,th,ps,  phd,thd,psd,  phdd,thdd,psdd,  xdd,ydd,zdd,  w1p, w2p, w3p, w4p) 

what is returned:  [
                    w,                         # the solution to the steepest descent method = w[k] = [ w1[k],w2[k],w3[k],w4[k] ] 
                    grad_norm_list,            # a list of the norm of the gradient at each time step 
                    kth_stopping_condition,    
                    end_iteration_number
                   ]    


'''

# this is the initialization of the list of motor speeds for each timestep
w_timeseries = [[600,700,800,900]]



ending_iteration_list = []
stopping_condition_list = []


# this loop calls the 'solve_for_motor_speeds()' function at each timestep
for k in range(len(acc_vec_timeseries)):
    solution = sdm.solve_for_motor_speeds(

                                 Ph[k],
                                 Th[k],
                                 Ps[k], 
                                 
                                 vPh[k],
                                 vTh[k],
                                 vPs[k],

                                 acc_vec_timeseries[k][3], #psidd
                                 acc_vec_timeseries[k][4], #phidd
                                 acc_vec_timeseries[k][5], #Thedd 
                                 
                                 acc_vec_timeseries[k][0], #xdd
                                 acc_vec_timeseries[k][1], #ydd
                                 acc_vec_timeseries[k][2], #zdd

                                 w_timeseries[-1][0], # w1p,
                                 w_timeseries[-1][1], # w2p,
                                 w_timeseries[-1][2], # w3p,
                                 w_timeseries[-1][3], # w4p
                                 )

    # save the appropriate values of the solution at the kth timestep for analysis
    wi = solution[0]
    grad_norm_list = solution[1]
    kth_stopping_condition = solution[2]
    end_iteration_number = solution[3]
    
    #update the timeseries lists	
    w_timeseries.append(wi[-1])    
    ending_iteration_list.append(end_iteration_number)
    stopping_condition_list.append(kth_stopping_condition)

    print 'k = ',k

    # for debugging, show solution every 100 timesteps 
    if k%50 == 0:
        print '\nk = ',k
        print 'wi[-1] = ',wi[-1]
        print 'end_iteration_number = ', end_iteration_number
        print 'kth_stopping_condition = ',kth_stopping_condition
#--------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------
#reporting Statistics of the simulation:        
'''        
print '\nw_timeseries = '
pp.pprint(w_timeseries[:10])
print '...'
pp.pprint(w_timeseries[-10:])

print '\nending_iteration_list = '
pp.pprint(ending_iteration_list[:10])
print '...'
pp.pprint(ending_iteration_list[-10:])
'''

stopping_condition_1_count = 0
stopping_condition_2_count = 0
stopping_condition_3_count = 0

for j in range(len(stopping_condition_list)):
    if stopping_condition_list[j]==1: stopping_condition_1_count = stopping_condition_1_count + 1
for j in range(len(stopping_condition_list)):
    if stopping_condition_list[j]==2: stopping_condition_2_count = stopping_condition_2_count + 1
for j in range(len(stopping_condition_list)):
    if stopping_condition_list[j]==3: stopping_condition_3_count = stopping_condition_3_count + 1    


print '\nSTOPPING CONDITION 1) iterates of the norm of grad(g) converged to within tol :'
print 'stopping_condition_1_count',stopping_condition_1_count
print '\nSTOPPING CONDITION 2) small steadystate osc detected in the iterates of w'
print 'stopping_condition_2_count',stopping_condition_2_count
print '\nSTOPPING CONDITION 3: the sequence did NOT converge, max_iterations reached!'
print 'stopping_condition_3_count',stopping_condition_3_count


print '\naverage ending iteration',n.mean(ending_iteration_list)
print '\nstandard deviation of ending iteration',n.std(ending_iteration_list)
'''
print '\nstopping_condition_list = '
pp.pprint(stopping_condition_list[:10])
print '...'
pp.pprint(stopping_condition_list[-10:])
'''

#create lists of motor speeds for plotting
w1_plotseries = [w_timeseries[m][0] for m in range(len(w_timeseries))]
w2_plotseries = [w_timeseries[m][1] for m in range(len(w_timeseries))]
w3_plotseries = [w_timeseries[m][2] for m in range(len(w_timeseries))]    
w4_plotseries = [w_timeseries[m][3] for m in range(len(w_timeseries))]    

print '\nthe time-integral of all motor speeds =', h*(sum(w1_plotseries)+sum(w2_plotseries)+sum(w3_plotseries)+sum(w4_plotseries))

# create the layout for the plot figure
import matplotlib.gridspec as gridspec
gs = gridspec.GridSpec(4,5)


fig = figure()
'''
# the 3D position plot
axes = fig.add_subplot(gs[0,4], projection='3d')
axes.scatter(x, y, z)
axes.set_xlabel('X')
axes.set_ylabel('Y')
axes.set_zlabel('Z')
'''
# rw is the time-series for plotting the motor speeds
rw = h*n.array(range(len(w_timeseries)))

ww = fig.add_subplot(gs[0:2,0:2])
ww.plot(rw,w1_plotseries ,  rw, w2_plotseries, rw, w3_plotseries, rw, w4_plotseries)
title('the four motor speeds over time',fontsize=10)
   
# rw is the time-series for plotting positions
rx = h*n.array(range(len(x)))

xx = fig.add_subplot(gs[0,2])#----------------------linear displacements
plt.plot(rx, x, rx, y,rx, z)
title('x,y,z',fontsize=10)

ra = h*n.array(range(len(ax)))
aa = fig.add_subplot(gs[0,4])#----------------------linear accelerations
plt.plot(ra, ax, ra, ay , ra , az)
title('ax,ay,az',fontsize=10)

rv = h*n.array(range(len(vx)))
vv = fig.add_subplot(gs[0,3])#----------------------linear accelerations
plt.plot(rv, vx, rv, vy , rv , vz)
title('vx,vy,vz',fontsize=10)


thth = fig.add_subplot(gs[1,2])#---------------------angles
plt.plot(rx, Th, rx , Ph , rx , Ps)
title('pitch roll yaw',fontsize=10)



athth = fig.add_subplot(gs[1,3])#---------------------angular acceleration
plt.plot(rv, vTh, rv , vPh , rv , vPs)
title('angular vel',fontsize=10)

athth = fig.add_subplot(gs[1,4])#---------------------angular acceleration
plt.plot(ra, aTh, ra , aPh , ra , aPs)
title('angular accelerations',fontsize=10)



r_debug = range(len(ending_iteration_list))

debug_1 = fig.add_subplot(gs[2,0:2])#---------------------ending iteration
plt.plot(r_debug, ending_iteration_list,"b-")
title('ending iteration = blue',fontsize=10)

debug_2 = fig.add_subplot(gs[3,0:2])#---------------------stoping condition\
plt.plot( r_debug , stopping_condition_list,'r-')
title('kth stopping condition = red',fontsize=10)

show()

