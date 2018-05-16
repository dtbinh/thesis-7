#!/usr/bin/env python
from pylab import plt, show, title
import numpy as n

#some sample waypoints
wpt_domain = [0,1,2,3,4,5,6,7,8,9,10,11]
wpt = [0,1,4,1,2,3,5,7,5,4,3,1]

'''
need to write each line segment as a function of t:

y - y1 = m * ( t - t1 ) 

y = m * ( t - t1 ) + y1

y = m *  t - m * t1  + y1

where m = (y2-y1/t2-t1)
'''

int_data = []# the set of data created by linear interpolation between the waypoints
int_data_domain= []

h = 0.1
    
for i in range( len( wpt ) - 1 ):
    
    #calculate a slope for the ith pair of waypoints
    m_i = ( wpt[i+1] - wpt[i] ) / ( wpt_domain[i+1] - wpt_domain[i]  )
    
    #the variable k is the domain of the interpolated data
    f_i = lambda k : m_i *  ( k -  wpt_domain[i] )  + wpt[i]    
    
    
    ith_int_data_domain = n.arange( wpt_domain[i], wpt_domain[i+1], h )
#    print '\nith_int_data_domain =',ith_int_data_domain
    int_data_domain.extend(ith_int_data_domain)

    
    for m in ith_int_data_domain:
        int_data.append( f_i(m) )
print 'int_data_domain = ', int_data_domain
print 'int_data =',[round( int_data[g], 3 ) for g in range(len(int_data))]

#create lisets for the the vel and acc data
vel_data = [(int_data[i+1]-int_data[i])/h for i in range(len(int_data)-1)]
print 'vel_data = ',vel_data
acc_data = [(vel_data[i+1]-vel_data[i])/h for i in range(len(vel_data)-1)]
print 'acc_data =',acc_data


acc_domain = int_data_domain[1:-1]

#import matplotlib.gridspec as gridspec
#gs = gridspec.GridSpec(1,3)

fig = plt.figure()

x_plot = fig.add_subplot(111)#gs[0,0])#----------------------linear displacements
plt.plot(int_data_domain, int_data, 'bo', wpt_domain, wpt, 'ro',acc_domain,acc_data)
title('interpolated data',fontsize=10)
'''
dx_plot = fig.add_subplot(gs[0,1])
plt.plot(int_data_domain[:-1], vel_data)
title('velocity',fontsize=10)

ddx_plot = fig.add_subplot(gs[0,2])
plt.plot(int_data_domain[:-2],acc_data)
title('acceleration',fontsize=10)
'''
show()
#==================================================================================


'''
use the liner interpolation data as the starting points for the minimization process.... maybe
'''

