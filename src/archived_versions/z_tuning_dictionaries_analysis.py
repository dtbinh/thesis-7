

import json


with open('z_tuning_dictionaries_data.json', 'rb') as fp:

    z_tuning_dictionaries = json.load(fp)

good_runs = []

ku_list = []
total_thrust_list = []

for i in z_tuning_dictionaries:

    if ('nan' not in i.values() ) and ('err' not in i.values() ) and (None not in i.values() ):

        good_runs.append(i)
    
        total_thrust_list.append(i['total_thrust'])

        ku_list.append(i['ku'])

    else:

        total_thrust_list.append(0)   
           
        ku_list.append(i['ku'])









import matplotlib.pyplot as plt
from pylab import title  
fig1 = plt.figure()

ku = fig1.add_subplot(111)
plt.plot(ku_list,total_thrust_list,'o')
title('ku vs total thrust',fontsize=10)
plt.show()

'''
min_total_thrust = 1000000  

optimal_run = None

for r in good_runs:

    if r['total_thrust'] < min_total_thrust:

        optimal_run = r

        min_total_thrust = r['total_thrust']

print '\n\noptimal run:',optimal_run
'''
