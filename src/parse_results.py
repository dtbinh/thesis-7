

import os
import json
import itertools
from operator import itemgetter
import pprint
pp = pprint.PrettyPrinter(indent=4)
import numpy

'''
def obj_fun(d):

    of = d['total_thrust']

    return of
'''




# need to go through all the output files and make a list of all the sims that still need to be run:

output_dir = '/home/ek/Dropbox/THESIS/python_scripts/brute_force_output_data/'

output_file_names = [fn for fn in os.listdir(output_dir)]

output_file_paths = [output_dir + ofn for ofn in output_file_names]


data = []

for ofp in output_file_paths:

    with open(ofp, 'rb') as fp:
        output_data = json.load(fp)

        data = data + output_data

#---------------------------------------collect the runs that actually converged

good_runs = []

for d in data:

    if d['return_val2'] == 1:

        good_runs.append(d)

number_of_convergent_runs = len(good_runs)

# --------------------------------------create a list of total thrust values from the convergent runs

T_list = [g['total_thrust'] for g in good_runs]

T_min = numpy.amin(T_list)
T_ave = numpy.mean(T_list)


# ----------------------------------------for the runs that actually converged, find the ones that satisfied the overshoot criteria

min_overshoot_runs = []

for g in good_runs:

    if ( g['x_over_shoot'] < 0.1 ) and ( g['y_over_shoot'] < 0.1 ) and ( g['z_over_shoot'] < 0.1 ):

        min_overshoot_runs.append(g)



min_oscillation_runs = []

for r in min_overshoot_runs:

    if (r['x_crossings'] < 4) and (r['y_crossings'] < 4) and (r['z_crossings'] < 4) :

        min_oscillation_runs.append(r)



thrust_sorted_good_runs = sorted(min_oscillation_runs, key=itemgetter('total_thrust'))

for t in thrust_sorted_good_runs[:20]:
    print '\n'
    pp.pprint(t)


##########################################

print "\n\nnumber_of_convergent_runs = ",  number_of_convergent_runs

print "minimum thrust = ", T_min

print "average total thrust = " , T_ave

print "number_of_runs_with_satisfactory_overshoot = ", len(min_overshoot_runs)

ave_x_crossings = numpy.mean([g['x_crossings'] for g in good_runs])
ave_y_crossings = numpy.mean([g['y_crossings'] for g in good_runs])
ave_z_crossings = numpy.mean([g['z_crossings'] for g in good_runs])

print 'ave_x_crossings = ', ave_x_crossings
print 'ave_y_crossings = ', ave_y_crossings
print 'ave_z_crossings = ', ave_z_crossings

ave_x_overshoot = numpy.mean([g['x_over_shoot']for g in good_runs])
ave_y_overshoot = numpy.mean([g['y_over_shoot']for g in good_runs])
ave_z_overshoot = numpy.mean([g['z_over_shoot']for g in good_runs])

print 'ave_x_overshoot = ' , ave_x_overshoot
print 'ave_y_overshoot = ' , ave_y_overshoot
print 'ave_z_overshoot = ' , ave_z_overshoot

print "number_of_runs_with_satisfactory_oscillations =  ", len(min_oscillation_runs)




# according to the available data, here is the best run...
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














