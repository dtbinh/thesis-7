
from zn_tuning import *

main_data_list = []

for i in range(10):

    set_point = [0,0,i]

    z_tuning_dictionaries = tune_z_direction(set_point)

    good_runs = []

    for i in z_tuning_dictionaries:

        if ('nan' not in i.values() ) and ('err' not in i.values() ) and (None not in i.values() ):

            good_runs.append(i)
    '''
    min_total_thrust = 1000000  

    optimal_run = None

    for r in good_runs:

        if r['total_thrust'] < min_total_thrust:

            optimal_run = r

            min_total_thrust = r['total_thrust']
       
    print '\n\noptimal run:',optimal_run
    '''


    main_data_list.append(good_runs)

with open('/home/ek/Dropbox/THESIS/python_scripts/setpoint_range_z_tuning_dictionaries.json', 'wb') as fp:
    json.dump(main_data_list, fp)


