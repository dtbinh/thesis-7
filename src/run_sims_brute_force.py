'''
THis is a last resort , brute force approch to finding the gain vector that 
produces the lowest objective function value for a set point of (1,1,2)

each run will start with the state variable and input lists produced by the take_off
function . for speed this data will be read from a json file which is produced beforehand

for each run the ku gain variable will be incremented by 5 and the objective function measured
'''
import sys
from numpy import arange
from brute_force_functions import *
from datetime import datetime
import json
import time
'''
gain_dictionary = {
                    'kpxy' : 10,     
                    'kpz'  : 40,
                    'kdxy' : 10,
                    'kdz'  : 40,
                    'kixy' : 0.5,
                    'kiz'  : 25}
'''

global_start_time = time.time()


kpxy_range = arange(5,30,5)

kpz_range = arange(20,70,10)

kdxy_range = arange(5,30,5)

kdz_range = arange(20,70,10)

kixy_range = arange(0.2,1.0,0.2)

kiz_range = arange(15,45,5)

print 'kpxy_range = ',kpxy_range

print 'kpz_range = ',kpz_range

print 'kdxy_range = ',kdxy_range

print 'kdz_range = ',kdz_range

print 'kixy_range = ',kixy_range

print 'kiz_range = ',kiz_range

number_of_sims = len(kpxy_range)*len(kpz_range)*len(kdxy_range)*len(kdz_range)*len(kixy_range)*len(kiz_range)

print 'number_of_sims = ',number_of_sims


index = int(sys.argv[1])

runtimes = []
run_dictionaries = []

for kpxy in [kpxy_range[index]]:
    for kpz in kpz_range:
        for kdxy in kdxy_range:
            for kdz in kdz_range:
                for kixy in kixy_range:

                    date_and_time = datetime.now().strftime('%Y-%m-%d__%H.%M.%S')

                    filepath = '/home/ek/Dropbox/THESIS/python_scripts/brute_force_output_data/bruteforce_output_index'+str(index)+'_' + date_and_time+ '.json'

                    with open(filepath, 'wb') as fp: 
                        json.dump(run_dictionaries, fp)
                        fp.close()
                                

                    run_dictionaries = []

                    for kiz in kiz_range:  

                        gain_dictionary = {
                                        'kpxy' : kpxy,     
                                        'kpz'  : kpz,
                                        'kdxy' : kdxy,
                                        'kdz'  : kdz,
                                        'kixy' : kixy,
                                        'kiz'  : kiz}

                        print '\ngain_dictionary = ',gain_dictionary

                        ith_starttime = time.time()                        

                        agent = take_off() # ----> returns the agent instance hovering at (0,0,1)      

                        set_point = [1,1,2]

                        test_run_dictionary = test_gain_vector(agent, set_point, gain_dictionary)

                        test_run_dictionary['ith_runtime'] = time.time() - ith_starttime 

                        print 'test_run_dictionary = ',test_run_dictionary

                        run_dictionaries.append( test_run_dictionary )
                        
                        #-------------------------------------------------------


total_run_time = time.time() - global_start_time



date_and_time = datetime.now().strftime('%Y-%m-%d__%H.%M.%S')

filepath = '/home/ek/Dropbox/THESIS/python_scripts/brute_force_output_data/bruteforce_output_index'+str(index)+'_' + date_and_time+ '.json'

with open(filepath, 'wb') as fp: 
    json.dump(run_dictionaries, fp)
    fp.close()


'''
for r in run_dictionaries:

    for kee in r.keys():

        if kee != 'variable_dictionary':

            print kee,r[kee]
'''






