
#-------------------------------------------------------------------------------

def get_good_runs(tuning_dictionaries):

    good_runs = []

    for i in tuning_dictionaries:

        if ('nan' not in i.values() ) and ('err' not in i.values() ) and (None not in i.values() ):

            good_runs.append(i)


    min_total_thrust = 1000000  

    optimal_run = None

    for r in good_runs:

        if r['total_thrust'] < min_total_thrust:

            optimal_run = r

            min_total_thrust = r['total_thrust']
 
    print '\n\noptimal run:',optimal_run

    return [good_runs , optimal_run]

   '''
    The following are a list of the gains that are known to work 

    a.kpx = 10     # ----------------------PID proportional gain values
    a.kpy = 10
    a.kpz = 40

    a.kdx = 10         #----------------- -----PID derivative gain values
    a.kdy = 10
    a.kdz = 40

    a.kix = .5
    a.kiy = .5
    a.kiz = 15
    '''




    '''
    if plots == True:

        #fig1_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig1_zn_module.png'

        #fig2_file_path = '/home/ek/Dropbox/THESIS/python_scripts/fig2_zn_module.png'

        agent.plot_results()#False, True, fig1_file_path, fig2_file_path)

        agent.print_dump(10)
    '''


