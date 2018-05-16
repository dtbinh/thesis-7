import numpy as n
#from reduced_system import *



'''
the function g is the sum of the squares of the components of f.  g = sum( fi(x1, x2, x3, x4)**2 ) -> g is the L2 norm of f(.). the function g has a minimum at the same point where the function f is zero. g is also convex, so the neg of the gradient of g will point in the direction of maximal decrease. Grad(g) is used as the direction at each iteration...

x[i+1] = x[i] - a * Grad( g( x(i) ) )

# determine each componenet of the grad of G individually: dw1 is the derivative of the function g with respect to w1...
'''

#The component functions of the gradient of G are also defined in the file 'reduced_system.py"


def Grad(h,w1,w2,w3,w4, Theta,Phi,Psi, xdd,ydd,zdd, psidd,thedd,phidd, w1p,w2p,w3p,w4p):

    #----------------------------------------------------------------------------------physical constants
    k = 0.004 #
    m = 2
    #h = 0.001
    Ir = 0.0001 #the moment of inertia of the rotor
    L = 0.4    
    g = - 9.81    
    
    return [
    
    #-----------------------------dw1
    sum( [
    
    2*(- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd )*(-(2*k*w1/m))*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)-g - zdd ) * (2*k*w1/m) * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir / (m*L**2 * h)) * ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir / (m*L**2 * h)),

    2* ( ( 12*k / (m*L) ) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * ( 24*k*w1 /(m*L))  
    
    ] ),
    
    #-----------------------------dw2
    sum( [
    
    2*(- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd )*(-(2*k*w2/m))*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)-g - zdd ) * (2*k*w2/m) * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir / (m*L**2 * h)) * ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir / (m*L**2 * h)),

    2* ( ( 12*k / (m*L) ) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * ( 24*k*w2 /(m*L))  
    
    ] ),
    
    #-----------------------------dw3
    sum( [
    
    2*(- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd )*(-(2*k*w3/m))*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)-g - zdd ) * (2*k*w3/m) * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir / (m*L**2 * h)) * ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir / (m*L**2 * h)),

    2* ( ( 12*k / (m*L) ) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * ( 24*k*w3 /(m*L))  
    
    ] ),

    #-----------------------------dw4
    sum( [
    
    2*(- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd )*(-(2*k*w4/m))*(n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)-g - zdd ) * (2*k*w4/m) * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir / (m*L**2 * h)) * ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir / (m*L**2 * h)),

    2* ( ( 12*k / (m*L) ) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * ( 24*k*w4 /(m*L))  
    
    ] )

    ]#close return list


'''
The function 'solve_for_motor_speeds()' solves for the appropriate motor speeds which correspond to the accelerations prescribed by the control law (xdd,ydd,zdd, psidd,thedd,phidd) . these are the accelerations that the quadrotor must undergo in order that the system stablize at the given setpoints. the function also takes as argument the angular positions (Theta,Phi,Psi ) and the motor speeds from the last iteration.
'''



def solve_for_motor_speeds(h, Theta,Phi,Psi, xdd,ydd,zdd, psidd,thedd,phidd, w1p,w2p,w3p,w4p):

    w = [[w1p, w2p, w3p, w4p],[w1p, w2p, w3p, w4p]]
    grad_norm_list = [0]
    kth_stopping_condition = None
    end_iteration_number = 0
    tol =0.1
    max_iterations = 1000 

    #----------------------------------------------------------------------------------begin steepest descent loop
    for i in range(max_iterations):    
        #print '\n#------------------------------------------------------------------------'
        #print 'i =',i    
        #print '\nw[-1] =', w[-1]
        grad_at_wi  =  n.array( 
                              Grad(
 				               h,
                                   w[-1][0], w[-1][1], w[-1][2], w[-1][3], 
                                   Theta,Phi,Psi, 
                                   xdd,ydd,zdd, 
                                   psidd,thedd,phidd, 
                                   w1p,w2p,w3p,w4p
                                  )
                              )  
        #print 'grad_at_wi =', grad_at_wi,'\n'    
        norm_grad_G = n.linalg.norm(grad_at_wi)        
        #print '\nnorm_grad_G = ', norm_grad_G
        #print '\ngrad_at_wi/norm_grad_G = ',grad_at_wi/norm_grad_G
	
        #------------------------------------------------------------tests for convergence / divergence
        grad_norm_list.append(norm_grad_G)

        if abs( grad_norm_list[-1] - grad_norm_list[-2] ) < tol and i > 2 :
            #print '\nSTOPPING CONDITION 1) iterates of the norm of grad(g) converged to within tol :'
            #print '\nabs( grad_norm_list[-1] - grad_norm_list[-2] ) < tol '
            kth_stopping_condition = 1
            end_iteration_number = i            
            #print '\ni =',i
            break
        #--------------------------
        # need a test to detect steady state osclillations in the values of the iterates of w
        # test if the average of the last two iteriates is within tol of the average of the last 10 iterates

        ave_of_20_w = sum( n.array( w[-10:] ) ) * 0.1
        ave_of_2_w = sum( n.array( w[-2:] ) ) * 0.5

        if all( abs( ave_of_20_w - ave_of_2_w) ) < 10**-3:
            #print 'STOPPING CONDITION 2) small steadystate osc detected in the iterates of w'
            #print '\ni =',i
            kth_stopping_condition = 2
            end_iteration_number = i
            break
        
        #--------------------------	
        if i >= max_iterations - 1:
            #print '\nSTOPPING CONDITION 3: the sequence did NOT converge, max_iterations reached!\n'
            kth_stopping_condition = 3
            end_iteration_number = i            
            break
        #---------------------------------------------------------------------------    


        w.append(  n.array(w[-1]) - 0.1 * grad_at_wi/norm_grad_G )


    #print 'EXITING LOOP----------------------------------\n'
    #print 'norm_grad_G = ',  norm_grad_G

    return [w,grad_norm_list,kth_stopping_condition,end_iteration_number]
#-----------------------------------------------------------------------------------------end  solve_for_motor_speeds()



#-----------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------
if __name__ == '__main__':
 
    import pprint
    pp = pprint.PrettyPrinter(indent=2)
    from pylab import figure, show  

    Theta = 0.1
    Phi = 0.1
    Psi = 0.1

    xdd = 0.01
    ydd = -0.01
    zdd = 0.01
    psidd = 0.01
    thedd = 0.01
    phidd = 0.01

    w1p = 1000
    w2p = 1100
    w3p = 1200
    w4p = 1300
 
    h = 0.01    

    solution = solve_for_motor_speeds(h, Theta,Phi,Psi, xdd,ydd,zdd, psidd,thedd,phidd, w1p,w2p,w3p,w4p  )

    w = solution[0]
    grad_norm_list = solution[1]
    print '\nstopping_condition =',solution[2] 

    if len(w)>10:
	 
        print '\nw = '
        pp.pprint(w[:5])
        print '...'
        pp.pprint(w[-10:])      

        print '\ngrad_norm_list = '
        pp.pprint(grad_norm_list[:5])
        print '...'
        pp.pprint(grad_norm_list[-10:])

    else: 
        print '\nw = ',pp.pprint(w)
        print '\ngrad_norm_list = '
        pp.pprint(grad_norm_list)

    w1_plotseries = [w[m][0] for m in range(len(w))]
    w2_plotseries = [w[m][1] for m in range(len(w))]
    w3_plotseries = [w[m][2] for m in range(len(w))]    
    w4_plotseries = [w[m][3] for m in range(len(w))]    
    
    
    r = range(len(w))
    fig = figure()
    ax = fig.add_subplot(111)
    ax.plot(r,w1_plotseries ,  r, w2_plotseries, r, w3_plotseries, r, w4_plotseries)
    #title('x',fontsize=10)
    show()

















'''

these were old tests for convergence

    if norm_grad_G < tol:
        print '\nthe sequence converged\n'
        break

    if norm_grad_G > 10**15:
        print '\nthe sequence is diverging\n'
        break
------------------------------
     
        # could use a simalar test for the elements of grad_norm_list to detect steady state oscillations

        ave_of_20_grad_norm = sum( n.array( grad_norm_list[-20:] ) ) * 0.05
        ave_of_2_grad_norm = sum( n.array( grad_norm_list[-2:] ) ) * 0.5

        if abs( ave_of_20_grad_norm - ave_of_2_grad_norm) < tol:
            print '\nsmall steadystate osc detected in the iterates of norm of gradient of g'
            print '\ni =',i
            break

    
'''
