from numpy import cos as c
from numpy import sin as s
import numpy as n
from coriolis_matrix import coriolis_matrix 
'''
The system of equations that we start with:

the angular equations are:
    
 n.dot( J , etadd ) + n.dot( Coriolis_matrix(eta,etad) , etad )
 = [
   l*k*( -w2**2 + w4**2 ),
   l*k*( -w1**2 + w3**2 ),
   b*(w1**2 + w2**2 + w3**2 + w4**2)
   ] 

The LHS of this expression is evaluated first to to give a three element vector:

   lhs_phi    =  l*k*( -w2**2 + w4**2 )
   lhs_theta  =  l*k*( -w1**2 + w3**2 )
   lhs_psi    =  b*(w1**2 + w2**2 + w3**2 + w4**2)

And the linear equations:

    m * xdd = -k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(psi) * s(theta) * c(phi) + s(psi) * s(phi))
    m * ydd =  k * (w1**2 + w2**2 + w3**2 + w4**2) * (s(psi) * s(theta) * c(phi) - c(psi) *s(phi))
    m * zdd + m * g = (w1**2 + w2**2 + w3**2 + w4**2) * c(phi) * c(theta)


Take the difference between the first two and last two equations to form a system with four equations and four unknowns:


   lhs_phi - lhs_theta   =  l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 )

   lhs_psi    =  b*(w1**2 + w2**2 + w3**2 + w4**2)

   m * xdd = -k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(psi) * s(theta) * c(phi) + s(psi) * s(phi))

   m * ( ydd - zdd - g ) = ( k * (s(psi) * s(theta) * c(phi) - c(psi) *s(phi)) - c(phi) * c(theta) ) * (w1**2 + w2**2 + w3**2 + w4**2)


These form the vector function f:R4 -> R6

f(w1, w2, w3, w4) = 
[
   l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta,    

   b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi,

   -k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(psi) * s(theta) * c(phi) + s(psi) * s(phi)) - m * xdd ,

    ( k * (s(psi) * s(theta) * c(phi) - c(psi) *s(phi)) - c(phi) * c(theta) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd - g )
]

#-----------------------------------------------------------------------------------------------------------------

G(w1, w2, w3, w4) = 

sum([

   (l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta)**2  ,  

   (b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi)**2 ,

   (-k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(psi) * s(theta) * c(phi) + s(psi) * s(phi)) - m * xdd)**2 ,

   (( k * (s(psi) * s(theta) * c(phi) - c(psi) *s(phi)) - c(phi) * c(theta) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd - g ))**2 

   ]

the function G is the sum of the squares of the components of f.  G = sum( fi(x1, x2, x3, x4)**2 ). the function g has a minimum at the same point where the function f is zero. G is also convex, so the neg of the gradient of G will point in the direction of maximal decrease. Grad(G) is used as the direction at each iteration...

x[i+1] = x[i] - a * Grad( G( x(i) ) )

# determine each componenet of the grad of G individually: dw1 is the derivative of the function G with respect to w1...
'''

def Grad(w1,w2,w3,w4, th,ph,ps, phd,thd,psd, phdd,thdd,psdd, xdd,ydd,zdd):

    #----------------------------------------------------------------------------------physical constants
    k = 10**-6 #
    m = 2
    l = 0.4    
    g = - 9.81    
    b = 10**-7
   
    beta = (1/12.0)*m*l**2    

    ixx = 0.5*beta
    iyy = 0.5*beta
    izz = beta    

    J = n.array([
    [ixx        ,                               0  , -ixx * s(th)                ],
    [0          , iyy*(c(ph)**2) + izz * s(ph)**2  , (iyy-izz)*c(ph)*s(ph)*c(th) ],
    [-ixx*s(th) , (iyy-izz)*c(ph)*s(ph)*c(th)      , ixx*(s(th)**2) + iyy*(s(th)**2)*(c(th)**2) + izz*(c(ph)**2)*(c(th)**2)]    
    ])    
    
    #eta = n.array([ph, th, ps])
    etad = n.array([phd, thd, psd])
    etadd = n.array([phdd,thdd,psdd])
    
    LHS = n.dot( J , etadd ) + n.dot( coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz) , etad )


    lhs_phi   = LHS[0]
    lhs_theta = LHS[1]
    lhs_psi   = LHS[2] 


    return [
    
    #-----------------------------dw1
    sum([

       2 * (l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta) * l * k * 2 * w1,

       2 * (b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi) * b * 2 * w1,

       2 * (-k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(ps) * s(th) * c(ph) + s(ps) * s(ph)) - m * xdd ) * -k * (c(ps) * s(th) * c(ph) + s(ps) * s(ph))  * 2 * w1,

       2 * (( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd + g )) * ( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) )  * 2 * w1
    
       ]),
    
    #-----------------------------dw2
    sum([

       2 * (l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta) * l * k * 2 * -w2,

       2 * (b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi) * b * 2 * w2,

       2 * (-k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(ps) * s(th) * c(ph) + s(ps) * s(ph)) - m * xdd ) * -k * (c(ps) * s(th) * c(ph) + s(ps) * s(ph))  * 2 * w2,

	  2 * (( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd + g )) * ( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) )  * 2 * w2
       
       ] ),
    
    #-----------------------------dw3
    sum( [

       2 * (l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta) * l * k * 2 * -w3,

       2 * (b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi) * b * 2 * w3,

       2 * (-k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(ps) * s(th) * c(ph) + s(ps) * s(ph)) - m * xdd ) * -k * (c(ps) * s(th) * c(ph) + s(ps) * s(ph))  * 2 * w3,

	  2 * (( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd + g )) * ( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) )  * 2 * w3
       
       ] ),

    #-----------------------------dw4
    sum([

       2 * (l * k * ( -w2**2 + w4**2 + w1**2 - w3**2 ) - lhs_phi + lhs_theta) * l * k * 2 * w4,

       2 * (b*(w1**2 + w2**2 + w3**2 + w4**2) - lhs_psi) * b * 2 * w4,

       2 * (-k * (w1**2 + w2**2 + w3**2 + w4**2) * (c(ps) * s(th) * c(ph) + s(ps) * s(ph)) - m * xdd ) * -k * (c(ps) * s(th) * c(ph) + s(ps) * s(ph))  * 2 * w4,

	 2 * (( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) ) * (w1**2 + w2**2 + w3**2 + w4**2)  - m * ( ydd - zdd + g )) * ( k * (s(ps) * s(th) * c(ph) - c(ps) *s(ph)) - c(ph) * c(th) )  * 2 * w4

       ])

    ]#close return list


'''
The function 'solve_for_motor_speeds()' solves for the appropriate motor speeds which correspond to the accelerations prescribed by the control law (xdd,ydd,zdd, psidd,thedd,phidd) . these are the accelerations that the quadrotor must undergo in order that the system stablize at the given setpoints. the function also takes as argument the angular positions (Theta,Phi,Psi ) and the motor speeds from the last iteration.
'''



def solve_for_motor_speeds(  ph,th,ps,  phd,thd,psd,  phdd,thdd,psdd,  xdd,ydd,zdd,  w1p, w2p, w3p, w4p):

    w = [[w1p, w2p, w3p, w4p],[w1p, w2p, w3p, w4p]]
    grad_norm_list = [0]
    kth_stopping_condition = None
    end_iteration_number = 0
    tol =1
    max_iterations = 2000 

    #----------------------------------------------------------------------------------begin steepest descent loop
    for i in range(max_iterations):    
        #print '\n#------------------------------------------------------------------------'
        #print 'i =',i    
        #print '\nw[-1] =', w[-1]
        grad_at_wi  =  n.array( 
                              Grad(w[-1][0],w[-1][1],w[-1][2],w[-1][3], th,ph,ps, phd,thd,psd, phdd,thdd,psdd, xdd,ydd,zdd)
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

        if all( abs( ave_of_20_w - ave_of_2_w) ) < 10**-2:
            #print 'STOPPING CONDITION 2) small steadystate osc detected in the iterates of w'
            #print '\ni =',i
            kth_stopping_condition = 2
            end_iteration_number = i
            break
        

        #--------------------------
        # need a test to detect steady state osclillations in the values of the norm of the gradient
        # test if the average of the last two iteriates is within tol of the average of the last 10 iterates

        ave_of_20_ng = sum( n.array( grad_norm_list[-10:] ) ) * 0.1
        ave_of_2_ng  = sum( n.array( grad_norm_list[-2:]  ) ) * 0.5

        if abs( ave_of_20_ng - ave_of_2_ng ) < 10**-2:
            #print 'STOPPING CONDITION 2) small steadystate osc detected in the values of the norm of g'
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

        if i%1000 == 0: print 'i = ',i

        w.append(  n.array(w[-1]) - 2 * grad_at_wi/norm_grad_G )


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

    ph = 1
    th = 1
    ps = 1

    phd = .1    
    thd = .01
    psd = .01
    
    phdd = .01
    thdd = .01
    psdd = .01
    
    xdd = .01
    ydd = .01
    zdd = .01

    w1p = 1000
    w2p = 900
    w3p = 800
    w4p = 700   

    solution = solve_for_motor_speeds( ph,th,ps,  phd,thd,psd,  phdd,thdd,psdd,  xdd,ydd,zdd,  w1p, w2p, w3p, w4p )

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
