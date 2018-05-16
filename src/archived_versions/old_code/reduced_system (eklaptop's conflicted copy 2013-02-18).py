# -*- coding: utf-8 -*-
"""
Created on Tue Jan  1 18:06:59 2013

@author: ekemper
"""




# -----------------------------------------------this set of equations was taken from 'quadrotor lagrangian.nb'


xdd = - (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta)

ydd = (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta)

zdd = (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)

psidd = ((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p))

thedd = ((12*k)/(m*L))*(w2**2 - w4**2)

phidd = ((12*k)/(m*L))*( w3**2 - w1**2)

# -----------------------------------------------the reduced set


xdd - ydd = (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta))

zdd = (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)

psidd = ((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p))

thedd - phidd = ((12*k)/(m*L)) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) )



# ---------------------------------solve each one of these for 0 and combine as the component functions of f(.)




f(w1, w2, w3, w4)=[
(- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd,

(k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd,

((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd,

((12*k)/(m*L)) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd,

]

# ----------------------------------------------the function g is the sum of the squares of the components of f


g = sum( fi(x1, x2, x3, x4)**2 ) -> g is essentially the L2 norm of the f(.)

the function g has a minimum at the same point where the function f is zero

g is also convex, so the neg of the gradient of g will point in the direction of 

maximal decrease. Grad(g) is used as the direction at each iteration....

x[i+1] = x[i] - a * Grad( g( x(i) ) )



g(w1, w2, w3, w4) =  sum( [

( (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd )**2,

( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd )**2,

( (12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd )**2,

( ((12*k)/(m*L)) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd )**2

] )


# ----------------------------------------------------------------------now take the gradient of g , holy crap!

# determine each componenet individually: dw1 is the derivative of the function g with respect to w1...



def dw1(w1,w2,w3,w4):
    return sum( [
    
    2* ( (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd ) * (- (k/m)*(2*w1) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd ) * (k/m) * 2*w1  * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir)/(m*L**2 * h)* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir)/(m*L**2 * h),

    2* ( ((12*k)/(m*L)) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * (12*k)/(m*L) * ( 2 * w1 )  
    
    ] )


def dw2(w1,w2,w3,w4): 
    return  sum( [

    2* ( (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd ) * (- (k/m)*(2*w2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd ) * (k/m) * 2*w2  * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir)/(m*L**2 * h)* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir)/(m*L**2 * h),

    2* ( ((12*k)/(m*L)) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * (12*k)/(m*L) * ( 2 * w2 )  
   
    
    ] )


def dw3(w1,w2,w3,w4):
    return sum( [

    2* ( (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd ) * (- (k/m)*(2*w3) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd ) * (k/m) * 2*w3  * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir)/(m*L**2 * h) * ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir)/(m*L**2 * h),

    2* ( (12*k)/(m*L) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) *  (12*k)/(m*L) * ( - 2 * w3 )
    
    ] )


def dw4(w1,w2,w3,w4):
    return sum( [

    2* ( (- (k/m)*(w1**2 + w2**2 + w3**2 + w4**2) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)) - xdd + ydd ) * (- (k/m)*(2*w4) ) * (n.sin(Theta) + n.sin(Phi)*n.cos(Theta)), 

    2* ( (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd ) * (k/m) * 2*w4  * n.cos(Phi) * n.cos(Theta),

    2* ( (12*Ir)/(m*L**2 * h)* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd ) * (12*Ir)/(m*L**2 * h),

    2* ( (12*k)/(m*L) * ( (w2**2 - w4**2) - ( w3**2 - w1**2) ) - thedd + phidd ) * ( (12*k)/(m*L)) * ( - 2 * w4 )
    
    ] )


def Grad(w1,w2,w3,w4):
    return [ dw1(w1,w2,w3,w4), dw2(w1,w2,w3,w4), dw3(w1,w2,w3,w4), dw4(w1,w2,w3,w4) ]



