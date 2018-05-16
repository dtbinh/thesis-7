# -*- coding: utf-8 -*-
"""
Created on Tue Jan  1 18:06:59 2013

@author: ekemper
"""


'''

# -----------------------------------------------this set of equations was taken from 'quadrotor lagrangian.nb'


xdd = - (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta)

ydd = (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta)

zdd = (k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta)

psidd = ((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p))

thedd = ((12*k)/(m*L))*(w2**2 - w4**2)

phidd = ((12*k)/(m*L))*( w3**2 - w1**2)



# ---------------------------------solve each one of these for 0 and combine as the component functions of f(.)




f(w1, w2, w3, w4)=[

(k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd,

(k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd,

(k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd,

((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd,

((12*k)/(m*L))*(w2**2 - w4**2) - thedd,

((12*k)/(m*L))*( w3**2 - w1**2) - phidd,

]

# ----------------------------------------------the function g is the sum of the squares of the components of f


g = sum( fi(x1, x2, x3, x4)**2 ) -> g is essentially the L2 norm of the f(.)

the function g has a minimum at the same point where the function f is zero

g is also convex, so the neg of the gradient of g will point in the direction of 

maximal decrease. Grad(g) is used as the direction at each iteration....

x[i+1] = x[i] - a * Grad( g( x(i) ) )



g(w1, w2, w3, w4) =  sum( [

((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd)**2,

((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd)**2,

((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd)**2,

(((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd)**2,

(((12*k)/(m*L))*(w2**2 - w4**2) - thedd)**2,

(((12*k)/(m*L))*( w3**2 - w1**2) - phidd)**2,

] )


# ----------------------------------------------------------------------now take the gradient of g , holy crap!

# determine each componenet individually:
'''


import numpy as n

def dw1(w1,w2,w3,w4):
    return sum( [
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd) * (2 * (k/m) * 2 * w1 * n.sin(Theta)),
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd) * (2 * (k/m) * 2 * w1 * n.sin(Phi) * n.cos(Theta)) ,
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd) * ( 2 * (k/m)* 2 * w1 * n.cos(Phi)*n.cos(Theta)) ,
    
    (((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd) * ( 2 * (12*Ir)/(m*L**2 * h) ),
    
    (((12*k)/(m*L))*( w3**2 - w1**2 ) - phidd) * ( 2 * ((12*k)/(m*L))* 2 * w1 )
    
    ] )


def dw2(w1,w2,w3,w4): 
    return  sum( [
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd) * (2 * (k/m) * 2 * w2 * n.sin(Theta)),
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd) * (2 * (k/m) * 2 * w2 * n.sin(Phi) * n.cos(Theta)) ,
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd) * ( 2 * (k/m)* 2 * w2 * n.cos(Phi)*n.cos(Theta)) ,
    
    (((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd) * ( 2 * (12*Ir)/(m*L**2 * h) ),
    
    (((12*k)/(m*L))*(w2**2 - w4**2) - thedd) * 2 * (12*k)/(m*L) * w2 * 2,
    
    ] )


def dw3(w1,w2,w3,w4):
    return sum( [

    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd) * (2 * (k/m) * 2 * w3 * n.sin(Theta)),
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd) * (2 * (k/m) * 2 * w3 * n.sin(Phi) * n.cos(Theta)) ,
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd) * ( 2 * (k/m)* 2 * w3 * n.cos(Phi)*n.cos(Theta)) ,
    
    (((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd) * ( 2 * (12*Ir)/(m*L**2 * h) ),
    
    (((12*k)/(m*L))*( w3**2 - w1**2 ) - phidd) * ( 2 * ((12*k)/(m*L))* 2 * w3 ),
    
    ] )


def dw4(w1,w2,w3,w4):
    return sum( [

    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Theta) + xdd) * (2 * (k/m) * 2 * w4 * n.sin(Theta)),
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.sin(Phi)*n.cos(Theta) - ydd) * (2 * (k/m) * 2 * w4 * n.sin(Phi) * n.cos(Theta)) ,
    
    ((k/m)*(w1**2 + w2**2 + w3**2 + w4**2)*n.cos(Phi)*n.cos(Theta) - zdd) * ( 2 * (k/m)* 2 * w4 * n.cos(Phi)*n.cos(Theta)) ,
    
    (((12*Ir)/(m*L**2 * h))* ((w1 + w2 + w3 + w4) - (w1p + w2p + w3p + w4p)) - psidd) * ( 2 * (12*Ir)/(m*L**2 * h) ),
    
    (((12*k)/(m*L))*(w2**2 - w4**2) - thedd) * 2 * (12*k)/(m*L) * w4 * 2,
    
    ] )


def Grad(w1,w2,w3,w4):
    return [ dw1(w1,w2,w3,w4), dw2(w1,w2,w3,w4), dw3(w1,w2,w3,w4), dw4(w1,w2,w3,w4) ]


#----------------------------------------------------------------------------------physical constants


xdd = 0.1
ydd = 0.1
zdd = 0.1

psidd = 0.01
thedd = 0.01
phidd = 0.01

k = 10**-5
m = 1

Theta = 0.1
Phi = 0.1
Psi = 0.1


h = 0.001
Ir = 0.00001
L = 0.4

w1p = 1400
w2p = 1300
w3p = 1200
w4p = 1100

w = [[0,0,0,0],[w1p, w2p, w3p, w4p]]
