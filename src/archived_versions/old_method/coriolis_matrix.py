# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 07:10:43 2013

@author: ekemper
"""
from numpy import cos as c
from numpy import sin as s
import numpy as n

def coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz): 
    
    c11 = 0

    c12 = (iyy-izz) * ( thd*c(ph)*s(ph) + psd*c(th)*s(ph)**2 )  + (izz-iyy)*psd*(c(ph)**2)*c(th) - ixx*psd*c(th)

    c13 = (izz-iyy) * psd * c(ph) * s(ph) * c(th)**2

    c21 = (izz-iyy) * ( thd*c(ph)*s(ph) + psd*s(ph)*c(th) ) + (iyy-izz) * psd * (c(ph)**2) * c(th) + ixx * psd * c(th)

    c22 = (izz-iyy)*phd*c(ph)*s(ph)

    c23 = -ixx*psd*s(th)*c(th) + iyy*psd*(s(ph)**2)*s(th)*c(th)

    c31 = (iyy-izz)*phd*(c(th)**2)*s(ph)*c(ph) - ixx*thd*c(th)

    c32 = (izz-iyy)*( thd*c(ph)*s(ph)*s(th) + phd*(s(ph)**2)*c(th) ) + (iyy-izz)*phd*(c(ph)**2)*c(th) + ixx*psd*s(th)*c(th) - iyy*psd*(s(ph)**2)*s(th)*c(th) - izz*psd*(c(ph)**2)*s(th)*c(th)

    c33 = (iyy-izz) * phd *c(ph)*s(ph)*(c(th)**2) - iyy * thd*(s(ph)**2) * c(th)*s(th) - izz*thd*(c(ph)**2)*c(th)*s(th) + ixx*thd*c(th)*s(th)

    return n.array([
                   [c11,c12,c13],
                   [c21,c22,c23],
                   [c31,c32,c33]
    	              ])

#--------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    ph = .1
    th = .1
    ps = .1
    phd = 1
    thd = 1
    psd = 1
    m = 1
 
    l = 0.4
    ixx = (1/12.)*0.5 * m * l**2
    iyy = (1/12.)*0.5 * m * l**2
    izz = (1/12.)* m * l**2

    matr = coriolis_matrix(ph,th,phd,thd,psd,ixx,iyy,izz)
    print 'coriolis_matrix = ',matr
    
    J = n.array([
    [ixx        ,                               0  , -ixx * s(th)                ],
    [0          , iyy*(c(ph)**2) + izz * s(ph)**2  , (iyy-izz)*c(ph)*s(ph)*c(th) ],
    [-ixx*s(th) , (iyy-izz)*c(ph)*s(ph)*c(th)      , ixx*(s(th)**2) + iyy*(s(th)**2)*(c(th)**2) + izz*(c(ph)**2)*(c(th)**2)]    
    ])    
    
    eta = n.array([ph, th, ps])
    etad = n.array([phd, thd, psd])
    etadd = n.array([1,1,1])
    
    LHS = n.dot( J , etadd ) + n.dot( coriolis_matrix(ph,th,phd,thd,psd ,ixx,iyy,izz) , etad )


    print 'LHS = ',LHS

