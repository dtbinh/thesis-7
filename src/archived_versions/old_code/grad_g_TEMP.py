import numpy as n

'''
as an example let 

f = w1**2 + w2**2


g = (w1**2 + w2**2)

'''


def dw1(w1,w2):
    2 * (w1**2 + w2**2 ) 
    
    
    
    
def Grad(w1,w2):
    return [ dw1(w1,w2), dw2(w1,w2) ]
