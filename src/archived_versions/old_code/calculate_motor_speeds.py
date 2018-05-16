import numpy

i = 0.1
ir = 0.0001
L = 0.5
a = 1
psidd = 0.1
thedd = 0.1
phidd = 0.1
the = 0.1
m = 1
xdd = 1
w = [[950,910,960,900],[1040,1100,1200,1400]]
h=0.001
def f(w,k):
	#global i,ir,L,a,psidd,thedd,phidd,the,m,xdd
	w1 = w[k][0]
	w2 = w[k][1]
	w3 = w[k][2]
	w4 = w[k][3]
	
	wPrev = w[k-1][0] + w[k-1][1] + w[k-1][2] + w[k-1][3] 

	return [
	[w2**2 - w4**2 - (i*phidd/(a*L))],
	[w3**2 - w1*2 - (i*thedd/(a*L))],
	[-(w1 + w2 + w3 + w4) + (2*i*psidd*h/ir) + wPrev],
	[(w1**2 + w2**2 + w3**2 + w4**2) + (m*xdd/(a*the))]
	]
# -------------------w is the list of vectors of motor speeds w[k] = [w1,w2,w3,w4] @ time step k
def jInv(w,k):
	w1 = w[k][0]
	w2 = w[k][1]
	w3 = w[k][2]
	w4 = w[k][3]
	
	oneOverTheDeterminant = (1/(16*w1*w2*w3 - 16*w1*w2*w4 + 16*w1*w3*w4 - 16*w2*w3*w4))
	print oneOverTheDeterminant
	jInvTemp = [
	[-4*w2*w3 + 4*w3*w4         , -4*w2*w3 + 8*w2*w4 - 4*w3*w4, 16*w2*w3*w4  , 4*w2*w3 + 4*w3*w4 ],
	[8*w1*w3 - 4*w1*w4 - 4*w3*w4, -4*w1*w4 + 4*w3*w4          , -16*w1*w3*w4 , -4*w1*w4 - 4*w3*w4],
	[-4*w1*w2 + 4*w1*w4         , 4*w1*w2 + 4*w1*w4 - 8*w2*w4 , 16*w1*w2*w4  , 4*w1*w2 + 4*w1*w4 ],
	[4*w1*w2 - 8*w1*w3 + 4*w2*w3, -4*w1*w2 + 4*w2*w3          , -16*w1*w2*w3 , -4*w1*w2 - 4*w2*w3]
	]
	print jInvTemp
	return oneOverTheDeterminant*jInvTemp#--- return the matrix Inverse(jacobian)
	
def solve_for_motor_speeds(w):
	x = [[10,10,10,10]]  #an initial value for the motor speed vector	
	for k in range(100):
		x.append(x[k] - numpy.dot(jInv(w,k),f(w,k)))
		if numpy.linalg.norm(f(w,k)) < 10**-4:break
	print x
	return x[-1]

if __name__ == '__main__':
	i = 0.1
	ir = 0.0001
	L = 0.5
	a = 1
	psidd = 0.1
	thedd = 0.1
	phidd = 0.1
	the = 0.1
	m = 1
	xdd = 1
	w = [[950,910,960,900],[1040,1100,1200,1400]]
	
	solve_for_motor_speeds(w)
	