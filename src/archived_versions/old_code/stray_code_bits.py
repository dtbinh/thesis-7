# print 'the final position is:'
# print [x[-1],y[-1],z[-1]]
# print 'the relative steady state error in position is:'
# print [abs(x[-1]-xd)/xd,abs(y[-1]-yd)/yd,abs(z[-1]-zd)/zd]
# print 'the steady state error in pitch, roll and yaw angles:'
# print [Th[-1],Ph[-1],Ps[-1]]


# for i in range(len(fx)):# this is a temporary fix to get rid of the few giant numbers that appear in the forces
	# if abs(fx[i]) > 100:			#need figure out whats going on here
		# fx[i] = 0
# for i in range(len(fy)):
	# if abs(fy[i]) > 100:
		# fy[i] = 0
# for i in range(len(fz)):
	# if abs(fz[i]) > 100:
		# fz[i] = 0




print 'time series length =',len(timeSeries),'\n'
print 'ax length =',len(ax),'\n'
print 'ay length =',len(ay),'\n'
print 'az length =',len(az),'\n'

