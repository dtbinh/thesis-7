
from numpy import sin as s

from numpy import cos as c

# initialize lists of state variables


y1 = [0,0]                      # x                  x[0] = 0
y2 = [0,0]                      # x dot
y3 = [3.14/4., 3.14/4.]         # theta        theta[0] = 3.14/4.
y4 = [0,0]                      # theta dot

h = 0.001
M = 1.
m = 1.
l = 1.
g = -9.8

mu = 0.0001


max_iterations = 30000



for k in range(max_iterations):

    y1.append(h*y2[-1] + y1[-1])

    y2.append( (h/(M + m)) * (m*l*(y4[-1] - y4[-2])/h  * c(y3[-1])  - l**2 * y4[-1]**2 * s(y3[-1]) ) + y2[-1] - mu * y2[-1])
    
    y3.append(  h* y4[-1] + y3[-1]  )

    y4.append(  (h/l) * ( ( (y2[-1] - y2[-2])/h) * c(y3[-1]) - g * s(y3[-1]) ) + y4[-1] - mu*y4[-1])




import matplotlib.pyplot as plt

fig = plt.figure()
x = fig.add_subplot()
plt.plot(range(len(y1)), y1,'b', range(len(y3)) , y3,'g')
plt.title('x = blue, theta = green',fontsize=10)
plt.savefig('unforced_pendulum_on_cart.png')

from os import system

system('gnome-open unforced_pendulum_on_cart.png &')

