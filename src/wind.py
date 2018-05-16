
import random as r
import numpy


# this function produces a time-series for each of the x,y, and z directions 
def wind_vector_time_series( max_gust , length ):
    wind_x = [0]
    wind_y = [0]
    wind_z = [0]    

    for i in range(length):

        # this is a hack to keep the values within a specified range
        if abs(wind_x[-1]) >= max_gust:
            wind_x.append( numpy.sign(wind_x[-1]) * 0.5 * max_gust)
        else:
            wind_x.append( wind_x[-1] + r.uniform(-1,1)  )


        if abs(wind_y[-1]) >= max_gust:
            wind_y.append( numpy.sign(wind_y[-1]) * 0.5 * max_gust)
        else:
            wind_y.append( wind_y[-1] + r.uniform(-1,1)  )


        if abs(wind_z[-1]) >= max_gust:
            wind_z.append( numpy.sign(wind_z[-1]) * 0.5 * max_gust)
        else:
            wind_z.append( wind_z[-1] + r.uniform(-1,1)  )

    return [ wind_x , wind_y , wind_z ]
#---------------------------------------------------------------------------------------------    
if __name__ == "__main__":

    length = 1000
    max_gust = 10

    wind_data = wind_vector_time_series( max_gust , length )

    wind_x = wind_data[0]
    wind_y = wind_data[1]
    wind_z = wind_data[2]
    

    timeSeries = range( length + 1 )

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from pylab import title   

    fig = plt.figure()
    ax = fig.add_subplot(111,)
    ax.plot(timeSeries, wind_x, '-b' , timeSeries , wind_y ,'-r', timeSeries , wind_z,'-g')

    plt.show()
