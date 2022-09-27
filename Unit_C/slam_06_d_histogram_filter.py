# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Put your code here.
    distributions = []
    mid_index_b = len(b.values) // 2
    for i in xrange(a.start(), a.stop()):
        d = Distribution(b.start() + i, [j*a.value(i) for j in b.values])
        distributions.append(d)
    return Distribution.sum(distributions)
    # return a  # Replace this by your own result.

# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.

def multiply(a, b):
     """Multiply two distributions and return the resulting distribution."""

     # --->>> Put your code here.
     start = max(a.start(), b.start())
     stop = min(a.stop(), b.stop())
     values = []
     for i in range(start, stop):
          values.append(a.value(i) * b.value(i))
     a = Distribution(start, values)
     a.normalize()

     return a  # Modify this to return your result.

if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10) # The robot moves by control[i] step
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
