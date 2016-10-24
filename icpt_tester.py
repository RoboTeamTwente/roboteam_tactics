import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from math import pi, atan2, degrees, sqrt, sin, cos, tan

#x_range = np.linspace(-10, 10, 500)
#y_range = np.linspace(-10, 10, 500)

bot_x = 3.0
bot_y = 3.0
bot_r = 0.0
bot_vx = 0.7
bot_vy = -0.3
bot_v = sqrt(bot_vx*bot_vx + bot_vy*bot_vy)

ball_x = 6.0
ball_y = 6.0
ball_vx = -0.25
ball_vy = 0.25

accel_b = 1.134
accel_a = 0.86

fig, ax = plt.subplots()

def bot_pos(t):
    return (bot_x + bot_vx * t + .275 * t * t, bot_y + bot_vy * t + .275 * t * t)

def ball_pos(t):
    return (ball_x + ball_vx * t, ball_y + ball_vy * t)

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

def accel(angle):
    x = (accel_a * accel_b) / sqrt(accel_b**2 + (accel_a**2) * (tan(angle)**2))
    y = sqrt(1-(x/accel_a)**2) * accel_b
    return x/2, y/2

def bot_range(t, angle, fix_accel=False):
    ax, ay = accel(angle) if not fix_accel else (.55, .55)
    x = bot_vx * t * cos(angle) - ax * t * t
    y = bot_vy * t * sin(angle) - ay * t * t
    return bot_x + x * cos(angle), bot_y + y * sin(angle)

def update(t):
    ax.cla()
    ball = ball_pos(t)
    
    #bot_range_x = bot_vx * t + .275 * t * t
    #bot_range_y = bot_vy * t + .275 * t * t
    #bot_range = sqrt(bot_range_x*bot_range_x + bot_range_y*bot_range_y)
    #circle = plt.Circle((bot_x, bot_y), bot_range, color='blue', fill=False)
    
    ax.set_xlim((0, 10))
    ax.set_ylim((0, 10))
    ax.plot(ball[0], ball[1], 'rx')
    ax.plot(bot_x, bot_y, 'bx')
    ax.plot([bot_x, ball[0]], [bot_y, ball[1]], 'r')
    #ax.add_artist(circle)
    
    xs = []
    ys = []
    fxs = []
    fys = []
    for theta in frange(0,2*pi,pi/32):
        x, y = bot_range(t, theta)
        fx, fy = bot_range(t, theta, True)
        #print '%f/%f: (%f,%f)' % (t, theta, x, y)
        xs.append(x)
        ys.append(y)
        fxs.append(fx)
        fys.append(fy)
    ax.plot(xs, ys, 'b')
    ax.plot(fxs, fys, 'g')

    print atan2(ball[1] - bot_y, ball[0] - bot_x)

#bot_data = ([], [])
#for t in frange(0, 10, 0.1):
#    (x, y) = bot_pos(t)
#    bot_data[0].append(x)
#    bot_data[1].append(y)
#
#ball_data = ([], [])
#for t in frange(0, 10, 0.1):
#    (x, y) = ball_pos(t)
#    ball_data[0].append(x)
#    ball_data[1].append(y)

#plt.plot(bot_data[0], bot_data[1], 'r')
#plt.plot(ball_data[0], ball_data[1], 'b')

slider = Slider(plt.axes([0, 0, 0.5, 0.05]), 'Time', 0, 10)
slider.on_changed(update)

plt.show()
