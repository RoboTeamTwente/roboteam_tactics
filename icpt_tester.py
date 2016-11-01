import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from math import pi, atan2, degrees, sqrt, sin, cos, tan

#x_range = np.linspace(-10, 10, 500)
#y_range = np.linspace(-10, 10, 500)

bot_base_vx = 0.0
bot_base_vy = 0.0

bot_x = 3.0
bot_y = 3.0
bot_r = 0.0
bot_vx = bot_base_vx
bot_vy = bot_base_vy
bot_v = sqrt(bot_vx*bot_vx + bot_vy*bot_vy)

ball_x = 6.0
ball_y = 6.0
ball_vx = -0.25
ball_vy = 0.25

accel_b = 0.67
accel_a = 0.43

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
    a = accel_a #accel_a * cos(bot_r) - accel_b * sin(bot_r)
    b = accel_b #accel_a * sin(bot_r) + accel_b * cos(bot_r)
    #x = (a * b) / sqrt(b**2 + (a**2) * (tan(angle)**2))
    #y = sqrt(1-(x/a)**2) * b
    #x = a * cos(angle)
    #y = b * sin(angle)
    x = a * cos(angle) * cos(bot_r) - b * sin(angle) * sin(bot_r)
    y = a * cos(angle) * sin(bot_r) + b * sin(angle) * cos(bot_r)
    #rx = x * cos(angle-bot_r) - y * sin(angle-bot_r)
    #ry = x * sin(angle-bot_r) + y * cos(angle-bot_r)
    #rx = x * cos(bot_r) - y * sin(bot_r)
    #ry = x * sin(bot_r) + y * cos(bot_r)
    print '%f: %f,%f' % (angle, x, y)
    return x,y

def bot_range(t, angle, fix_accel=False):
    ax, ay = accel(angle) if not fix_accel else (.55, .55)
    #rx = ax * cos(bot_r) - ay * sin(bot_r)
    #ry = ax * sin(bot_r) + ay * cos(bot_r)
    #x = bot_x + bot_vx * t * cos(angle) + (ax/2) * t * t
    #y = bot_y + bot_vy * t * sin(angle) + (ay/2) * t * t
    x = bot_vx * t + (ax/2) * t * t
    y = bot_vy * t + (ay/2) * t * t
    rx = bot_x + x #x * cos(angle) - y * sin(angle)
    ry = bot_y + y #x * sin(angle) + y * cos(angle)
    return rx, ry

last_t = 0.0

def update_rot(rot):
    global bot_r, bot_vx, bot_vy
    bot_r = rot
    #bot_vx = bot_base_vx * cos(rot) + bot_base_vy * sin(rot)
    #bot_vy = -bot_base_vx * sin(rot) + bot_base_vy * cos(rot)
    update(last_t)

def update(t):
    global last_t
    last_t = t
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
    for theta in frange(0,2*pi,pi/64):
        x, y = bot_range(t, theta)
        fx, fy = accel(theta)
        #print '%f/%f: (%f,%f)' % (t, theta, x, y)
        #xs.append((x-bot_x) * cos(bot_r) - (y-bot_y) * sin(bot_r) + bot_x)
        #ys.append((x-bot_x) * sin(bot_r) + (y-bot_y) * cos(bot_r) + bot_y)
        #fxs.append((fx-bot_x) * cos(bot_r) - (fy-bot_y) * sin(bot_r) + bot_x)
        #fys.append((fx-bot_x) * sin(bot_r) + (fy-bot_y) * cos(bot_r) + bot_y)
        xs.append(x)
        ys.append(y)
        fxs.append(fx+bot_x)
        fys.append(fy+bot_y)

    ax.plot(xs, ys, 'b')
    ax.plot(fxs, fys, 'g')

   # print atan2(ball[1] - bot_y, ball[0] - bot_x)

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

slider = Slider(plt.axes([0, 0, 0.5, 0.05]), 'Time', 0, 5)
slider.on_changed(update)

rslider = Slider(plt.axes([0.5, 0, 0.5, 0.05]), 'Rotation', 0, 2*pi)
rslider.on_changed(update_rot)

plt.show()
