# Copyright (c) 2021 John Lauer for modifications
# 
# Original copyrights and code by
# Original URL https://github.com/odriverobotics/ODrive/blob/a8461c40fd6d97991adc9d553db3fdbce4a01dc4/tools/motion_planning/PlanTrap.py
# Copyright (c) 2018 Paul Guénette
# Copyright (c) 2018 Oskar Weigl

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# This algorithm is based on:
# FIR filter-based online jerk-constrained trajectory generation
# https://www.researchgate.net/profile/Richard_Bearee/publication/304358769_FIR_filter-based_online_jerk-controlled_trajectory_generation/links/5770ccdd08ae10de639c0ff7/FIR-filter-based-online-jerk-controlled-trajectory-generation.pdf

def calc_plan_trapezoidal_path_at_start(Xf, Xi, Vi, Vmax, Amax, Dmax):
    dX = Xf - Xi    # Distance to travel
    stop_dist = Vi**2 / (2*Dmax)  # Minimum stopping distance
    dXstop = np.sign(Vi)*stop_dist # Minimum stopping displacement
    s = np.sign(dX - dXstop)   # Sign of coast velocity (if any)
    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    # If we start with a speed faster than cruising, then we need to decel instead of accel
    # aka "double deceleration move" in the paper
    if s*Vi > s*Vr:
        print("Handbrake!")
        Ar = -s*Amax

    # Time to accel/decel to/from Vr (cruise speed)
    Ta = (Vr-Vi)/Ar
    Td = -Vr/Dr

    # Integral of velocity ramps over the full accel and decel times to get
    # minimum displacement required to reach cruising speed
    dXmin = Ta*(Vr+Vi)/2.0 + Td*(Vr)/2.0

    # Are we displacing enough to reach cruising speed?
    if s*dX < s*dXmin:
        print("Short Move:")
        # From paper:
        # Vr = s*math.sqrt((-(Vi**2/Ar)-2*dX)/(1/Dr-1/Ar))
        # Simplified for less divisions:
        Vr = s*math.sqrt((Dr*Vi**2 + 2*Ar*Dr*dX) / (Dr-Ar))
        Ta = max(0, (Vr - Vi)/Ar)
        Td = max(0, -Vr/Dr)
        Tv = 0
    else:
        print("Long move:")
        Tv = (dX - dXmin)/Vr # Coasting time

    Tf = Ta+Tv+Td

    # We only know acceleration (Ar and Dr), so we integrate to create
    # the velocity and position curves
    # we should do this once at the start of the move, not
    # each time thru this step method
    y_Accel = Xi + Vi*Ta + 0.5*Ar*Ta**2

    print("Xi: {:.2f}\tXf: {:.2f}\tVi: {:.2f}".format(Xi, Xf, Vi))
    print("Amax: {:.2f}\tVmax: {:.2f}\tDmax: {:.2f}".format(Amax, Vmax, Dmax))
    print("dX: {:.2f}\tdXst: {:.2f}\tdXmin: {:.2f}".format(dX, dXstop, dXmin))
    print("Ar: {:.2f}\tVr: {:.2f}\tDr: {:.2f}".format(Ar, Vr, Dr))
    print("Ta: {:.2f}\tTv: {:.2f}\tTd: {:.2f}".format(Ta, Tv, Td))
    print("y_Accel: {:.2f}".format(y_Accel))
    print("Tf (full move time): {:.2f}".format(Tf))
    

    return (Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel)

def calc_step_pos_at_arbitrary_time(t, Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel):

    # t is the current time
    # t would be calculated based on microseconds() passed
    # since the start of the current move
    #t = 4.5

    y = None
    yd = None
    ydd = None

    print()
    print("t (seconds): {:.2f}".format(t))

    if t < 0: # Initial conditions
        print("Initial conditions")
        y   = Xi
        yd  = Vi
        ydd = 0
    elif t < Ta: # Acceleration
        print("Acceleration stage")
        y   = Xi + Vi*t + 0.5*Ar*t**2
        yd  = Vi + Ar*t
        ydd = Ar
    elif t < Ta+Tv: # Coasting
        print("Coasting stage")
        y   = y_Accel + Vr*(t-Ta)
        yd  = Vr
        ydd = 0
    elif t < Tf: # Deceleration
        print("Deceleration stage")
        td     = t-Tf
        y   = Xf + 0*td + 0.5*Dr*td**2
        yd  = 0 + Dr*td
        ydd = Dr
    elif t >= Tf: # Final condition
        print("Final condition")
        y   = Xf
        yd  = 0
        ydd = 0
    else:
        raise ValueError("t = {} is outside of considered range".format(t))

    print("y (pos): {:.2f}".format(y))
    print("y_Accel (): {:.2f}".format(y_Accel))
    print("yd (vel): {:.2f}".format(yd))
    print("ydd (acc): {:.2f}".format(ydd))

    return (y, yd, ydd)

if __name__ == '__main__':

    Xf = 30 # Position we're moving to (rads)
    Xi = 0 # Initial position (rads)
    Vi = 0 # Initial velocity (rads/s)
    Vmax = 10 # Velocity max (rads/s)
    Amax = 5 # Acceleration max (rads/s/s)
    Dmax = Amax # Decelerations max (rads/s/s)

    # Plan the path
    (Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel) = calc_plan_trapezoidal_path_at_start(Xf, Xi, Vi, Vmax, Amax, Dmax)
    
    # Example calls each time through loop during move
    # Time = 1 second
    (Y, Yd, Ydd) = calc_step_pos_at_arbitrary_time(1, Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel)
    # Time = 2.5 seconds
    (Y, Yd, Ydd) = calc_step_pos_at_arbitrary_time(2.5, Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel)
    # Time = 4 seconds
    (Y, Yd, Ydd) = calc_step_pos_at_arbitrary_time(4, Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf, y_Accel)