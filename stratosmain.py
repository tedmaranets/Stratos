import numpy as np
import math
import matplotlib.pyplot as pyp

class Felix(object):

    def __init__(self, mass, y, v):
        self.mass = mass
        self.y = y
        self.vel = v

    def euler(self, force, dt):
        self.y = self.y + self.vel*dt
        self.vel = self.vel + (force/self.mass)*dt


# Dependents
gy = lambda s: (-3*10**(-6))*s + 9.8064 # m/s^2 # accel of grav as a function of altitude
rho = lambda s: 1.4213*math.exp(-(10**(-4))*s) # kg/m^3 # air density as a function of altitude

# Constants
gc = 9.81 # m/s^2
mass = 104.78 # kg
y0 = 38969.4 # m # initial altitude
vel0 = 0 # m/s # initial velocity
Cd = 0.7 # drag coefficient of felix (assuming he's falling head first)
A = 0.4 # m^2 surface area of felix ("")
dt = 0.2 # 0.2 s

# Functions
def constgravaccel():
    # Object
    felix = Felix(mass, y0, vel0)
    # Data
    Y = [y0]
    V = [vel0]
    t = [0]
    # Free fall loop
    # 2,567 m is the altiude when Felix deployed his main parachute
    while felix.y > 2567:
        # Position independent grav force
        F = -mass * gc + 0.5 * Cd * A * rho(felix.y) * (felix.vel) ** 2
        felix.euler(F, dt)
        Y.append(felix.y)
        V.append(felix.vel)
        t.append(t[-1] + dt)
    return [t,Y,V]

def vargravaccel():
    # Object
    felix = Felix(mass, y0, vel0)
    # Data
    Y = [y0]
    V = [vel0]
    t = [0]
    # Free fall loop
    # 2,567 m is the altiude when Felix deployed his main parachute
    while felix.y > 2567:
        # Position dependent grav force
        F = -mass*gy(felix.y) + 0.5*Cd*A*rho(felix.y)*(felix.vel)**2
        felix.euler(F, dt)
        Y.append(felix.y)
        V.append(felix.vel)
        t.append(t[-1] + dt)
    return [t, Y, V]

def main():
    # Get data sets
    [t1, Y1, V1] = constgravaccel()
    [t2, Y2, V2] = vargravaccel()
    # Convert to Arrays
    Y1_data = np.array(Y1)
    V1_data = np.array(V1)
    t1_data = np.array(t1)
    Y2_data = np.array(Y2)
    V2_data = np.array(V2)
    t2_data = np.array(t2)
    # Save data
    data1 = np.stack((t1_data,Y1_data,V1_data), axis=1)
    data2 = np.stack((t2_data, Y2_data, V2_data), axis=1)
    np.savetxt("posindep.csv",data1,delimiter=",")
    np.savetxt("posdep.csv",data2,delimiter=",")

    # Print some relevant info
    maxvel_i = np.argmin(V1_data)
    print("\nCalculated maximum vertical speed with constant gravitational acceleration is: " + str(round(V1_data[maxvel_i] * 0.001 * 3600,2)) + " kmh")
    print("The nominal value is: -1357.6 kmh")
    print("The time at calculated max velocity is t = " + str(round(t1_data[maxvel_i],2) )+ " sec")
    print("The nominal value is: 50 sec")
    maxvel_i = np.argmin(V2_data)
    print("\nCalculated maximum vertical speed with altitude dependent gravitational acceleration is: " + str(round(V2_data[maxvel_i] * 0.001 * 3600, 2)) + " kmh")
    print("The nominal value is: -1357.6 kmh")
    print("The time at calculated max velocity is t = " + str(round(t2_data[maxvel_i], 2)) + " sec")
    print("The nominal value is: 50 sec")

    # Plot Data
    # Altitude
    pyp.plot(t1_data, Y1_data, color='r', ls='-', lw=3)
    pyp.xlabel('time (s)')
    pyp.ylabel('altitude (m)')
    pyp.title('Altitude vs. Time (Constant Gravitational Acceleration)')
    pyp.show()
    pyp.plot(t2_data, Y2_data, color='m', ls='-', lw=3)
    pyp.xlabel('time (s)')
    pyp.ylabel('altitude (m)')
    pyp.title('Altitude vs. Time (Position Dependent Gravitational Acceleration)')
    pyp.show()
    # Velocity
    pyp.plot(t1_data, V1_data, color='b', ls='-', lw=3)
    pyp.xlabel('time (s)')
    pyp.ylabel('velocity (m/s)')
    pyp.title('Velocity vs. Time (Constant Gravitational Acceleration)')
    pyp.show()
    pyp.plot(t2_data, V2_data, color='g', ls='-', lw=3)
    pyp.xlabel('time (s)')
    pyp.ylabel('velocity (m/s)')
    pyp.title('Velocity vs. Time (Position Dependent Gravitational Acceleration)')
    pyp.show()

###
main()


