import numpy as np
import os
import matplotlib.pyplot as plt
os.system("cls")

time = 4 #time canards are active
initialRollRate = np.radians(500) #degrees per second
Ixx = 0.2487 #kgm^2
numberOfCanards = 3
dt = 0.01 #choose time step
vy0 =100 #initial velocity when canards are actuated
vyt = 30 #final velocity when canards are actuated
vy = np.linspace(vy0,vyt,round(time/dt)) #generate vertical velocity array
trange = np.linspace(0,time,round(time/dt)) #generate time array


omega = [initialRollRate] #initialise roll rate
Minit = 0.1 #initialise value of initial moment

while abs(omega[-1])>1: #loop to find intial required moment
    k = Minit/(vy0**2) #M=k*v^2
    M = []
    omega = [initialRollRate] #reset roll rate
    beta = [np.arctan(initialRollRate*0.13/vy0)]

    for i in range(round(time/dt)-1):
        M.append(k*vy[i]**2) #store moment value
        omegai = omega[i] - (M[i]/Ixx)*dt #calculate next value of roll rate
        omega.append(omegai)
        beta.append(np.arctan(omegai*0.13/vy[i]))

    
    Minit = Minit + float(0.1)

print("Initial value of torque =",Minit)

#plt.plot(trange,np.degrees(beta))
#plt.show()

#Canard design

MDDeff = 0.7 #mach drag divergence of aerofoil
MDD = 0.78 #Max mach number of flight
Cr = 0.13 #root chord
a0 = 6.226315 #airfoil lift curve slope
AR = 1.42 #assume aspect ratio
dxEst = 0.148 #guess for value of moment arm - to be iterated
phi25 = np.arccos((MDDeff/MDD)**(1)) #calculate quarter chord sweep angle
print("Quarter chord sweep angle",np.degrees(phi25))
a3d = (a0*np.cos(phi25))/(np.sqrt(1+((a0*np.cos(phi25))/(np.pi*AR))**2)+((a0*np.cos(phi25))/(np.pi*AR))) #calculate lift curve slope of 3d wing
Cl3d = a3d * np.radians(2) #lift curve slope x angle in radians
Area = (2*Minit/3)/( Cl3d * vy[0]**2 * 1  * dxEst) #from lift equation and modified cl from sweep eq
Ct = np.sqrt(Cr**2-8*Area*np.tan(phi25)/3) #find tip chord given sweep angle and area
print("Adjusted CL =", Cl3d)
print("Tip chord =",Ct)
b = 2*Area/(Cr+Ct) #find span of single canard
print("Sref=",Area)
print("Canard span =",b)
ARactual = (2*b)**2/(2*Area)
print("Aspect ratio =",ARactual)

x = [0,0,b,b,0]
y = [0,Cr,Ct,0,0]
plt.plot(x,y, color='red')
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
ax.fill_between(x,y,color='red')
plt.show()

lamda = Cr/Ct #find sweep ratio
yac = b * (1/3) * (1+2*lamda)/(1+lamda) # find y aero centre
print("Y aerodynamic centre =",yac)
dx = 0.107 + yac #find moment arm
print("Moment arm =",dx)

#Test if goood - not good so ignore this for now
M = []
omegaReal = [initialRollRate]

for i in range(round(time/dt)-1):
    M.append(3*Cl3d*Area*dx*vy[i]**2)
    omegai = omega[i] - (M[i]/Ixx)*dt
    omegaReal.append(omegai)

plt.plot(trange,omegaReal)
plt.show()