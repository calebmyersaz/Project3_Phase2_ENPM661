

import matplotlib.pyplot as plt
import numpy as np
import math
def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) *
        dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D
actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5]]

for action in actions:
    k=cost(0,0,45, action[0],action[1]) # (0,0,45) hypothetical start
print(k[3])
WheelRadius= 2.2#0.038
WheelDistance= 28.7#0.354
fig, ax = plt.subplots()
def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    
    dt = 0.05
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    # Xi, Yi,Thetai: Input point's cooWheelRadiusdinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*WheelRadius* (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*WheelRadius* (UL + UR) * math.sin(Thetan) * dt
        Thetan += (WheelRadius/ WheelDistance) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="blue")
    Tai = Thetai*3.14/180
    phi = Tai+Thetan
    if phi < 0:
        phi = 360 + phi
    elif phi >= 360:
        phi = 360 - phi
    
    XDot = (WheelRadius/2)*(UL+UR)*math.cos(Tai)
    YDot = (WheelRadius/2)*(UL+UR)*math.sin(Tai)
    LinearVel = math.sqrt(XDot**2 + YDot**2)
    Thetan = 180 * (Thetan) / 3.14
    print('r',phi)
    print(Thetan)
    
    ThetaDot = (WheelRadius/WheelDistance)*(UL-UR)
    
            
    # XDot = (WheelRadius/2)*(UL+UR)*math.cos(ThetaRad)
    # YDot = (WheelRadius/2)*(UL+UR)*math.sin(ThetaRad)
    # LinearVel = math.sqrt(XDot**2 + YDot**2)
    
    return Xn, Yn, Thetan, D, ThetaDot,LinearVel
RPM1 = 50
RPM2 = 60


# actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
# for action in actions:
#     X1= plot_curve(0,0,30, action[0],action[1]) # (0,0,45) hypothetical start
    
#     for action in actions:
#         X2=plot_curve(X1[0],X1[1],X1[2], action[0],action[1])
        
# plt.grid()
# # ax.set_aspect('equal')
# plt.xlim(0,600)
# plt.ylim(0,200)
# plt.title('How to plot a vector in matplotlib ?',fontsize=10)
# plt.show()
# plt.close()

WheelRadius = 2.2
WheelDistance = 28.7


# wl = (RPM1*2*np.pi) / 60
# wr = (RPM2*2*np.pi) / 60


# t=10
# # Change in of robot orientation 'theta' in radians, corresponding to 'action'
# # theta = int((RobotRadius / WheelDistance) * (wl - wr) * t)
# theta = (wl - wr) / t
# print("theta: ", theta)
# theta = int(theta)
# # theta = 1
# phi = theta+30
# if phi < 0:
#     phi = 360 + phi
# elif phi >= 360:
#     phi = 360 - phi
# ThetaDot = (WheelRadius/WheelDistance)*(wl-wr)
# XDot = (WheelRadius/2)*(wl+wr)*math.cos(phi)
# YDot = (WheelRadius/2)*(wl+wr)*math.sin(phi)
# LinearVel = math.sqrt(XDot**2 + YDot**2)
# print(ThetaDot)
# print(LinearVel)
# deg = np.pi/180
# y = int(0 + ((WheelRadius/2) * (wl + wr) * np.sin(phi*deg) * t))
# x = int(0 + ((WheelRadius/2) * (wl + wr) * np.cos(phi*deg) * t))
# l = phi
x,y,l,c2c,ThetaDot,LinearVel = plot_curve(0,0,30, RPM1,RPM2) # (0,0,45)
# plt.plot(x,y,'r*')
print(x)
print(y)
print(l)
print(c2c)
print(ThetaDot)
print(LinearVel)
plt.grid()
# ax.set_aspect('equal')
plt.xlim(-100,100)
plt.ylim(-100,100)
plt.title('How to plot a vector in matplotlib ?',fontsize=10)
plt.show()


