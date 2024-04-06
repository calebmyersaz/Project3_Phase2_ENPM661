

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

fig, ax = plt.subplots()
def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 14.35#0.038
    L = 28.7#0.354
    dt = 0.05
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
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="blue")
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D
RPM1 = 10
RPM2 = 3


actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
for action in actions:
    X1= plot_curve(0,0,30, action[0],action[1]) # (0,0,45) hypothetical start

    for action in actions:
        X2=plot_curve(X1[0],X1[1],X1[2], action[0],action[1])
plt.grid()
# ax.set_aspect('equal')
plt.xlim(0,600)
plt.ylim(0,200)
plt.title('How to plot a vector in matplotlib ?',fontsize=10)
plt.show()
plt.close()