import math
import array as arr
import sympy
from sympy import Abs
from sympy import *
import numpy as np
import DirectKinematics as dk
import InverseKinematics as invKin

y = 0.1
DK = dk.dkmatrix()
A = invKin.A
Q = invKin.Q
J = invKin.J_lamb
q_init = invKin.q_init
A_init = invKin.A_init
A_final = Matrix([[A_init[0, 0]],
                  [A_init[1, 0]],
                  [A_init[2, 0]],
                  [A_init[3, 0]+y],
                  [A_init[0, 1]],
                  [A_init[1, 1]],
                  [A_init[2, 1]],
                  [A_init[3, 1]],
                  [A_init[0, 2]],
                  [A_init[1, 2]],
                  [A_init[2, 2]],
                  [A_init[3, 2]]])
i = 0
theta1 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta2 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta3 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta4 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta5 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta6 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
theta7 = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
q_ini = invKin.incremental_ik(A, q_init, A_init, A_final)
while i < 8:
    # we're assuming here target is 0.7 away from endeffector
    theta1[i] = q_ini[0]
    theta2[i] = q_ini[1]
    theta3[i] = q_ini[2]
    theta4[i] = q_ini[3]
    theta5[i] = q_ini[4]
    theta6[i] = q_ini[5]
    theta7[i] = q_ini[6]
    A_init = A.evalf(subs={'theta1': q_ini[0], 'theta2': q_ini[1], 'theta3': q_ini[2], 'theta4': q_ini[3],
                           'theta5': q_ini[4], 'theta6': q_ini[5], 'theta7': q_ini[6]})
    A_final = Matrix([[A_init[0, 0]],
                      [A_init[1, 0]],
                      [A_init[2, 0]],
                      [A_init[3, 0]+y],
                      [A_init[0, 1]],
                      [A_init[1, 1]],
                      [A_init[2, 1]],
                      [A_init[3, 1]],
                      [A_init[0, 2]],
                      [A_init[1, 2]],
                      [A_init[2, 2]],
                      [A_init[3, 2]]])
    q_ini = invKin.incremental_ik(A, q_ini, A_init, A_final)


def trajectory(B):
    max_acc = 50
    t = arr.array('i', [2, 2, 2, 2, 2, 1])
    # t is the time from one joint angle to the other
    tx = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
    ty = arr.array('d', [0, 0, 0, 0, 0, 0])
    speed = arr.array('d', [0, 0, 0, 0, 0, 0])
    acceleration = arr.array('d', [0, 0, 0, 0, 0, 0, 0])
    # segment1
    if B[0]-B[1] > 0:
        acceleration[0] = max_acc
    else:
        acceleration[0] = -max_acc

    tx[0] = t[0]-math.sqrt((t[0]**2)-2*(B[1]-B[0])/max_acc)
    speed[0] = (B[1]-B[0])/(t[0]-(0.5*tx[0]))
    # segment2
    speed[1] = (B[2]-B[1])/t[1]
    if speed[1]-speed[0] > 0:
        acceleration[1] = max_acc
    else:
        acceleration[1] = -max_acc
    tx[1] = (speed[1]-speed[0])/acceleration[1]
    ty[1] = t[1]-tx[0]-(0.5*tx[1])
    # segment 3 onwards
    x = 2
    while x < len(B-2):
        speed[x] = (B[x+1]-B[x])/t[x]
        if speed[x]-speed[x-1] > 0:
            acceleration[x] = max_acc
        else:
            acceleration[x] = -max_acc

        tx[x] = (speed[x]-speed[x-1])/acceleration[x]
        ty[x] = t[x]-(0.5*(tx[x-1]+tx[x]))
        x = x+1
    # last segment
    if B[len(B)-1]-B[len(B)-2] > 0:
        acceleration[len(B)-1] = max_acc
    else:
        acceleration[len(B)-1] = -max_acc
    tx[len(B)-1] = t[len(t)-1]-(math.sqrt((t[len(t)-1])**2)+(2*((B[len(B)-1])-(B[len(B)-2]))))
    speed[len(B)-1] = B[len(B)-1]-B[len(B)-2]/((t[len(t)-1])-0.5*tx[len(B)-1])

    # prior to last segment
    if speed[len(speed)-1]-speed[len(speed)-2] > 0:
        acceleration[len(B)-2] = max_acc
    else:
        acceleration[len(B)-2] = -max_acc
    tx[len(B)-2] = (speed[len(speed)-1]-speed[len(speed)-3])/acceleration[len(B)-2]
    ty[len(ty)-2] = t[len(t)-2]-(0.5*(tx[len(B)-2]+tx[len(B)-3]))
    ty[len(ty)-1] = t[len(t)-1]-tx[len(B)-1]-(0.5*tx[len(B)-2])

    return speed


trajectory(theta1)
trajectory(theta2)



















