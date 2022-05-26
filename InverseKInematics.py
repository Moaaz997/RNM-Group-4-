#!/usr/bin/python3

from sympy import symbols, init_printing, Matrix
from sympy import sin as s
from sympy import cos as c
import sympy
from sympy import Abs
import numpy as np
import DirectKinematics as dk

init_printing(use_unicode=True)

# let's implement the direct kinematics symbolically
DK = dk.dkmatrix()

# reshape to column vector A = [a11, a21, a31, ..., a34]
A = DK[0:3, 0:4]  # crop last row
A = sympy.transpose(A)

A = Matrix([[A[0, 0]],
            [A[1, 0]],
            [A[2, 0]],
            [A[3, 0]],
            [A[0, 1]],
            [A[1, 1]],
            [A[2, 1]],
            [A[3, 1]],
            [A[0, 2]],
            [A[1, 2]],
            [A[2, 2]],
            [A[3, 2]]])

Q = Matrix([dk.q0, dk.q1, dk.q2, dk.q3, dk.q4, dk.q5, dk.q6])
J = A.jacobian(Q)
# jacobian has the size (12,7)

q_init = Matrix([[np.deg2rad(40), np.deg2rad(40), np.deg2rad(40), np.deg2rad(40), np.deg2rad(40), np.deg2rad(40),
                  np.deg2rad(40)]]).transpose()
A_init = A.evalf(subs={'theta1': q_init[0], 'theta2': q_init[1], 'theta3': q_init[2], 'theta4': q_init[3],
                       'theta5': q_init[4], 'theta6': q_init[5], 'theta7': q_init[6]})
A_final = Matrix([
    0.5792, 0.5792, -0.5736,
    -0.7071, 0.7071, 0,
    0.4056, 0.4056, 0.8192,
    93.2834, 93.2834, 288.4050
])


def incremental_ik(B, q_current, A_current, A_finall):
    delta_A = (A_finall - A_current)
    while max(Abs(delta_A)) > 0.01:
        J_q = J.evalf(subs={'theta1': q_current[0], 'theta2': q_current[1], 'theta3': q_current[2],
                            'theta4': q_current[3], 'theta5': q_current[4], 'theta6': q_current[5],
                            'theta7': q_current[6]})
        # use pseudoinverse to solve over-determined problem
        # delta_A/10 is our linear interpolation between current and final pose
        delta_q = J_q.pinv() @ delta_A / 10  # @ is matrix product
        q_current = q_current + delta_q
        A_current = B.evalf(subs={'theta1': q_current[0], 'theta2': q_current[1], 'theta3': q_current[2],
                                  'theta4': q_current[3], 'theta5': q_current[4], 'theta6': q_current[5],
                                  'theta7': q_current[6]})
        delta_A = (A_finall - A_current)
    return q_current


q_final = incremental_ik(A, q_init, A_init, A_final)

A_final2 = A.evalf(subs={'theta1': q_init[0], 'theta2': q_init[1], 'theta3': q_init[2],
                         'theta4': q_init[3], 'theta5': q_init[4], 'theta6': q_init[5],
                         'theta7': q_init[6]})
print(A_final2.reshape(4, 3))
