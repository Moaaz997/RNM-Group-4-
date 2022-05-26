#!/usr/bin/python3

import math
import numpy
from sympy import sin, cos, symbols, MatMul, Matrix

q0, q1, q2, q3, q4, q5, q6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6 theta7')


def dkmatrix():
    a1 = Matrix([[cos(q0), -sin(q0), 0, 0],
                 [sin(q0), cos(q0), 0, 0],
                 [0, 0, 1, 0.333],
                 [0, 0, 0, 1]])
    a2 = Matrix([[cos(q1), 0, -sin(q1), 0],
                 [sin(q1), 0, cos(q1), 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])

    a3 = Matrix([[cos(q2), 0, sin(q2), 0],
                 [sin(q2), 0, -cos(q2), 0],
                 [0, 1, 0, 0.316],
                 [0, 0, 0, 1]])

    a4 = Matrix([[cos(q3), 0, sin(q3), 0.0825 * cos(q3)],
                 [sin(q3), 0, -cos(q3), 0.0825 * cos(q3)],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    a5 = Matrix([[cos(q4), 0, -sin(q4), -0.0825 * cos(q4)],
                 [sin(q4), 0, cos(q4), -0.0825 * cos(q4)],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])
    a6 = Matrix([[cos(q5), 0, sin(q5), 0],
                 [sin(q5), 0, -cos(q5), 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    a7 = Matrix([[cos(q6), 0, sin(q6), 0.088 * cos(q6)],
                 [sin(q6), 0, -cos(q6), 0.088 * cos(q6)],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    t = MatMul(a1, a2, a3, a4, a5, a6, a7)
    return t
