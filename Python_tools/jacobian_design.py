import sympy as sp
import sys
from sympy import pprint

sys.stdout.reconfigure(encoding="utf-8")

sp.init_printing(use_unicode=True)

def clean(expr):
    expr = sp.nsimplify(expr)
    expr = sp.simplify(expr)
    expr = sp.trigsimp(expr)
    expr = sp.factor(expr)
    return expr


def print_expr(name, expr):
    print(f"\n{name}:")
    pprint(clean(expr), wrap_line=False, num_columns=140)


def print_matrix(name, M):
    print(f"\n{name}:")
    pprint(M.applyfunc(clean), wrap_line=False, num_columns=160)


def dh_transform(a, alpha, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                             0,                           1]
    ])


def get_R(T):
    return T[:3, :3]


def get_o(T):
    return T[:3, 3]


def get_z(T):
    return T[:3, 2]


theta1, theta2, d3 = sp.symbols("theta_1 theta_2 d_3")

# Fixed base transform
T_base = dh_transform(0.0, 0, 10, 0.0)

# Actual joint transforms
T1 = dh_transform(20, 0, sp.Rational(7, 2), theta1)   # revolute
T2 = dh_transform(15, 0, sp.Rational(7, 2), theta2)   # revolute
T3 = dh_transform(sp.Rational(7, 4), 0, d3, 0)       # prismatic TCP offset

# Cumulative transforms
H0 = T_base
H1 = T_base * T1
H2 = T_base * T1 * T2
H3 = T_base * T1 * T2 * T3

o_end = get_o(H3)

# 6x3 geometric Jacobian
J = sp.zeros(6, 3)

# Joint 1: revolute
z0 = get_z(H0)
o0 = get_o(H0)

J[:3, 0] = z0.cross(o_end - o0)
J[3:6, 0] = z0

# Joint 2: revolute
z1 = get_z(H1)
o1 = get_o(H1)

J[:3, 1] = z1.cross(o_end - o1)
J[3:6, 1] = z1

# Joint 3: prismatic
z2 = get_z(H2)

J[:3, 2] = z2
J[3:6, 2] = sp.Matrix([0, 0, 0])

J = sp.simplify(J)

J_pos = clean(J[:3, :])

w1, w2, v3 = sp.symbols("omega_1 omega_2 v_3")
vx, vy, vz = sp.symbols("v_x v_y v_z")

q_dot = sp.Matrix([
    w1,
    w2,
    v3
])

tcp_vel = clean(J_pos * q_dot)

x_dot = clean(tcp_vel[0])
y_dot = clean(tcp_vel[1])
z_dot = clean(tcp_vel[2])

print_matrix("Position Jacobian", J_pos)

print_expr("x_dot", x_dot)
print_expr("y_dot", y_dot)
print_expr("z_dot", z_dot)

det_J = clean(J_pos.det())
print_expr("det(J_pos)", det_J)

J_pos_inv = clean(J_pos.inv())
print_matrix("Inverse Position Jacobian", J_pos_inv)

tcp_vel_cmd = sp.Matrix([
    vx,
    vy,
    vz
])

q_dot_solution = clean(J_pos_inv * tcp_vel_cmd)

theta1_dot_eq = clean(q_dot_solution[0])
theta2_dot_eq = clean(q_dot_solution[1])
d3_dot_eq = clean(q_dot_solution[2])

print_expr("theta1_dot", theta1_dot_eq)
print_expr("theta2_dot", theta2_dot_eq)
print_expr("d3_dot", d3_dot_eq)

def print_c(name, expr):
    print(sp.ccode(clean(expr), assign_to=name, full_prec=False))


print("\nC code equations:")

print_c("x_dot", x_dot)
print_c("y_dot", y_dot)
print_c("z_dot", z_dot)

print_c("theta1_dot", theta1_dot_eq)
print_c("theta2_dot", theta2_dot_eq)
print_c("d3_dot", d3_dot_eq)

# x_dot = -1.0/4.0*(80*omega_1*sin(theta_1) + 67*omega_1*sin(theta_1 + theta_2) + 67*omega_2*sin(theta_1 + theta_2));
# y_dot = (1.0/4.0)*(80*omega_1*cos(theta_1) + 67*omega_1*cos(theta_1 + theta_2) + 67*omega_2*cos(theta_1 + theta_2));
# z_dot = v_3;
# theta1_dot = (1.0/20.0)*(v_x*cos(theta_1 + theta_2) + v_y*sin(theta_1 + theta_2))/sin(theta_2);
# theta2_dot = -1.0/1340.0*(80*v_x*cos(theta_1) + 67*v_x*cos(theta_1 + theta_2) + 80*v_y*sin(theta_1) + 67*v_y*sin(theta_1 + theta_2))/sin(theta_2);
# d3_dot = v_z;