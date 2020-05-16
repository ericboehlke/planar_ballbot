from pydrake.all import SymbolicVectorSystem, Variable, sin, cos
import numpy as np

def make_robot():
    t = Variable("t") # time

    # state of the robot (in cartesian coordinates)
    z1 = Variable("z1") # ball angle (theta)
    z2 = Variable("z2") # body angle 
    z3 = Variable("z3") # ball angular velocity 
    z4 = Variable("z4") # body angular velocity
    cartesian_state = [z1, z2, z3, z4]

    # control input of the robot
    u1 = Variable("u1") # wheel torque
    ctrl_input = [u1]

    # nonlinear dynamics, the whole state is measured (output = state)
    mb = 1    # mass of the ball
    mB = 1    # mass of the body
    rb = 1    # radius of the ball
    l = 1     # height of the center of gravity
    Ib = 1    # inertia of the ball
    IB = 1    # inertia of the body
    g = 1  # gravitational acceleration

    theta = z1 
    theta_dot = z3 
    phi = z1 - z2 
    phi_dot = z3 - z4

    gamma1 = Ib + IB + mb*rb**2 + mB*rb**2 + mB*l**2
    gamma2 = mB*l**2 + IB

    m1 = gamma1 + 2*mB*rb*l*cos(theta + phi)
    m2 = gamma2 + mB*rb*l*cos(theta + phi)
    m3 = gamma2 + mB*rb*l*cos(theta + phi)
    m4 = gamma2

    Minv = np.array([[m4, -m2],[-m3, m1]])/(m1*m4 - m2*m3)
    C = np.array([[-mB*rb*l*sin(theta + phi)*(theta_dot + phi_dot)**2], [0]])
    G = np.array([[-mB*g*l*sin(theta+phi)], [-mB*g*l*sin(theta+phi)]])
    B = np.array([[0],[u1]])
    D = np.array([[theta_dot], [phi_dot]])
    qddot = (Minv*(B-C-G-D)).T[0]
    theta_ddot, phi_ddot = qddot
    z5 = theta_ddot
    z6 = theta_ddot - phi_ddot

    dynamics = np.array([z3, z4, z5, z6])

    # dynamics = [u1*cos(z3), u1*sin(z3), u2]
    robot = SymbolicVectorSystem(
        state=cartesian_state,
        input=ctrl_input,
        output=cartesian_state,
        dynamics=dynamics,
    )
    return robot

def make_noisy_robot():
    t = Variable("t") # time

    # state of the robot (in cartesian coordinates)
    z1 = Variable("z1") # ball angle (theta)
    z2 = Variable("z2") # body angle 
    z3 = Variable("z3") # ball angular velocity 
    z4 = Variable("z4") # body angular velocity
    cartesian_state = [z1, z2, z3, z4]

    # control input of the robot
    u1 = Variable("u1") # wheel torque
    ctrl_input = [u1]

    # nonlinear dynamics, the whole state is measured (output = state)
    mb = 1    # mass of the ball
    mB = 1    # mass of the body
    rb = 1    # radius of the ball
    l = 1     # height of the center of gravity
    Ib = 1    # inertia of the ball
    IB = 1    # inertia of the body
    g = 1.5  # gravitational acceleration

    theta = z1 
    theta_dot = z3 
    phi = z1 - z2 
    phi_dot = z3 - z4

    gamma1 = Ib + IB + mb*rb**2 + mB*rb**2 + mB*l**2
    gamma2 = mB*l**2 + IB

    m1 = gamma1 + 2*mB*rb*l*cos(theta + phi)
    m2 = gamma2 + mB*rb*l*cos(theta + phi)
    m3 = gamma2 + mB*rb*l*cos(theta + phi)
    m4 = gamma2

    Minv = np.array([[m4, -m2],[-m3, m1]])/(m1*m4 - m2*m3)
    C = np.array([[-mB*rb*l*sin(theta + phi)*(theta_dot + phi_dot)**2], [0]])
    G = np.array([[-mB*g*l*sin(theta+phi)], [-mB*g*l*sin(theta+phi)]])
    B = np.array([[0],[u1]])
    D = np.array([[1.5*theta_dot], [phi_dot]])
    qddot = (Minv*(B-C-G-D)).T[0]
    theta_ddot, phi_ddot = qddot
    z5 = theta_ddot
    z6 = theta_ddot - phi_ddot

    dynamics = np.array([z3, z4, z5, z6])

    # dynamics = [u1*cos(z3), u1*sin(z3), u2]
    robot = SymbolicVectorSystem(
        state=cartesian_state,
        input=ctrl_input,
        output=cartesian_state,
        dynamics=dynamics,
    )
    return robot