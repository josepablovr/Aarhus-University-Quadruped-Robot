import numpy as np

def ForwardK1(t0, t1, t2, lambda_, delta):
    a0 = 0.04
    a1 = 0.3
    a2 = 0.3
    # h = 0.05
    l = 0.3
    w = 0.175

    s0 = np.sin(t0)
    s1 = np.sin(t1)
    c0 = np.cos(t0)
    c1 = np.cos(t1)
    s12 = np.sin(t1 + t2)
    c12 = np.cos(t1 + t2)

    x = -a1 * s1 - a2 * s12 + delta * l
    y = lambda_ * a0 * c0 + a2 * s0 * c12 + a1 * c1 * s0 + lambda_ * w
    z = lambda_ * a0 * s0 - a2 * c0 * c12 - a1 * c0 * c1

    return x, y, z

def CrossMatrix(wb):
    crossMatrix = np.matrix([[0, -wb[2, 0], wb[1, 0]],
                            [wb[2, 0], 0, -wb[0, 0]],
                            [-wb[1, 0], wb[0, 0], 0]])
    return crossMatrix

def R1(alpha, beta, gamma):
    t2 = np.cos(alpha)
    t3 = np.cos(beta)
    t4 = np.cos(gamma)
    t5 = np.sin(alpha)
    t6 = np.sin(beta)
    t7 = np.sin(gamma)
    w = np.matrix([
        [t2*t3, t3*t5, -t6],
        [-t4*t5 + t2*t6*t7, t2*t4 + t5*t6*t7, t3*t7],
        [t5*t7 + t2*t4*t6, -t2*t7 + t4*t5*t6, t3*t4]
    ])
    return w