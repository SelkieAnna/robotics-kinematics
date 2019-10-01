import math
import numpy as np

links_len = [670, 312, 1075, 225, 1280, 215]

def R_x(angle):
    R = np.zeros((4, 4))
    R[0, 0] = 1
    R[1, 1] = math.cos(angle)
    R[1, 2] = -math.sin(angle)
    R[2, 1] = math.sin(angle)
    R[2, 2] = math.cos(angle)
    R[3, 3] = 1
    return R

def R_y(angle):
    R = np.zeros((4, 4))
    R[1, 1] = 1
    R[2, 2] = math.cos(angle)
    R[2, 0] = -math.sin(angle)
    R[0, 2] = math.sin(angle)
    R[0, 0] = math.cos(angle)
    R[3, 3] = 1
    return R

def R_z(angle):
    R = np.zeros((4, 4))
    R[2, 2] = 1
    R[0, 0] = math.cos(angle)
    R[0, 1] = -math.sin(angle)
    R[1, 0] = math.sin(angle)
    R[1, 1] = math.cos(angle)
    R[3, 3] = 1
    return R

def T_x(amount):
    T = np.zeros((4, 4))
    T[0, 0] = 1
    T[1, 1] = 1
    T[2, 2] = 1
    T[0, 3] = amount
    T[3, 3] = 1
    return T

def T_y(amount):
    T = np.zeros((4, 4))
    T[0, 0] = 1
    T[1, 1] = 1
    T[2, 2] = 1
    T[1, 3] = amount
    T[3, 3] = 1
    return T

def T_z(amount):
    T = np.zeros((4, 4))
    T[0, 0] = 1
    T[1, 1] = 1
    T[2, 2] = 1
    T[2, 3] = amount
    T[3, 3] = 1
    return T

# R to euler angles (xyx)
#   sin(b) is assunmed to be > 0
def rot2eul(R):
    c = math.atan2(R[1, 2], R[1, 3])
    b = math.atan2(math.sqrt(R[1, 2]**2, R[1, 3]**2), R[1, 1])
    a = math.atan2(R[2, 1], -R[3, 1])
    return [a, b, c]

# T - transformation matrix of current end-effector position
def FK(q):
    T = np.dot(R_z(q[0]), T_z(links_len[0])).dot(T_y(links_len[1])).dot(R_x(q[1]))\
        .dot(T_y(links_len[2])).dot(R_x(q[2])).dot(T_y(links_len[3]))\
            .dot(T_z(-links_len[4])).dot(R_z(q[3])).dot(R_x(q[4])).dot(R_z(q[5]))\
                .dot(T_z(-links_len[5]))
    return T

# T = transformation matrix of desired end-effector position
# def IK(T):
    # move the frame from the position of the tool to the position of q4
    # first, 2-link manipulator consisting of l3 and l4
    # k = TODO
    # z = TODO
    # q3 = math.acos((k**2 - links_len[2]**2 - links_len[3]**2)/(2*links_len[2]*links_len[3]))
    # q2 = - math.copysign(math.atan2(links_len[3]*math.sin(q3), links_len[2] + links_len[3]*math.cos(q3)), q3) + \
    #     math.atan2(z, k)
    # project to 3D
    # TODO
    # pass

if __name__ == "__main__":
    print(FK([0, 0, 0, 0, 0, 0]))