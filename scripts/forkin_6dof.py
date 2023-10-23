import numpy as np

def forkin_6dof(q, Rb, b):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    rotations = np.zeros((3, 3, 6))
    rotations[:, :, 0] = rotx(q1)
    rotations[:, :, 1] = roty(q2)
    rotations[:, :, 2] = roty(q3)
    rotations[:, :, 3] = rotx(q4)
    rotations[:, :, 4] = roty(q5)
    rotations[:, :, 5] = rotx(q6)

    displacements = np.zeros((3, 1, 6))
    displacements[:, :, 0] = np.array([[0.140, 0, 0]]).transpose()
    displacements[:, :, 1] = np.array([[0.120, 0, 0]]).transpose()
    displacements[:, :, 2] = np.array([[0.140, 0, 0]]).transpose()
    displacements[:, :, 3] = np.array([[0.115, 0, 0]]).transpose()
    displacements[:, :, 4] = np.array([[0.140, 0, 0]]).transpose()
    displacements[:, :, 5] = np.array([[0.095, 0, 0]]).transpose()

    frames = np.zeros((4, 4, 6))
    frames[3, 3, 0] = 1
    frames[0:3, 0:3, 0] = np.dot(Rb, rotations[:, :, 0])
    frames[0:3, 3, 0] = b.transpose() + np.dot(frames[0:3, 0:3, 0], displacements[:, :, 0]).transpose()
    
    for i in range(1,6):
        frames[3, 3, i] = 1
        frames[0:3, 0:3, i] = np.dot(frames[0:3, 0:3, i-1],rotations[:, :, i])
        frames[0:3, 3, i] = np.add(frames[0:3, 3, i-1], np.dot(frames[0:3, 0:3, i], displacements[:, :, i]).transpose())

    return frames



def rotx(th):
    R = np.array([[1, 0, 0],\
        [0, np.cos(th), -np.sin(th)],\
        [0, np.sin(th), np.cos(th)]])
    return R



def roty(th):
    R = np.array([[np.cos(th), 0, np.sin(th)],\
        [0, 1, 0],\
        [-np.sin(th), 0, np.cos(th)]])
    return R



def rotz(th):
    R = np.array([[np.cos(th), -np.sin(th), 0],\
        [np.sin(th), np.cos(th), 0],\
        [0, 0, 1]])
    return R