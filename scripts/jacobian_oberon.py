import numpy as np
from forkin_oberon import forkin_oberon

def jacobian_oberon(q, Rb, b):
	frames = forkin_oberon(q, Rb, b)
	P = frames[0:3, 3, 5]

	z1 = np.array([0, 0, 1])
	p1 = np.array([b[0][0], b[1][0], b[2][0]])

	J = np.zeros((6, 6))
	Jp1 = np.cross(z1, np.add(P, -p1))
	Jo1 = z1
	J[:, 0] = np.append(Jp1, Jo1, axis=0)

	for i in range(1,6):
		if (i == 3) or (i == 5):
			zi = frames[0:3, 0, i]
		else:
			zi = -1*frames[0:3, 1, i]

		pi = frames[0:3, 3, i-1]
		Jpi = np.cross(zi, np.add(P, -pi))
		Joi = zi
		J[:, i] = np.append(Jpi, Joi, axis=0)

	return J