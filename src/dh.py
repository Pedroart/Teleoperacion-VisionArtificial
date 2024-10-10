import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    cth = sp.cos(theta); sth = sp.sin(theta)
    ca = sp.cos(alpha); sa = sp.sin(alpha)
    T = sp.Matrix([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0,        sa,     ca,      d],
                  [0,         0,      0,      1]])
    return T

def forwardDh(DH,m):
    T = dh(DH[0,0],DH[0,1],DH[0,2],DH[0,3])
    for i in range(m):
        Td = dh(DH[i,0],DH[i,1],DH[i,2],DH[i,3])
        T = sp.simplify(T*Td)

    return T

# Definir Denavit-Hartenberg
Seq = " RRRRRRR"
max_arti = 5
q = [0, 0, 0, 0, -1.74, 0, 0, 0]


# Construccion de la matriz

d     = sp.Matrix([0,0,0,1,0,1,0,0])
theta = sp.Matrix([0,
                   q[1]+sp.pi/2, 
                   q[2]+3*sp.pi/2,
                   q[3],
                   q[4]+sp.pi/2,
                   q[5]+sp.pi/2,
                   q[6]+sp.pi/2,
                   q[7]+sp.pi/2
                   ])
a     = sp.Matrix([0,0,0,0,0,0,0,0])
alfa  = sp.Matrix([0,
                   sp.pi/2,
                   sp.pi/2,
                   -sp.pi/2,
                   sp.pi/2,
                   sp.pi/2,
                   sp.pi/2,
                   sp.pi/2
                ])

DH = sp.Matrix.hstack(d,theta,a,alfa)
print(DH)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([0, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


for x in range(1,max_arti+1):
    sH = forwardDh(DH,x) 
    H = forwardDh(DH,x).evalf()
    origin = np.array(H[:3, 3], dtype=float).flatten()
    x_axis = np.array(H[:3, 0], dtype=float).flatten()
    y_axis = np.array(H[:3, 1], dtype=float).flatten()
    z_axis = np.array(H[:3, 2], dtype=float).flatten()

    ax.quiver(*origin, *x_axis, color='r', length=0.1, normalize=True)
    ax.quiver(*origin, *y_axis, color='g', length=0.1, normalize=True)
    ax.quiver(*origin, *z_axis, color='b', length=0.1, normalize=True)
    ax.text(*origin, f'Frame {x-1}', fontsize=10, ha='center')
    print(sH)

plt.show()