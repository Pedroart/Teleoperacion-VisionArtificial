import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),                d],
        [0,              0,                            0,                            1]
    ])
    
    

def fkine(q):
    # Parámetros DH para un brazo robótico estándar de 6 DoF
    dh_params = [
        [q[0] + np.pi, 0.089159, 0, np.pi/2],  # Joint 1
        [q[1], 0, -0.425, 0],                 # Joint 2
        [q[2], 0, -0.39225, 0],               # Joint 3
        [q[3], 0.10915, 0, np.pi/2],          # Joint 4
        [q[4], 0.09465, 0, -np.pi/2],         # Joint 5
        [q[5], 0.0823, 0, 0]                  # Joint 6
    ]
    
    T = np.eye(4)
    for params in dh_params:
        theta, d, a, alpha = params
        T = T @ dh(d, theta, a, alpha)
    return T


def jacobian(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 # Crear una matriz 3xn
 n = q.size
 J = np.zeros((3,n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine(q)
    
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta
  dq[i] += delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_inc = fkine(dq)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J


def ikine(xdes, q0):
    """
    Calcular la cinemática inversa de un brazo robótico numéricamente a partir
    de la configuración articular inicial q0. El objetivo incluye posición y orientación
    descritas por un cuaternión.
    
    Entrada:
        xdes -- Pose deseada en formato [x, y, z, ew, ex, ey, ez]
        q0   -- Configuración inicial de las articulaciones (vector numpy)
    
    Salida:
        q -- Configuración articular que aproxima la pose deseada
    """
    epsilon = 0.001  # Tolerancia para la convergencia
    max_iter = 1000  # Número máximo de iteraciones
    delta = 0.00001  # Incremento para diferencias finitas

    q = copy(q0)
    for i in range(max_iter):
        # Cinemática directa para la configuración actual
        T = fkine(q)
        xcurr = TF2xyzquat(T)
        
        # Error de posición y orientación
        error_pos = xdes[0:3] - xcurr[0:3]
        error_orient = xdes[3:] - xcurr[3:]
        
        # Concatenar errores
        error = np.hstack((error_pos, error_orient))
        
        # Si el error está por debajo del umbral, salir del bucle
        if np.linalg.norm(error) < epsilon:
            break
        
        # Calcular Jacobiano
        J = jacobian(q)
        
        # Extender Jacobiano para incluir orientación
        J_orient = np.zeros((4, J.shape[1]))
        
        for j in range(J.shape[1]):
            dq = copy(q)
            dq[j] += delta
            T_inc = fkine(dq)
            quat_curr = rot2quat(T[0:3, 0:3])
            quat_inc = rot2quat(T_inc[0:3, 0:3])
            J_orient[:, j] = (quat_inc - quat_curr) / delta
        
        J_ext = np.vstack((J, J_orient))
        
        # Resolver para el cambio en las articulaciones
        dq = np.linalg.pinv(J_ext).dot(error)
        
        # Actualizar configuración articular
        q += dq
    
    return q



def ik_gradient(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo gradiente.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001

 q  = copy(q0)
 for i in range(max_iter):
  # Main loop
  pass
    
 return q

    
def rot2quat(R):
 """
 Convertir una matriz de rotacion en un cuaternion

 Entrada:
  R -- Matriz de rotacion
 Salida:
  Q -- Cuaternion [ew, ex, ey, ez]

 """
 dEpsilon = 1e-6
 quat = 4*[0.,]

 quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
 if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
  quat[1] = 0.0
 else:
  quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
 if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
  quat[2] = 0.0
 else:
  quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
 if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
  quat[3] = 0.0
 else:
  quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

 return np.array(quat)


def TF2xyzquat(T):
 """
 Convert a homogeneous transformation matrix into the a vector containing the
 pose of the robot.

 Input:
  T -- A homogeneous transformation
 Output:
  X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
       is Cartesian coordinates and the last part is a quaternion
 """
 quat = rot2quat(T[0:3,0:3])
 res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 return np.array(res)


# Configuración inicial de las articulaciones (ejemplo para un robot de 6 DoF)
q0 = np.array([0.0, -np.pi/4, np.pi/4, 0.0, np.pi/6, 0.0])

# Pose deseada [x, y, z, ew, ex, ey, ez]
xdes = np.array([0.3, 0.2, 0.4, 0.7071, 0.0, 0.7071, 0.0])  # Pose con orientación usando cuaterniones

# Test de ikine
q_result = ikine(xdes, q0)

# Mostrar resultados
print("Configuración inicial q0:", q0)
print("Pose deseada xdes:", xdes)
print("Configuración final q_result:", q_result)

# Verificar resultado
T_result = fkine(q_result)
x_result = TF2xyzquat(T_result)
print("Pose alcanzada x_result:", x_result)
print("Error de posición:", np.linalg.norm(x_result[:3] - xdes[:3]))
print("Error de orientación:", np.linalg.norm(x_result[3:] - xdes[3:]))
