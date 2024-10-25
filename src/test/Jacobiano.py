import sympy as sp
import numpy as np

def jacobiano(params_dh):

    
    # Función para obtener la matriz de transformación homogénea usando los parámetros DH
    def dh_matrix(theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0,              sp.sin(alpha),               sp.cos(alpha),               d],
            [0,              0,                           0,                           1]
        ])
    
    # Inicializamos las matrices para obtener las posiciones y ejes
    T_total = sp.eye(4)
    p = []   # Posiciones de los eslabones
    z = []   # Ejes de rotación (z) de cada eslabón

    # Posición y eje de la base
    p.append(sp.Matrix([0, 0, 0]))
    z.append(sp.Matrix([0, 0, 1]))

    # Multiplicamos las matrices DH de cada eslabón y obtenemos las posiciones y ejes
    for param in params_dh:
        theta, d, a, alpha = param
        T_total = T_total * dh_matrix(theta, d, a, alpha)
        
        # Extraemos la posición y el eje z de la transformación actual
        p.append(T_total[:3, 3])   # Posición
        z.append(T_total[:3, 2])   # Eje z

    # Posición del efector final (último eslabón)
    p_n = p[-1]

    # Inicializamos el Jacobiano
    J_linear = []
    J_angular = []

    # Calculamos cada columna del Jacobiano
    for i in range(len(params_dh)):
        # Si la articulación es rotacional:
        J_linear.append(z[i].cross(p_n - p[i]))  # Parte lineal (producto cruz)
        J_angular.append(z[i])                   # Parte angular (eje z)

    # Convertimos en matrices
    J_linear = sp.Matrix.hstack(*J_linear)
    J_angular = sp.Matrix.hstack(*J_angular)

    # Jacobiano completo
    J = sp.Matrix.vstack(J_linear, J_angular)
    
    return J

def evaluar_jacobiano(J, q_values):
    # Reemplazar los valores conocidos de las variables en el Jacobiano
    J_numeric = J.subs(q_values)
    
    # Convertir el Jacobiano a una matriz numpy
    J_numpy = np.array(J_numeric).astype(np.float64)
    
    return J_numpy



# Inicializamos las variables simbólicas para las articulaciones
q1, q2, q3, q4, q5, q6, q7 = sp.symbols('q1 q2 q3 q4 q5 q6 q7')

# Ejemplo de uso
params_dh = [
    [q1, 0.237, 0.081, -sp.pi/2],  # Joint 1
    [q2, 0.1925, 0, -sp.pi/2],     # Joint 2
    [q3, 0.4, 0, -sp.pi/2],        # Joint 3
    [q4, -0.1685, 0, -sp.pi/2],    # Joint 4
    [q5, 0.4, 0, -sp.pi/2],        # Joint 5
    [q6, 0.1363, 0, -sp.pi/2],     # Joint 6
    [q7, 0.11, 8.08E-07, 0]        # Joint 7
]

# Calcular el Jacobiano simbólico
J_simbolico = jacobiano(params_dh)

# Evaluar el Jacobiano con valores conocidos
q_values = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0, q7: 0}
J_numerico = evaluar_jacobiano(J_simbolico, q_values)

# Mostrar el Jacobiano con los valores numéricos
print(J_numerico)
