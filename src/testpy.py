import sympy as sp

def dh_transform_matrix(theta, d, a, alpha):
    """
    Calcula la matriz de transformación DH para los parámetros dados, utilizando símbolos.
    """
    T = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

# Definir las variables simbólicas para los ángulos y las longitudes
q1, q2, q3, q4, q5, q6, q7 = sp.symbols('q1 q2 q3 q4 q5 q6 q7')

# Definir los parámetros DH simbólicos según la tabla proporcionada
params_dh = [
    [q1, 0.237, 0.081, -sp.pi/2],  # Joint 1
    [q2, 0.1925, 0, -sp.pi/2],     # Joint 2
    [q3, 0.4, 0, -sp.pi/2],        # Joint 3
    [q4, -0.1685, 0, -sp.pi/2],    # Joint 4
    [q5, 0.4, 0, -sp.pi/2],        # Joint 5
    [q6, 0.1363, 0, -sp.pi/2],     # Joint 6
    [q7, 0.11, 8.08E-07, 0]        # Joint 7
]

# Calcular las matrices de transformación para cada junta simbólicamente
T_matrices = []
for params in params_dh:
    theta, d, a, alpha = params
    T_matrices.append(dh_transform_matrix(theta, d, a, alpha))

# Calcular la matriz de transformación acumulada para cada junta
T_accumulated = sp.eye(4)  # Matriz identidad 4x4 para inicializar
T_accumulated_matrices = []  # Lista para guardar las matrices acumuladas

for i, T in enumerate(T_matrices):
    T_accumulated = T_accumulated * T  # Producto de matrices
    T_accumulated_matrices.append(T_accumulated)

# Imprimir las matrices de transformación acumuladas
for i, T_acc in enumerate(T_accumulated_matrices):
    sp.pretty_print(f"Matriz de transformación acumulada hasta la junta {i+1}:")
    sp.pretty_print(T_acc)
    print("\n")
