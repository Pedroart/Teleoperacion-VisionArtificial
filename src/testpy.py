import numpy as np

# Definimos las coordenadas de los puntos A, B y C
A = np.array([0, 1, 0])  # Ejemplo de coordenada de A
B = np.array([1, 0, 0])  # Ejemplo de coordenada de B

C = np.array([0, 0, 0])  # Ejemplo de coordenada de C

# Calculamos los vectores AC y BC
AC = C - A
BC = C - B

# Calculamos el ángulo entre los vectores AC y BC usando el producto punto
cos_theta = np.dot(AC, BC) / (np.linalg.norm(AC) * np.linalg.norm(BC))
theta_rad = np.arccos(cos_theta)

# Convertimos el ángulo de radianes a grados
theta_deg = np.degrees(theta_rad)

print(A)