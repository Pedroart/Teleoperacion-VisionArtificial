from scipy.spatial.transform import Rotation as R
import numpy as np

# Ejemplo: matriz de rotación de error R_error
R_error = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1]
])

# Crear una instancia de Rotation a partir de la matriz de rotación
rotation = R.from_matrix(R_error)

# Convertir a un vector de rotación
rotation_vector = rotation.as_rotvec()
print(rotation_vector)
