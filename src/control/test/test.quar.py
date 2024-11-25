from scipy.spatial.transform import Rotation as R
import numpy as np

def quaternion_difference(q_current, q_target):
    """
    Calcula la diferencia entre un cuaternión actual y un objetivo.

    Parámetros:
    q_current : np.array
        Cuaternión actual en formato [w, x, y, z].
    q_target : np.array
        Cuaternión objetivo en formato [w, x, y, z].

    Retorna:
    np.array
        Diferencia de cuaterniones en formato [w, x, y, z].
    """
    # Normalizar ambos cuaterniones
    q_current = q_current / np.linalg.norm(q_current)
    q_target = q_target / np.linalg.norm(q_target)

    # Cambiar el formato de [w, x, y, z] a [x, y, z, w] para scipy
    q_current_scipy = np.roll(q_current, -1)
    q_target_scipy = np.roll(q_target, -1)

    # Convertir los cuaterniones a objetos de rotación de scipy
    r_current = R.from_quat(q_current_scipy)
    r_target = R.from_quat(q_target_scipy)

    # Calcular el cuaternión de diferencia
    r_diff = r_target * r_current.inv()

    # Convertir de vuelta al formato [w, x, y, z]
    q_diff = np.roll(r_diff.as_quat(), 1)

    return q_diff

# Test de orientación
def test_orientation_error():
    """
    Genera matrices de rotación a partir de diferentes ángulos de Euler,
    las convierte a cuaterniones y calcula el error de orientación entre
    una orientación actual y una objetivo, incluyendo conversión a Euler.
    """
    # Orientación actual (roll, pitch, yaw en radianes)
    roll_current, pitch_current, yaw_current = np.radians([90, 0, 0])
    R_current = R.from_euler('xyz', [roll_current, pitch_current, yaw_current])
    q_current = R_current.as_quat()  # Cuaternión [x, y, z, w]
    q_current = np.roll(q_current, 1)  # Cambiar a [w, x, y, z]

    # Orientación objetivo (roll, pitch, yaw en radianes)
    roll_target, pitch_target, yaw_target = np.radians([0, 0, 0])  # Cambiado a grados
    R_target = R.from_euler('xyz', [roll_target, pitch_target, yaw_target])
    q_target = R_target.as_quat()  # Cuaternión [x, y, z, w]
    q_target = np.roll(q_target, 1)  # Cambiar a [w, x, y, z]

    # Calcular el error de orientación usando cuaterniones
    q_diff = quaternion_difference(q_current, q_target)
    e_o = q_diff[1:]  # Parte imaginaria como error

    # Convertir el cuaternión de diferencia a Euler
    q_diff_scipy = np.roll(q_diff, -1)  # Convertir a [x, y, z, w] para scipy
    euler_error = R.from_quat(q_diff_scipy).as_euler('xyz', degrees=True)

    # Imprimir resultados
    print("Orientación Actual (Roll, Pitch, Yaw):", np.degrees([roll_current, pitch_current, yaw_current]))
    print("Matriz de Rotación Actual:\n", R_current.as_matrix())
    print("Cuaternión Actual:", q_current)

    print("\nOrientación Objetivo (Roll, Pitch, Yaw):", np.degrees([roll_target, pitch_target, yaw_target]))
    print("Matriz de Rotación Objetivo:\n", R_target.as_matrix())
    print("Cuaternión Objetivo:", q_target)

    print("\nError de Orientación (cuaternión imaginario):", e_o)
    print("Error de Orientación en Euler (grados):", euler_error)

# Ejecutar el test
if __name__ == "__main__":
    test_orientation_error()
