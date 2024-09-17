import numpy as np
from scipy.spatial.transform import Rotation as R

def unitario(fin,inicio):
    vector_ = fin-inicio
    return vector_/np.linalg.norm(vector_)

def matriz_rotacion(vector,deseado):
    rot, _ = R.align_vectors(vector, deseado)
    return rot.as_matrix()