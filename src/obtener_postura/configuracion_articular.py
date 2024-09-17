import mediapipe as mp
import matplotlib.pyplot as plt
import numpy as np 
from scipy.spatial.transform import Rotation

fig = plt.figure()
bx = fig.add_subplot(1,2,2, projection='3d')



LANDMARK_GROUPS = [
    [8, 6, 5, 4, 0, 1, 2, 3, 7],   # eyes
    [10, 9],                       # mouth
    [11, 13, 15, 17, 19, 15, 21],  # left arm
    [11, 23, 25, 27, 29, 31, 27],  # left body side
    [12, 14, 16, 18, 20, 16, 22],  # right arm
    [12, 24, 26, 28, 30, 32, 28],  # right body side
    [11, 12],                      # shoulder
    [23, 24],                      # waist
]

ARM_MARK_GRUPO = [
    [0,1], # Brazo
    [1,2], # Antebrazo
    [2,3,4,5], # Mano
    [2,6], # Pulgar
]

def unitario(inicio,fin):
    vector_ = fin-inicio
    return vector_/np.linalg.norm(vector_)

def get_unitario(fin):
    inicio=np.zeros(3)
    vector_ = fin-inicio
    return vector_/np.linalg.norm(vector_)

def plot_matrix(R):
    bx.cla()
    bx.set_xlim3d(-1, 1)
    bx.set_ylim3d(-1, 1)
    bx.set_zlim3d(-1, 1)

    origin = np.array([0, 0, 0])
    x_axis = np.array([1, 0, 0])
    y_axis = np.array([0, 1, 0])
    z_axis = np.array([0, 0, 1])
    # Graficar los ejes de referencia
    bx.quiver(*origin, *x_axis, color='r', label='X original')
    bx.quiver(*origin, *y_axis, color='g', label='Y original')
    bx.quiver(*origin, *z_axis, color='b', label='Z original')

    # Graficar los ejes rotados
    bx.quiver(*origin, *R[:,0], color='r', linestyle='dashed', label='X rotado')
    bx.quiver(*origin, *R[:,1], color='g', linestyle='dashed', label='Y rotado')
    bx.quiver(*origin, *R[:,2], color='b', linestyle='dashed', label='Z rotado')

# Crear una figura y un eje
ax = fig.subplots()

def plot_angular_bars(angulos_totales):
    # Definir las etiquetas y asegurarse de que coincidan con el número de ángulos
    labels = [
        'Hombro X', 'Hombro Y', 'Hombro Z',
        'Codo X', 'Codo Y', 'Codo Z',
        'Muneca X', 'Muneca Y', 'Muneca Z'
    ]
    
    # Índices de los ángulos que se desean mostrar
    indices_a_mostrar = [0,1,2,5]  # Puedes ajustar estos índices según sea necesario
    
    # Filtrar los ángulos y las etiquetas según los índices proporcionados
    angulos_filtrados = angulos_totales[indices_a_mostrar]
    labels_filtrados = [labels[i] for i in indices_a_mostrar]
    
    # Limpiar el contenido del eje
    ax.cla()
    
    # Crear el gráfico de barras
    bar_width = 0.35
    index = np.arange(len(angulos_filtrados))
    bars = ax.bar(index, angulos_filtrados, bar_width, label='Ángulos de Euler')
    
    ax.set_ylim(-3.14, 3.14)
    # Añadir etiquetas y título
    ax.set_xlabel('Ángulos')
    ax.set_ylabel('Valor')
    ax.set_title('Ángulos de Euler en Diferentes Segmentos del Brazo')
    ax.set_xticks(index)
    ax.set_xticklabels(labels_filtrados, rotation=45, ha='right')
    ax.legend()


def procesamiento(brazo):
    # Definicion de marco de referencia de analisis
    
    resta = np.array([brazo[0] for i in range(7)])
    
    ajuste_hombro = brazo-resta
    coord_hombro = ajuste_hombro[:,[0,2,1]]
    coord_hombro[:,2] *=-1
    
    
    z=unitario(coord_hombro[ARM_MARK_GRUPO[0][1],:],coord_hombro[ARM_MARK_GRUPO[0][0],:])
    y_proyectada=unitario(coord_hombro[ARM_MARK_GRUPO[1][0],:],coord_hombro[ARM_MARK_GRUPO[1][1],:])
    x =get_unitario(np.cross(y_proyectada,z))
    y =get_unitario(np.cross(z,x))
    
    R_hombro = np.column_stack((x, y, z))
    # Crear un objeto de rotación
    r = Rotation.from_matrix(R_hombro)
    angulos_hombro = r.as_euler('ZYX')
    print("Ángulos de Euler (hombro):", angulos_hombro)
    
    z_proyectada = get_unitario(np.cross(x,y_proyectada))
    R_codo = np.column_stack((x, y_proyectada , z_proyectada))
    
    r = Rotation.from_matrix(R_hombro.T@R_codo)
    angulos_codo = r.as_euler('ZYX')
    # Obtener los ángulos de Euler en el orden ZYX (u otro orden según sea necesario)
    #print("Ángulos de Euler (codo):", angulos_codo)

    # Calculo del pulgar
    z=unitario(coord_hombro[ARM_MARK_GRUPO[3][0],:],coord_hombro[ARM_MARK_GRUPO[3][1],:])
    x_proyectada=unitario(coord_hombro[ARM_MARK_GRUPO[2][0],:],coord_hombro[ARM_MARK_GRUPO[2][1],:])
    y =get_unitario(np.cross(z,x_proyectada))
    x =get_unitario(np.cross(y,z))
    R_muneca = np.column_stack((x, y, z))
    # Crear un objeto de rotacióna
    r = Rotation.from_matrix(R_muneca)
    angulos_muneca = r.as_euler('ZYX')
    #print("Ángulos de Euler (muneca):", angulos_muneca)
    #plot_matrix(R_hombro)
    
    angulos_totales = np.concatenate((angulos_hombro, angulos_codo, angulos_muneca))
    plot_angular_bars(angulos_totales)
    return angulos_totales