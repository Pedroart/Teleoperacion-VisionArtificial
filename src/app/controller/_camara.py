import cv2

class camara:
    ancho = 0
    alto = 0
    def __init__(self,devN = 0) -> None:
        self.devN = devN
        self.activo = False
        self.imagen = None

        self.cap = cv2.VideoCapture(0)
        self.set_camara(self.devN)
        

    def set_camara(self,devN):
        if self.cap.isOpened():
            self.cap.release()
        self.devN = devN
        self.cap = cv2.VideoCapture(devN)
        self.activo = True
        if self.cap.isOpened():
            self.ancho = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.alto = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(self.ancho,self.alto)
        else:
            self.activo = False
        return True

    def captura(self):
        if self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                self.activo = False
                return False
            self.imagen = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            return True
        
        self.activo = False
        return False
    
    def set_redimencionar(self,size):
        #INTER_NEAREST
        '''
        INTER_NEAREST resize:  Time Taken: 0:00:00.001024
        INTER_LINEAR resize:   Time Taken: 0:00:00.004321
        INTER_CUBIC resize:    Time Taken: 0:00:00.007929
        INTER_LANCZOS4 resize: Time Taken: 0:00:00.021042
        INTER_AREA resize:     Time Taken: 0:00:00.065569
        '''
        self.imagen = cv2.resize(self.imagen, size, interpolation=cv2.INTER_AREA)
        return True
    
def redimensionar_imagen_porcentaje(imagen, porcentaje):
    """
    Redimensiona la imagen en función de un porcentaje de su tamaño original.

    :param imagen: La imagen original a redimensionar.
    :param porcentaje: Porcentaje de redimensionamiento (ejemplo: 50 para reducir al 50%, 150 para aumentar al 150%).
    :return: La imagen redimensionada.
    """
    # Obtener las dimensiones originales de la imagen
    alto_original, ancho_original = imagen.shape[:2]

    # Calcular las nuevas dimensiones
    nuevo_ancho = ancho_original * porcentaje // 100
    nuevo_alto = alto_original * porcentaje // 100

    # Redimensionar la imagen usando cv2.resize
    imagen_redimensionada = cv2.resize(imagen, (nuevo_ancho, nuevo_alto), interpolation=cv2.INTER_AREA)

    return imagen_redimensionada

def set_redimencionar(imagen,size):
        return cv2.resize(imagen, size, interpolation=cv2.INTER_AREA)