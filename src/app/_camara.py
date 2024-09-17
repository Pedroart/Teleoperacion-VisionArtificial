import cv2

class camara:

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
        