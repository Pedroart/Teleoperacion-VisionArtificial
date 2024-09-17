import cv2

class camara:
    '''
    
    '''

    def __init__(self) -> None:
        self.devN = 2
        self.activo =False
        self.imagen = None

        self.set_caramara(self.devN)
        

    def set_caramara(self,devN):
        self.cap = cv2.VideoCapture(devN)
        self.activo = True
        return True

    def caputer(self):
        if self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                
                self.activo = False
                return False
            self.imagen = image
        return True
    
    def set_redimencionar(self,size):
        #INTER_NEAREST
        '''
        INTER_NEAREST resize:  Time Taken: 0:00:00.001024
        INTER_LINEAR resize:   Time Taken: 0:00:00.004321
        INTER_CUBIC resize:    Time Taken: 0:00:00.007929
        INTER_LANCZOS4 resize: Time Taken: 0:00:00.021042
        INTER_AREA resize:     Time Taken: 0:00:00.065569
        '''
        return cv2.resize(self.imagen, size, interpolation=cv2.INTER_NEAREST)
        