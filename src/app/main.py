import time

from app import App

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)  # Asegura que se realice limpieza al cerrar
    app.mainloop()
