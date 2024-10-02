import time

from app import App

def medir_tiempo(func):
    def wrapper(*args, **kwargs):
        start = time.time()  # Tiempo de inicio
        result = func(*args, **kwargs)  # Ejecuta la función original
        end = time.time()  # Tiempo de fin
        print(f"Tiempo de ejecución de {func.__name__}: {end - start:.6f} segundos")
        return result  # Retorna el resultado de la función original
    return wrapper

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)  # Asegura que se realice limpieza al cerrar
    app.mainloop()
