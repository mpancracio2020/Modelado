import csv
import matplotlib.pyplot as plt


def plot_results(csv_filename):
    # Listas para almacenar los datos de posición y velocidad
    pos = []
    vel = []

    # Leer datos del archivo CSV
    with open(csv_filename, 'r') as archivo_csv:
        lector_csv = csv.reader(archivo_csv)
        #next(lector_csv)  # Omitir la primera fila si contiene encabezados
        for fila in lector_csv:
            cadena = fila[0].split(" ")
            pos.append(float(cadena[1]))  # Suponiendo que la posición está en la primera columna
            vel.append(float(cadena[2]))  # Suponiendo que la velocidad está en la segunda columna

    # Trazar el gráfico
    plt.plot(pos, vel)
    plt.xlabel('Posición (metros)')
    plt.ylabel('Velocidad (metros/segundos)')
    plt.title('Variación de la velocidad del robot en función de su posición')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    nombre_archivo_csv = 'Fase3.csv'  # Nombre del archivo CSV con los datos del robot
    plot_results(nombre_archivo_csv)