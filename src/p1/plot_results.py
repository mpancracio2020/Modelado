import csv
import matplotlib.pyplot as plt
import math
import numpy as np


def plot_results(csv_filename_1, csv_filename_2):
    # Listas para almacenar los datos de posición y velocidad
    pos_1 = []
    vel_1 = []
    pos_2 = []
    vel_2 = []
    predic_1 = []
    predic_2 = []

    # Leer datos del archivo CSV
    with open(csv_filename_1, 'r') as archivo_csv_1:
        lector_csv_1 = csv.reader(archivo_csv_1)
        #next(lector_csv)  # Omitir la primera fila si contiene encabezados
        for fila in lector_csv_1:
            cadena = fila[0].split(" ")
            pos_1.append(float(cadena[1]))  # Suponiendo que la posición está en la primera columna
            vel_1.append(float(cadena[2]))  # Suponiendo que la velocidad está en la segunda columna
            predic_1.append(2)

    
    # Leer datos del archivo CSV
    with open(csv_filename_2, 'r') as archivo_csv_2:
        lector_csv_2 = csv.reader(archivo_csv_2)
        #next(lector_csv)  # Omitir la primera fila si contiene encabezados
        for fila in lector_csv_2:
            cadena = fila[0].split(" ")
            pos_2.append(float(cadena[1]))  # Suponiendo que la posición está en la primera columna
            vel_2.append(float(cadena[2]))  # Suponiendo que la velocidad está en la segunda columna
            predic_2.append(2)

    MSE_1 = np.square(np.subtract(vel_1,predic_1)).mean()
    MSE_2 = np.square(np.subtract(vel_2,predic_2)).mean()

    RMSE_1 = math.sqrt(MSE_1)
    RMSE_2 = math.sqrt(MSE_2)

    print("Error fase3: ", RMSE_1, " Error fase4: ", RMSE_2)


    # Trazar el gráfico
    plt.plot(pos_1, vel_1)
    plt.plot(pos_2, vel_2)
    plt.xlabel('Posición (metros)')
    plt.ylabel('Velocidad (metros/segundos)')
    plt.title('Variación de la velocidad del robot en función de su posición')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    fase3 = 'Fase3.csv'
    fase4 = 'Fase4.csv'  # Nombre del archivo CSV con los datos del robot
    plot_results(fase3, fase4)