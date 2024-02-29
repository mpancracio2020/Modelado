import csv
import matplotlib.pyplot as plt
import math
import numpy as np


def plot_results_fases_3_4(csv_filename_1, csv_filename_2):
    # Store velocities and positions
    pos_1 = []
    vel_1 = []
    pos_2 = []
    vel_2 = []
    predic_1 = []
    predic_2 = []

    # Read from csv file 1.
    with open(csv_filename_1, 'r') as archivo_csv_1:
        lector_csv_1 = csv.reader(archivo_csv_1)
        for row in lector_csv_1:
            cadena = row[0].split(" ")
            pos_1.append(float(cadena[1])) # Get positions.
            vel_1.append(float(cadena[2])) # Get velocities.
            predic_1.append(2)

     # Read from csv file 2.
    with open(csv_filename_2, 'r') as archivo_csv_2:
        lector_csv_2 = csv.reader(archivo_csv_2)
        for row in lector_csv_2:
            cadena = row[0].split(" ")
            pos_2.append(float(cadena[1]))  # Get positions.
            vel_2.append(float(cadena[2]))  # Get velocities.
            predic_2.append(2)

    MSE_1 = np.square(np.subtract(vel_1,predic_1)).mean()
    MSE_2 = np.square(np.subtract(vel_2,predic_2)).mean()

    RMSE_1 = math.sqrt(MSE_1)
    RMSE_2 = math.sqrt(MSE_2)

    print("Error fase3: ", RMSE_1, " Error fase4: ", RMSE_2)


    # Plot
    plt.plot(pos_1, vel_1)
    plt.plot(pos_2, vel_2)
    plt.xlabel('position (meters)')
    plt.ylabel('velocity (meters/seconds)')
    plt.title('Variation of the robot\'s speed depending on its position')
    plt.grid(True)
    plt.show()

def plot_results_fases_3(csv_filename_1, csv_filename_2, csv_filename_3):
    # Listas para almacenar los datos de position y velocity
    pos_1 = []
    vel_1 = []
    pos_2 = []
    vel_2 = []
    pos_3 = []
    vel_3 = []
    predic_1 = []
    predic_2 = []
    predic_3 = []

    # Read from csv file 1.
    with open(csv_filename_1, 'r') as archivo_csv_1:
        lector_csv_1 = csv.reader(archivo_csv_1)
        for row in lector_csv_1:
            cadena = row[0].split(" ")
            pos_1.append(float(cadena[1])) # Get positions.
            vel_1.append(float(cadena[2])) # Get velocities.
            predic_1.append(2)

    
    # Read from csv file 2.
    with open(csv_filename_2, 'r') as archivo_csv_2:
        lector_csv_2 = csv.reader(archivo_csv_2)
        for row in lector_csv_2:
            cadena = row[0].split(" ")
            pos_2.append(float(cadena[1]))  # Get positions.
            vel_2.append(float(cadena[2]))  # Get velocities.
            predic_2.append(2)

    # Read from csv file 3.
    with open(csv_filename_3, 'r') as archivo_csv_3:
        lector_csv_3 = csv.reader(archivo_csv_3)
        for row in lector_csv_3:
            cadena = row[0].split(" ")
            pos_3.append(float(cadena[1]))  # Get positions.
            vel_3.append(float(cadena[2]))  # Get velocities.
            predic_3.append(2)

    MSE_1 = np.square(np.subtract(vel_1,predic_1)).mean()
    MSE_2 = np.square(np.subtract(vel_2,predic_2)).mean()
    MSE_3 = np.square(np.subtract(vel_3,predic_3)).mean()

    RMSE_1 = math.sqrt(MSE_1)
    RMSE_2 = math.sqrt(MSE_2)
    RMSE_3 = math.sqrt(MSE_3)

    print("Error fase3.1: ", RMSE_1, " Error fase3.2: ", RMSE_2, " Error fase 3.3: " ,RMSE_3)


    # Trazar el gráfico
    plt.plot(pos_1, vel_1, "b")
    plt.plot(pos_2, vel_2, "g")
    plt.plot(pos_3, vel_3, "r")
    plt.xlabel('position (meters)')
    plt.ylabel('velocity (meters/seconds)')
    plt.title('Variation of the robot\'s speed depending on its position')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # Csv files names
    fase3_1 = 'Fase3.1.csv'
    fase3_2 = 'Fase3.2.csv'
    fase3_3 = 'Fase3.csv'
    fase4 = 'Fase4.csv' 
    plot_results_fases_3(fase3_1, fase3_2, fase3_3) 
    plot_results_fases_3_4(fase3_3, fase4)