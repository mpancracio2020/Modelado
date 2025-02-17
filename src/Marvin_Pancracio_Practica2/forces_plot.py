import csv
import matplotlib.pyplot as plt
import math
import numpy as np


def plot_results(folder):

     # Store velocities and positions
    pos_1 = []
    vel_1 = []
    pos_2 = []
    vel_2 = []
    predic_1 = []
    predic_2 = []

    g_force = [ ]
    times = [ ]


    # Read from csv file 1.
    with open(folder, 'r') as archivo_csv_1:
        lector_csv_1 = csv.reader(archivo_csv_1)
        for row in lector_csv_1:
            cadena = row[0].split(",")
            g_force.append(abs(float(row[0]))) # Get positions.
            times.append(abs(float(row[2]))) # Get velocities.


    # Plot
    g = 1205768624.972866
    desv = np.std(g_force)
    plt.plot(g_force,times)
    plt.xlabel('time (sec)')
    plt.ylabel('force (N)')
    plt.title(f"Variation of the robot\"s forces | G_Total: {g} | \nStandard deviation: {desv}")
    plt.grid(True)
    plt.show()




if __name__ == "__main__":
    # Csv files names    
    f = 'Fase3_Marvin_Pancracio.csv'
    plot_results(f) 