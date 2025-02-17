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
            g_force.append(abs(float(cadena[0]))) # Get positions.
            times.append(abs(float(cadena[2]))) # Get velocities.


    # Plot
    plt.plot(times, g_force)
    plt.xlabel('time (sec)')
    plt.ylabel('force (N)')
    plt.title('Variation of the robot\'s forces ')
    plt.grid(True)
    plt.show()




if __name__ == "__main__":
    # Csv files names
    
    forces = 'p2.py'
    plot_results(forces) 