import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_hyperplanes(A_file, b_file):
    # Load the coefficients and constant terms from the input files
    A = np.loadtxt(A_file)
    b = np.loadtxt(b_file)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Generate grid points for plotting
    x = np.linspace(-10, 10, 10)
    y = np.linspace(-10, 10, 10)
    X, Y = np.meshgrid(x, y)

    for i in range(len(A)):
        # Extract coefficients and constant term from hyperplane equation
        hyperplane = np.append(A[i], b[i])
        A_i = hyperplane[:-1]
        b_i = hyperplane[-1]

        # Solve for the z-coordinate of the hyperplane
        Z = (b_i - A_i[0]*X - A_i[1]*Y) / A_i[2]

        # Plot the hyperplane
        ax.plot_surface(X, Y, Z, alpha=0.5)

    # Set labels and show the plot
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


# Example usage
A_file = '../build/polyA.txt'
b_file = '../build/polyb.txt'
plot_hyperplanes(A_file, b_file)
