import numpy as np



import time

# Define the function you want to time
def my_function():
    # Define constants
    mu0 = 4 * np.pi * 10**-7
    N1= 350 # Number of turns in inner coil
    N2 = 412 # Number of turns in middle coil
    N3 = 409 # Number of turns in outer coil
    NAvg = (N1+N2+N3)/3 # Average number of turns
    R1 = 0.0868 # Radius of inner coil
    R2 = 0.1268 # Radius of middle coil
    R3 = 0.1688 # Radius of outer coil

    K = 0.8 # Constant depending oyen magnetic properties of the conductive object

# Define the desired force and distance to the conductive object
    F_desired = 0.001 # Newtons
    d = 0.20 # meters

# Set initial guesses for the currents
    I1 = 1 # Amperes
    I2 = 1 # Amperes
    I3 = 1 # Amperes

# Define a tolerance for convergence
    tolerance = 0.000001 # Newtons

# Start the iterative solver
    while True:
    # Calculate the magnetic field strengths for each coil
        B1 = mu0 * N1 * I1 / (2 * R1)
        B2 = mu0 * N2 * I2 / (2 * R2)
        B3 = mu0 * N3 * I3 / (2 * R3)
    
    # Calculate the force on the conductive object
        F_calculated = NAvg * K * B1 * B2 * B3 / d**2
    
    # Check for convergence
        if abs(F_calculated - F_desired) < tolerance:
            break
    
    # Adjust the currents based on the difference between the calculated and desired forces
        I1 += (F_desired - F_calculated) / (N1 * K * B2 * B3 / d**2)
        I2 += (F_desired - F_calculated) / (N2 * K * B1 * B3 / d**2)
        I3 += (F_desired - F_calculated) / (N3 * K * B1 * B2 / d**2)

# Print the final current values
    print("Current through coil 1: {:.2f} A".format(I1))
    print("Current through coil 2: {:.2f} A".format(I2))
    print("Current through coil 3: {:.2f} A".format(I3))
    pass

# Call the function and time it
start_time = time.time_ns()
my_function()
end_time = time.time_ns()

# Calculate the elapsed time
elapsed_time = end_time - start_time

# Print the elapsed time in seconds
print("Elapsed time: {:.2f} seconds".format(elapsed_time))