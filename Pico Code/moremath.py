import math

# Constants
mu0 = 4 * math.pi * 1e-7  # Vacuum permeability
sigma = 5.96e7  # Electrical conductivity of copper
r = 0.01  # Radius of copper sphere
d = 0.2  # Distance from sphere to electromagnets
B_target = 1  # Target magnetic field strength

# Function to calculate magnetic field strength at a point
def calc_B(x, y, z, I1, I2, I3):
    # Calculate distance from each electromagnet
    r1 = math.sqrt((d)**2 + y**2 + z**2)
    r2 = math.sqrt(x**2 + (d)**2 + z**2)
    r3 = math.sqrt(x**2 + y**2 + (d)**2)
    
    # Calculate magnetic field contribution from each electromagnet
    B1 = mu0 * I1 / (4 * math.pi * r1**2)
    B2 = mu0 * I2 / (4 * math.pi * r2**2)
    B3 = mu0 * I3 / (4 * math.pi * r3**2)
    
    # Calculate total magnetic field strength
    B = math.sqrt(B1**2 + B2**2 + B3**2)
    
    # Calculate eddy currents contribution
    B_eddy = -2 * sigma * r**2 * B / (3 * mu0 * d)
    
    # Return total magnetic field strength
    return B + B_eddy

# Function to calculate currents for the electromagnets
def calc_currents():
    # Initialize currents to zero
    I1, I2, I3 = 0, 0, 0
    
    # Set initial guess for I1
    I1 = B_target * (4 * math.pi * (d - r)**2) / (mu0 * 3 * math.sqrt(3))
    
    # Use gradient descent to iteratively improve currents
    for i in range(100):
        # Calculate magnetic field strength at center of sphere
        B = calc_B(0, 0, 0, I1, I2, I3)
        
        # Calculate gradient of magnetic field with respect to currents
        dB_dI1 = (calc_B(1e-6, 0, 0, I1 + 1e-6, I2, I3) - B) / 1e-6
        dB_dI2 = (calc_B(0, 1e-6, 0, I1, I2 + 1e-6, I3) - B) / 1e-6
        dB_dI3 = (calc_B(0, 0, 1e-6, I1, I2, I3 + 1e-6) - B) / 1e-6
        
        # Update currents using gradient descent
        I1 -= 0.1 * dB_dI1
        I2 -= 0.1 * dB_dI2
        I3 -= 0.1 * dB_dI3
        
    return I1, I2, I3

# Calculate currents for electromagnets and print result
I1, I2, I3 = calc_currents()
print(I1, I2, I3)