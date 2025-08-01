import numpy as np
import matplotlib.pyplot as plt

def generate_path(hz, velocity, shape='square'):
    """
    Generate a path with dynamic time steps and rest time at corners.

    Parameters:
        hz (float): Command frequency in Hz (e.g., 10).
        velocity (float): Robot velocity in meters per second (e.g., 0.06 m/s).
        shape (str): Shape of the path ('square', 'triangle', 'line').

    Returns:
        np.ndarray: Generated path as a numpy array of shape (2, N).
    """
    rest_time = 0.5  # Rest time at corners in seconds

    # Define points for different shapes
    if shape == 'square':
        points = np.array([[0, 0.03, 0.03, 0, 0], [0.03, 0.03, 0, 0, 0.03]])  # Square
    elif shape == 'squareCCW':
        points = np.array([[0, 0, 0.03, 0.03, 0], [0.03, 0, 0, 0.03, 0.03]])  # Square CCW
    elif shape == 'triangle':
        points = np.array([[0, 0.03, 0.015, 0], [0, 0, 0.03, 0]])  # Triangle
    elif shape == 'diagonal1':
        points = np.array([[0, 0.03], [0.03, 0]])  # Line
    elif shape == 'diagonal2':
        points = np.array([[0.03, 0], [0.03, 0]])  # Line
    elif shape == 'diagonal3':
        points = np.array([[0.03, 0], [0, 0.03]])  # Line
    elif shape == 'diagonal4':
        points = np.array([[0, 0.03], [0, 0.03]])  # Line
    else:
        raise ValueError("Invalid shape. Use 'square', 'triangle', or 'diagonal'.")

    # Rest points at corners
    rest_steps = int(rest_time * hz)
    path = []

    # Generate segments for each pair of consecutive points
    for i in range(points.shape[1] - 1):
        # Add rest points at the start of the segment (corners)
        if i > 0:
            corner_x, corner_y = points[:, i]
            rest_segment = np.tile([[corner_x], [corner_y]], (1, rest_steps))
            path.append(rest_segment)

        # Calculate the distance and time for the segment
        x_start, x_end = points[0, i], points[0, i + 1]
        y_start, y_end = points[1, i], points[1, i + 1]
        distance = np.sqrt((x_end - x_start) ** 2 + (y_end - y_start) ** 2)
        time = distance / velocity
        time_steps = max(1, int(time * hz))

        # Interpolate the segment
        x_segment = np.linspace(x_start, x_end, time_steps, endpoint=True)
        y_segment = np.linspace(y_start, y_end, time_steps, endpoint=True)
        path.append(np.stack((x_segment, y_segment)))

    # Combine all segments into a single array
    path = np.hstack(path)

    return path


def generate_circle_path(hz, velocity, radius=0.015, direction='CW'):
    """
    Generate a circular path.

    Parameters:
        hz (float): Command frequency in Hz (e.g., 10).
        velocity (float): Robot velocity in meters per second (e.g., 0.06 m/s).
        radius (float): Radius of the circle in meters (default: 0.015).
        center (tuple): Center of the circle as (x, y) (default: (0.015, 0.015)).
        direction (str): 'CW' for clockwise or 'CCW' for counterclockwise.

    Returns:
        np.ndarray: Circle path as a numpy array of shape (2, N).
    """
    center=(0.015, 0.015)

    # Calculate the time step
    circumference = 2 * np.pi * radius
    total_time = circumference / velocity
    total_steps = int(total_time * hz)

    # Generate angles for the circle
    if direction == 'CW':
        angles = np.linspace(0, -2 * np.pi, total_steps, endpoint=True)
    elif direction == 'CCW':
        angles = np.linspace(0, 2 * np.pi, total_steps, endpoint=True)
    else:
        raise ValueError("Invalid direction. Use 'CW' or 'CCW'.")

    # Compute the points on the circle
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)

    return np.stack((x, y))



def main():
    # Parameters
    velocity = 0.04  # Robot velocity in meters per second
    frequency = 20   # Command frequency in Hz
    shape = 'diagonal4'  # Shape of the path ('square', 'triangle', 'line')
    

    # Generate the path
    path = generate_path(hz=frequency, velocity=velocity, shape=shape)
    # path = generate_circle_path(hz=frequency, velocity=velocity, radius=0.015, direction='CW')

    print("Path Shape:", path.shape)
    print(path)

    # Visualize the path
    plt.figure(figsize=(6, 6))
    plt.plot(path[0, :], path[1, :], 'o-', label='Path')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Path with Rest Time at Corners')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

    plt.figure(figsize=(6,6))
    plt.plot(path[0, :])
    plt.plot(path[1, :])
    plt.show()

if __name__ == "__main__":
  main()