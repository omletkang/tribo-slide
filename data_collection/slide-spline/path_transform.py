from svg.path import parse_path
from xml.dom import minidom

def extract_path_coordinates_basic(svg_file):
    # Parse the SVG file
    doc = minidom.parse(svg_file)
    path_strings = [path.getAttribute('d') for path in doc.getElementsByTagName('path')]
    doc.unlink()
    
    coordinates = []
    
    # Iterate through each path in the file
    for path_string in path_strings:
        path = parse_path(path_string)
        
        # Sample points on the path
        for i in range(100):  # Adjust sampling resolution here
            point = path.point(i / 100)
            coordinates.append((point.real, point.imag))
    
    return coordinates


def extract_path_coordinates(svg_file, velocity=0.04, hz=20):
    """
    Extract coordinates from an SVG path, adjusting timesteps based on velocity and frequency.

    Args:
        svg_file (str): Path to the SVG file.
        velocity (float): Desired velocity in meters per second.
        hz (int): Sampling frequency in Hz.

    Returns:
        list: List of (x, y) coordinates in meters.
    """
    # Parse the SVG file
    doc = minidom.parse(svg_file)
    path_strings = [path.getAttribute('d') for path in doc.getElementsByTagName('path')]
    doc.unlink()

    coordinates = []

    for path_string in path_strings:
        path = parse_path(path_string)

        # Calculate total path length
        total_length = sum(segment.length() for segment in path)
        
        # Convert total length from mm to meters
        total_length_m = total_length / 1000.0

        # Calculate total time and timesteps
        total_time = total_length_m / velocity  # in seconds
        num_timesteps = int(total_time * hz)

        # Sample points along the path
        for i in range(num_timesteps):
            point = path.point(i / num_timesteps)
            coordinates.append((point.real / 1000.0, point.imag / 1000.0))  # Convert mm to m

    return coordinates


def main():
    svg_file = "path_sliding_v1-06.svg" 
    velocity = 0.04  # in m/s
    hz = 20  # in Hz

    coordinates = extract_path_coordinates(svg_file, velocity, hz)

    print("Extracted Coordinates:")
    for x, y in coordinates:
        print(f"{x:.3f}, {y:.3f}")

if __name__ == "__main__":
    main()
