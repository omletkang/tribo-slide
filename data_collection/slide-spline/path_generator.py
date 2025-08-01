import numpy as np
from path_transform import extract_path_coordinates

class PathGenerator:
    def __init__(self, svg_file, x=0.0225, y=0.50000, z=0.440, v=0.04, hz=20):
        self.x = x
        self.y = y
        self.z = z
        self.rx = 1.220
        self.ry = 1.176
        self.rz = 1.184

        self.height = 0.1 # 0.1m = 100mm = 10cm
        self.distance = 0.1

        self.v = v
        self.hz = hz
        self.base_array = np.array([self.x, self.y, self.z, self.rx, self.ry, self.rz])

        # Extract path coordinates from SVG
        self.coordinates = extract_path_coordinates(svg_file, velocity=self.v, hz=self.hz)

        self.x_i = self.coordinates[0][0]
        self.x_e = self.coordinates[-1][0]
        self.y_i = self.coordinates[0][1]
        self.y_e = self.coordinates[-1][1]

    def generate_init(self):
        path_array = np.array([[self.x_i, self.y_i, self.height, 0, 0, 0] for _ in range(60)]) # 60/20hz = 3 seconds
        return path_array

    def generate_path1(self):
        # Path 1: z + height -> z
        total_time = self.height / self.v # 0.1m / v along z axis
        steps = int(np.ceil(total_time * self.hz)) # round up
        z_values = np.linspace(self.height, 0, steps)
        path_array = np.array([[self.x_i, self.y_i, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[self.x_i, self.y_i, z_values[-1], 0, 0, 0] for _ in range(10)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path2(self):
        # Path 2: Extracted SVG path coordinates
        path_array = np.array([[x, y, 0, 0, 0, 0] for x, y in self.coordinates])
        pause_array = np.array([[self.x_e, self.y_e, 0, 0, 0, 0] for _ in range(10)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path3(self):
        # Path 3: z -> z + height
        total_time = self.height / self.v
        steps = int(np.ceil(total_time * self.hz))
        z_values = np.linspace(0, self.height*0.8, steps)
        path_array = np.array([[self.x_e, self.y_e, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[self.x_e, self.y_e, z_values[-1], 0, 0, 0] for _ in range(10)]) # 0.5 second pause
        return path_array

    def generate_path4(self):
        # Path 4 : Returning
        distance = np.sqrt((self.x_i-self.x_e)**2 + (self.y_i-self.y_e)**2)
        distance = max(0.02, distance) # Minimum length
        total_time = distance / self.v
        steps = int(np.ceil(total_time * self.hz))
        x_values = np.linspace(self.x_e, self.x_i, steps)
        y_values = np.linspace(self.y_e, self.y_i, steps)
        path_array = np.array([[x, y, self.height, 0, 0, 0] for x, y in zip(x_values, y_values)])
        return path_array

    def generate_total_path(self):
        path_init = self.generate_init()
        path1 = self.generate_path1()
        path2 = self.generate_path2()
        path3 = self.generate_path3()
        path4 = self.generate_path4()

        full_path = np.concatenate((path_init, path1, path2, path3, path4), axis=0)

        # Add the base array element-wise to each row
        full_path_with_base = full_path + self.base_array
        return full_path_with_base

    def save(self, filename="./test.csv"):
        path_array = self.generate_total_path()
        np.savetxt(filename, path_array, delimiter=",", fmt='%.5f',comments='')


def main():
    velocity = 0.04 # in m/s
    frequency = 20 # in Hz

    # Example SVG file
    for idx in range(1, 21):
        idx = str(idx) # int to string
        svg_file = f"path_sliding_v1-{idx.zfill(2)}.svg"  # path_sliding_v1-01.svg

        path = PathGenerator(svg_file, x=-0.095, y=0.57500, z=0.24130, v=velocity, hz=frequency)
        path.save(f"./path_{svg_file[-6:-4]}.csv") # path_01


if __name__ == "__main__":
    main()
