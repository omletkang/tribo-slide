import numpy as np

class PathGenerator:
    def __init__(self, x=0.0225, y=0.50000, z=0.440, v=0.06, hz=10, repeat=1):
        self.x = x
        self.y = y
        self.z = z
        self.rx = 1.220
        self.ry = 1.176
        self.rz = 1.184

        self.height = 0.1
        self.distance = 0.1

        self.v = v
        self.hz = hz
        self.repeat = repeat
        self.base_array = np.array([self.x, self.y, self.z, self.rx, self.ry, self.rz])

    def generate_init(self):
        path_array = np.array([[0, 0, self.height, 0, 0, 0] for _ in range(30)]) # 30/10hz = 3 seconds
        return path_array

    def generate_path1(self):
        # Path 1: z + 0.1 -> z
        total_time = self.height /self.v # 0.1m / v along z axis
        steps = int(np.ceil(total_time * self.hz)) # round up
        z_values = np.linspace(self.height, 0, steps)
        path_array = np.array([[0, 0, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[0, 0, z_values[-1], 0, 0, 0] for _ in range(5)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path2(self):
        # Path 2: y -> y + 0.1
        total_time = self.distance / self.v
        steps = int(np.ceil(total_time * self.hz))
        y_values = np.linspace(0, 0 + self.distance, steps)
        path_array = np.array([[0, y, 0, 0, 0, 0] for y in y_values])
        pause_array = np.array([[0, y_values[-1], 0, 0, 0, 0] for _ in range(5)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path3(self):
        # Path 3: z -> z + 0.1
        total_time = self.height / self.v
        steps = int(np.ceil(total_time * self.hz))
        z_values = np.linspace(0, self.height, steps)
        path_array = np.array([[0, 0.1, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[0, 0.1, z_values[-1], 0, 0, 0] for _ in range(5)]) # 0.5 second pause
        return path_array

    def generate_path4(self):
        # Path 4: y + 0.1 -> y
        total_time = self.distance / self.v
        steps = int(np.ceil(total_time * self.hz))
        y_values = np.linspace(self.distance, 0, steps)
        path_array = np.array([[0, y, self.height, 0, 0, 0] for y in y_values])
        return path_array

    def generate_total_path(self):
        path_init = self.generate_init()
        path1 = self.generate_path1()
        path2 = self.generate_path2()
        path3 = self.generate_path3()
        path4 = self.generate_path4()
        # Repeat path
        repeat_paths = []
        for _ in range(self.repeat):
            repeat_paths.extend([path1, path2, path3, path4])
        
        full_repeated_path = np.concatenate(repeat_paths, axis=0)
        full_path = np.concatenate((path_init, full_repeated_path), axis=0)

        # Add the base array element-wise to each row
        full_path_with_base = full_path + self.base_array
        return full_path_with_base

    def save(self, filename="./test.csv"):
        path_array = self.generate_total_path()
        np.savetxt(filename, path_array, delimiter=",", fmt='%.5f',comments='')


def main():
    path = PathGenerator()
    path.save("./path_test.csv")


if __name__ == "__main__":
	main()