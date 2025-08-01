import numpy as np
from bspline import Distribution, RandomSpline

class PathGenerator:
    def __init__(self, bspline_array, x=0.0225, y=0.50000, z=0.440, v=0.06, hz=10):
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

        # B Spline
        self.out = bspline_array

        self.x_i = self.out[0][0]
        self.x_e = self.out[0][-1]
        self.y_i = self.out[1][0]
        self.y_e = self.out[1][-1]

    def generate_init(self):
        path_array = np.array([[self.x_i, self.y_i, self.height, 0, 0, 0] for _ in range(30)]) # 30/10hz = 3 seconds
        return path_array

    def generate_path1(self):
        # Path 1: z + height -> z
        total_time = self.height /self.v # 0.1m / v along z axis
        steps = int(np.ceil(total_time * self.hz)) # round up
        z_values = np.linspace(self.height, 0, steps)
        path_array = np.array([[self.x_i, self.y_i, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[self.x_i, self.y_i, z_values[-1], 0, 0, 0] for _ in range(5)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path2(self): # B Spline

        path_array = np.array([[self.out[0][i], self.out[1][i], 0, 0, 0, 0] for i in range(len(self.out[0]))])
        pause_array = np.array([[self.x_e, self.y_e, 0, 0, 0, 0] for _ in range(5)]) # 0.5 second pause
        return np.concatenate((path_array, pause_array), axis=0)

    def generate_path3(self):
        # Path 3: z -> z + height
        total_time = self.height / self.v
        steps = int(np.ceil(total_time * self.hz))
        z_values = np.linspace(0, self.height, steps)
        path_array = np.array([[self.x_e, self.y_e, z, 0, 0, 0] for z in z_values])
        pause_array = np.array([[self.x_e, self.y_e, z_values[-1], 0, 0, 0] for _ in range(5)]) # 0.5 second pause
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
    velocity = 0.06
    frequency = 10

    # B Spline OPTION !!
    seed = 0
    
    dist = Distribution()
    random_sp = RandomSpline(dist.probabilities, seed = seed, v=velocity, hz=frequency)
    random_sp.make_spline()
    outs = random_sp.get_scaled_outs()
    
    for index in range(5):
        out = outs[index]
        path = PathGenerator(out, x=-0.090, y=0.61800, z=0.30000, v=velocity, hz=frequency)
        path.save(f"./path_{seed * 10 + index}.csv") # 0, 1, 2, 3, 4



if __name__ == "__main__":
	main()