import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

# Run the code and Check in the Google CoLab

class Distribution:
    def __init__(self):

        self.x_values = np.arange(0, 31)
        # Calculate probabilities using the custom function
        self.probabilities = np.array([self.distinct_u_shaped_distribution(x) for x in self.x_values])
        
        self.probabilities = self.probabilities / np.sum(self.probabilities) # Normalize the probabilities to sum to 1

    def distinct_u_shaped_distribution(self, x, min_val=0, max_val=30):
        
        high_prob = 0.05 # Define high probability for edge ranges
        low_prob = 0.001 # Define lower probability for the middle range

        if min_val <= x <= 2 or 28 <= x <= max_val:
            return high_prob
        else:
            return low_prob

    def plot_distribution(self):

        plt.figure(figsize=(10, 6))
        plt.bar(self.x_values, self.probabilities)
        plt.xlabel('Value')
        plt.ylabel('Probability')
        plt.title('Distinct U-Shaped Distribution')
        plt.show()

class RandomSpline:

    seed_list = [1301, 1303, 1305, 1306, 1307, 1310, 1312, 1315, 1418, 1419]

    def __init__(self, probabilities, min=0, max=30, seed=0, v=0.06, hz=10):
        self.seed = seed
        np.random.seed(RandomSpline.seed_list[seed])
        self.min = min
        self.max = max
        self.v = v
        self.hz = hz
        self.outs = []
        self.probabilities = probabilities

    def make_spline(self, point_num=4, repeat=5):
        x_list = list(range(self.min, self.max + 1))
        y_list = list(range(self.min, self.max + 1))

        for _ in range(repeat):
            x = np.random.choice(x_list, point_num, list(self.probabilities))
            y = np.random.choice(y_list, point_num, list(self.probabilities))
            # print(x)
            # print(y)

            l=len(x)

            t=np.linspace(0,1,l-2,endpoint=True)
            t=np.append([0,0,0],t)
            t=np.append(t,[1,1,1])

            tck=[t,[x,y],3]
            u3=np.linspace(0,1,(max(l*2,50)),endpoint=True)
            out = interpolate.splev(u3,tck)

            # Check Spline Length and Apply steps
            dx, dy = interpolate.splev(u3, tck, der=1) # Calculate the derivative of the spline (dx/dt, dy/dt)
            ds = np.sqrt(dx**2 + dy**2) # Calculate the differential length along the curve
            spline_length = np.sum(ds) * (u3[1] - u3[0]) # Integrate to get the total length of the spline
            print(f"Spline Length: {spline_length}")
            total_time = spline_length*0.001 /self.v # Check unit (mm -> m) !!!!
            steps = int(np.ceil(total_time * self.hz)) # round up

            # Remap the spline
            u3=np.linspace(0,1,(max(l*2,steps)),endpoint=True) # steps
            out = interpolate.splev(u3,tck)

            self.append_list(np.array(out))
            print("size of outs: ", np.shape(self.outs))
        return np.stack(self.outs) if self.outs else np.array([])

    def append_list(self, array):
        if isinstance(array, np.ndarray):
            self.outs.append(array)
        else:
            raise ValueError("Input array must be a NumPy array")

    def plot_spline(self):
        fig = plt.figure(figsize=(10, 6))
        for i in range(len(self.outs)):
            # plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
            # plt.plot(x[0],y[0],marker='o',markerfacecolor='red')
            # plt.plot(x[-1],y[-1],marker='o',markerfacecolor='red')
            # plt.plot(x,y,'ro',label='Control points only')
            plt.plot(self.outs[i][0],self.outs[i][1],'b',linewidth=2.0,label='B-spline curve')
        # plt.legend(loc='best')
        plt.axis([self.min, self.max, self.min, self.max])
        plt.title('Cubic B-spline curve evaluation')
        plt.show()

    def save_file(self):
        for i in range(len(self.outs)):
            np.savetxt(f"./random_spline_{self.seed}_{i}.csv", self.outs[i].T, delimiter=",", fmt="%.2f")

    def get_scaled_outs(self):
        scaled_outs = [out * 0.001 for out in self.outs] # Multiply all values in self.outs by 0.001 (mm -> m) Check Unit
        return np.stack(scaled_outs) if scaled_outs else np.array([])  


def main():

    dist = Distribution()
    # dist.plot_distribution()

    random_sp = RandomSpline(dist.probabilities, seed = 0)
    random_sp.make_spline()
    # random_sp.plot_spline()
    random_sp.save_file()
    print(np.shape(random_sp.outs))

if __name__ == "__main__":
	main()