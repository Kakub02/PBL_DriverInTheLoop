import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class RealTimePlot:
    def __init__(self, 
                 lines_labels = ["line_1"],
                 title="Default Title", 
                 x_label="Default xLabel", 
                 y_label="Default yLabel", 
                 x_len=200, 
                 y_range=(-180, 180),
                 frequency_of_animation = 100):
        
        self.running = True
        # This is list of strings which are then used as a keys in y_data and lines dictionaries
        self.lines_labels = lines_labels
        # Length of x-axis that is number of points that plot is displaying
        self.x_len = x_len
        # Range of y-axis
        self.y_range = y_range
        # Lock to safely read data from and save data to y_data dictionary - needed because y_data is used by 2 threads
        self.data_lock = threading.Lock()
        # Function that will update data to display - must be implemented
        self.update_data_function = None

        # Time of one animation in miliseconds
        self.interval = 1000 / frequency_of_animation

        # Create figure for plotting
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(self.y_range)
        
        self.xs = list(range(0, self.x_len))

        # For each label we create a list of data initialized to 0. We will update the data in update_data_function
        self.y_data = {label: [0] * self.x_len for label in self.lines_labels}

        # For each label we create blank line. We will update the lines in animate function based on what current y_data[label] is
        self.lines = {label: self.ax.plot(self.xs, self.y_data[label], label=label)[0] for label in self.lines_labels}

        # Add labels and legend
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        self.ax.legend()

    def set_updating_function(self, update_data_function):
        self.update_data_function = update_data_function

    def animate(self, frame):
        with self.data_lock:
            for label in self.lines_labels:
                self.y_data[label] = self.y_data[label][-self.x_len:]  # slice the data so it has x_len points
                self.lines[label].set_ydata(self.y_data[label][:self.x_len])  # set the line y_data accordingly to y_data dictionary
        return tuple(self.lines[label] for label in self.lines_labels)
    
    def run(self):  
        # Start the data acquisition in a separate thread
        acquisition_thread = threading.Thread(target=self.update_data_function)
        acquisition_thread.start()

        # Start the animation
        ani = animation.FuncAnimation(self.fig, self.animate, interval=self.interval, blit=True)
        plt.show()

        # Wait for the data acquisition thread to finish
        acquisition_thread.join()

######################################################################################
# How to use?
# 1. Create labels (must be unique) accordingly to what you want to display
#     labels = ["first_plot", "second_plot"]
# 2. Implement update_data_function
# 
# def my_update_data_function(rtp_instance: RealTimePlot, number_of_iterations=1000):
#     frequency = 50
#     for loop_count in range(number_of_iterations):
#         start_time = round(time.time(), 4)
#
#         # Get your data from somewhere
#         first_plot_point = random.uniform(-10, 10)
#         second_plot_point = random.uniform(0, 20)
#
#         # Update data in current RealTimePlot instance
#         with rtp_instance.data_lock:
#             rtp_instance.y_data[rtp_instance.labels[0]].append(first_plot_point)
#             rtp_instance.y_data[rtp_instance.labels[1]].append(second_plot_point)
#
#         # Add your sleep time calculation here
#         sleep_time = 1 / frequency - (time.time() - start_time)  
#         if sleep_time > 0:
#             time.sleep(sleep_time)
# 
# 3. Create RealTimePlot object
# real_time_plot = RealTimePlot(labels, 'Your title', 'Samples', 'Values', 200, (-50, 50), 50)
#
# 4. Set the updating data function to your function
# real_time_plot.set_updating_function(lambda: my_update_data_function(real_time_plot, 1000))
#
# 5. Run the animation and have fun
# real_time_plot.run()
# 
######################################################################################