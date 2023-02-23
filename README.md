![Alt text](https://github.com/sfowzia1001/kalman_cpp/blob/main/media/mahalanobis_distance.png?raw=true "Kalman_cpp")

# Kalman_cpp

This program assumes that the system being modeled is linear and that both the system dynamics and measurement models are known.

This implementation simulates a simple 1D linear system and applies the Kalman filter to estimate the system's state. The KalmanFilter class has methods for initializing the filter (init), running one iteration of the filter given a control input and measurement (update), and retrieving the current state estimate (getState).

The KalmanFilter object is initialized with an initial state of all zeros and an identity covariance matrix. Then, the system is simulated for 10 time steps, with each iteration calling the update method of the KalmanFilter object to estimate the system state based on the control input and measurement. The current state estimate is printed to the console at each time step

After each time step, the KalmanFilter object's state estimate, innovation covariance, innovation vector, innovation Mahalanobis distance, and log-likelihood of the measurement are printed to the console. This provides additional insight into the filter's operation and can help with debugging and tuning the filter.

## Instructions

- Clone this repository in a desired location with `git clone https://github.com/sfowzia/kalman_cpp`
- Change directory into Kalman_cpp with `cd kalman_cpp`
- Make a build directory with `mkdir build && cd build`
- Compile with CMake with `cmake ..`
- Make the executable with `make`
- Run the example with `./kalman_cpp`

If you have any questions, please feel free to contact me:
sfowzia00@gmail.com
