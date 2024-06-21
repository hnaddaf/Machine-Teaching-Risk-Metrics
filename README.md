# Machine-Teaching-Risk-Metrics
This project measures the distance between a demonstrated path and a reproduced path using different types of risk metrics. It then finds the optimal demonstration among various demos that minimizes the error based on the given metrics.

Features
Measures distance between demonstrated and reproduced paths.
Utilizes different risk metrics for error calculation:
Root Mean Square Error (RMSE)
Normalized Mean Square Error (NMSE)
Absolute Error Loss (AEloss)
Huber Loss
Mean Squared Logarithmic Error (MSLE)
Quantile Loss
Finds the optimal demonstration path that minimizes error based on the given metrics.

Requirements
MATLAB

Usage

Open MATLAB.
Navigate to the project directory.
Run Experiment_1_d script.

The script will:
Generate multiple demonstration paths.
Calculate various error metrics for each path.
Identify and display the optimal demonstration path that minimizes the error based on the specified metrics.

Code Overview
GetLinearPath.m
Generates a linear motion path using a graphical slider input.
Records the position over time and plots the path.

getdemo.m
Simulates a demonstration of a two-link robotic arm using PD control.
Calculates and visualizes the position and control actions over time.

Experminment_1_d.m
Sets up the system parameters and error metrics.
Generates multiple demonstration paths using getdemo.
Calculates various error metrics for each path.
Identifies the optimal demonstration path based on the minimum error for each metric.
Displays and plots the results.
Customization

Path Generation: Modify getdemo.m to generate different types of paths or use different control strategies.
Error Metrics: Add or modify error metrics in Experminment_1_d.m to suit your specific needs.

