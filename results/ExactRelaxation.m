clear; close all;

% 1. Define symbolic variables and functions
syms x(t) a b x0

% 2. Define the Differential Equation (DE)
DE = diff(x, t) ==

a * log(x / b);

% 3. Solve the DE with the initial condition x(0) = x0
cond = x(0) == x0;
x_sol(t) = dsolve(DE, cond);

% Display the exact analytical solution in the command window
disp('Analytical Solution x(t):');
disp(x_sol(t));

%% 4. Numerical Example and Plotting
% Assign arbitrary physical values to the constants for visualization
a_val = 2; 
b_val = 5; 
x0_val = 1; 

% Substitute constants into the symbolic solution
x_numeric(t) = subs(x_sol, [a, b, x0], [a_val, b_val, x0_val]);

% Convert the symbolic expression to a function handle for plotting
x_fun = matlabFunction(x_numeric); % Use matlabFunction(x_numeric) in standard MATLAB

% Plot the trajectory over time
fplot(x_numeric, [0, 5], 'LineWidth', 2);
grid on;
xlabel('Time (t)');
ylabel('x(t)');
title('Solution Curve for dx/dt = a*log(x/b)');