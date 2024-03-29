1;
Function_File;
pkg load symbolic      # Load the octave symbolic library
syms x1 x2 m L u g            # Define symbolic variables x1 and x1
##x1_dot = -x1 + 2*x1^3 + x2;       # Write the expressions for x1_dot and x2_dot
##x2_dot = -x1 - x2;   # YOU CAN MODIFY THESE 2 LINES TO TEST OUT OTHER EQUATIONS

x1_dot = x2;       # Write the expressions for x1_dot and x2_dot
x2_dot = (-(g/L)*sin(x1)) + (u/(m*(L^2)));   # YOU CAN MODIFY THESE 2 LINES TO TEST OUT OTHER EQUATIONS
[x_1 x_2 jacobians eigen_values stability] = main_function(x1_dot, x2_dot);