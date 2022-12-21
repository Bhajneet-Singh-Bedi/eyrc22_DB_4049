# Finding A-lambda*I

# Those will be the poles or eigen values.

pkg load symbolic
syms lambda

I = eye(2)

#A = int32([0 1; 9.8/1 0])
A = int32([0 1; 0 0]);
A-lambda*I
eig(A-lambda*I)