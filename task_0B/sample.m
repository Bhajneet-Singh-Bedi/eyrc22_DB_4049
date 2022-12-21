%%% some file bro...
%pkg load symbolic
syms x1 x2
x1_dot = -x1 + 2*x1^3 + x2;      
x2_dot = -x1 - x2;
% Evaluating the equilibrium points
[x_1, x_2] = solve(x1_dot, x2_dot);

% For finding jacobian matrix_type
jac_matrix = jacobian([x1_dot x2_dot]);

% Now substituting the eq points in jac_matrix using subs  ->  subs(evaluated, new, old)
solutions = [x_1, x_2];

%for i=1:size(sub_values)(1)
%  subs(jac_matrix, [x1 x2], [1])  
%endfor
[m,n]=size(solutions);
jacobian_matrices = subs(jac_matrix, x1, solutions)
%for i=1:m
%  jacobian_matrices = subs(jac_matrix, [x1,x2], solutions(i,:))
%end
jacobian_matrices = subs(jac_matrix, [x1,x2], solutions(2,:))
jacobian_matrices = subs(jac_matrix, [x1,x2], solutions(3,:))

%for i=1:size(sub_values)(1)
  %disp(i)
  %disp(sub_values(i))
  %subs(jac_matrix, [x1,x2], sub_values(i,:)) 
 % jacobian_matrices(:,:,i) = i*subs(jac_matrix, [x1,x2], sub_values(i,:)) 
%endfor 
