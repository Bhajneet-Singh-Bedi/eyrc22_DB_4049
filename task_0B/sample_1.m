%%% Testing custom find jacobian function...
syms x1 x2 m1 m2 g r u         % Define symbolic variables x1 and x1
%m1 = 7.5;
 % m2 = 7.51;
  %g = 9.8;
  %r = 0.2;
x1_dot = x2;       % Write the expressions for x1_dot and x2_dot
x2_dot = ((m1-m2)*g+u/r*(m2-m1))/(m1+m2);   % YOU CAN MODIFY THESE 2 LINES TO TEST OUT OTHER EQUATIONS
%%x1_dot = -x1 + 2*x1^3 + x2;       % Write the expressions for x1_dot and x2_dot
%%x2_dot = -x1 - x2;   % YOU CAN MODIFY THESE 2 LINES TO TEST OUT OTHER EQUATIONS

x1_dot == 0;
x2_dot == 0;
[x_1, x_2] = solve(x1_dot, x2_dot);
%x_1=double(x_1);
%x_2=double(x_2);
x_1
x_2

solutions = [x_1, x_2]
jacobian_matrices = {};
jac = jacobian([x1_dot x2_dot])
[m,n]=size(solutions);
for i=1:m
    jacobian_matrices{i}={subs(jac, [x1, x2],solutions(i,:))}  
end
jacobian_matrices;

stability = {};
  eigen_values = {};
  for k = 1:length(jacobian_matrices)
    matrix = jacobian_matrices(:,k);
    flag = 1;
    % Converting cell to matrix using cell2mat
    % Calculating eigen values using eig.
    lambda = eig(cell2mat(matrix));
    % Storing value in eigen_values cell.
    eigen_values(k)=lambda;
    % Separating real part using real.
    rl = real(lambda);
    [m,n]=size(rl);
    % finding if any value in it is positive, if yes then put flag=0.
    for i=1:m
      for j=1:n
        if rl(i,j)>0
          flag = 0;
        end
      end
    end

    if flag == 1
      fprintf("The system is stable for equilibrium point (%d, %d) \n",double(x_1(k)),double(x_2(k)));
      stability{k} = "Stable";
    else
      fprintf("The system is unstable for equilibrium point (%d, %d) \n",double(x_1(k)),double(x_2(k)));
      stability{k} = "Unstable";
    end
  end
