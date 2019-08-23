function y = learningKernelFunc(x)
%convert linear input data to higher-dimensional nonlinear vector
%accepts 3D, which is the position
%each x is a row vector

y = [x x(1)^2 x(2)^2 x(3)^2 x(1)*x(2) x(2)*x(3) x(1)*x(3)];

end

