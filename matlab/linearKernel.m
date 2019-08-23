function k = linearKernel(a,b,param)
r = norm(a-b);
k = 1-r/param;

end

