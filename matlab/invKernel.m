function k = invKernel(a,b,param)
r = norm(a-b);
k = param/(param+r);

end

 