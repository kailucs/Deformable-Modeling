function k = inv2Kernel(a,b,param)
r = norm(a-b);
k = param/(param+(r*1000)^2);

end

