function f = forceModel(x,C)
    if x < C(1)
        f = C(2);
    else
        f = C(2)+ C(3)*(x-C(1))+C(4)*(x-C(1))^2+C(5)*(x-C(1))^3+C(6)*(x-C(1))^4;
    end
end
