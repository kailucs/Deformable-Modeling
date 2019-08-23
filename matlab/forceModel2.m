function f = forceModel2(x,C)
    if x < C(1)
        f = 0 ;
    else
        f = C(2)*(x-C(1))+C(3)*(x-C(1))^2+C(4)*(x-C(1))^3+C(5)*(x-C(1))^4;
    end
end
