clc;
clear;
syms L a b

A =[1 10/(10+L^2/4) 10/(10+L^2);
    10/(10+L^2/4) 1 10/(10+L^2/4);
    10/(10+L^2) 10/(10+L^2/4) 1];

B = [1 1-a 1-a-b;1-a 1 1-a;1-a-b 1-a 1]

simplify(inv(A)*[1;1;1])
simplify(inv(B)*[1;1;1])