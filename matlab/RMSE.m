function error = RMSE(input)
total = 0;
for i=1:1:size(input,1)
   total = total + input(i)^2;
end
error = sqrt(total/size(input,1));

end

