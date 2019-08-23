function error = RMSE2(input)
total = 0;
for i=1:1:size(input,1)
   total = total + abs(input(i));
end
error = sqrt(total/size(input,1));

end

