function A = diluteData(B,N)
%select only 1/N of the rows. (Not randomly)
selectRow = 1:N:size(B,1);
A = B(selectRow,:);

end

