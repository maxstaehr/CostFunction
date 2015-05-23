function [V F C] = generateBox(dim, c)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
V = [0 0 0;
       dim(1) 0 0;
       dim(1) dim(2) 0;
       0 dim(2) 0
       0 0 dim(3);
       dim(1) 0 dim(3);
       dim(1) dim(2) dim(3);
       0 dim(2) dim(3)];
F = [1 2 3;
       1 4 3;
       5 6 7;
       5 8 7;
       1 2 6;
       6 5 1;
       2 3 7;
       7 6 2;
       1 8 4;
       1 5 8;
       3 4 7;
       4 8 7];
C = repmat(c,numrows(F),1);

end

