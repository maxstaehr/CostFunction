function [ dist ] = distanceFromModelVector( v, pa )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x = v(1);
y = v(2);
z = v(3);

a = pa(1);
b = pa(2);
c = pa(3);

t1 = (x/a)^2;
t2 = (y/b)^2;
t3 = (z/c)^2;
sum = t1 + t2 + t3;
vp = v/sqrt(sum);
dist = v-vp;
end

