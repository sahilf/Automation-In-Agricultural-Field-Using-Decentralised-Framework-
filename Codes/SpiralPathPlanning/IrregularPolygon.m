N= 5;                                 % Number Of Sides To Polygon
a = sort(rand(N,1))*2*pi;
r = randi(9, N, 1);
x = cos(a).*r;
y = sin(a).*r;
figure(1)
plot([x;x(1)],[y;y(1)]  )
x
x(end+1) = x(1)
y
y(end+1) = y(1)
points = [x,y]
