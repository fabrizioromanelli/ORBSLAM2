x = CameraTrajectory3.y;
y = CameraTrajectory3.z;

i = 1;
flag = true;

% idx = find(y > 25);
c = x(end);

while flag
  if i > size(y,1)
    flag = false;
  else
    x(i) = x(i) - c*(i/size(x,1))^3;
    i = i + 1;
  end
end

a = 680; b = 1350;

X = x(a:b)-x(a); Y = y(a:b)-y(a);

figure(2); plot(X*2,Y); grid on;