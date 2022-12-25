% Read and plot a dataset
dataset = 'isl_01/';
b = readtable(strcat(dataset,'CameraTrajectory1500feats.txt'));

x_vo = b.Var2;
y_vo = b.Var4;
z_vo = -b.Var3;

% Get the orientation displacement after few steps, supposing we are moving
% in a straight direction.
idx_max = 200;

X = 1; Y = 2; Z = 3;
vec_max = [x_vo(idx_max) y_vo(idx_max) z_vo(idx_max)];
vec_x   = [x_vo(idx_max) y_vo(idx_max) 0];

% Compute the orientation between vec_max and vec_x
q_xyz = cross(vec_x, vec_max);
q_w = sqrt((norm(vec_x)^2) * (norm(vec_max)^2)) + dot(vec_x, vec_max);
Q = quaternion(q_w, q_xyz(X), q_xyz(Y), q_xyz(Z));
Q = Q.normalize;

for i=1:size(x_vo,1)
  inP = [x_vo(i) y_vo(i) z_vo(i)];
  ouP = rotatepoint(Q.conj, inP);
  x_vo(i) = ouP(1);
  y_vo(i) = ouP(2);
  z_vo(i) = ouP(3);
end

figure(1); grid on; hold on; plot(x_vo,y_vo,'k.');legend('ORB-SLAM2'); xlabel('X'); ylabel('Y');
figure(2); grid on; hold on; plot(y_vo,z_vo,'k.');legend('ORB-SLAM2'); xlabel('Y'); ylabel('Z');
figure(3); grid on; hold on; plot3(x_vo,y_vo,z_vo,'k.'); legend('ORB-SLAM2'); xlabel('X'); ylabel('Y'); zlabel('Z');