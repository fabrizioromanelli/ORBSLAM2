% Read, adjust and synchronize an EuRoC dataset
dataset = 'MH_03/';
a = readtable(strcat(dataset,'dataSGTE.csv'));
b = readtable(strcat(dataset,'CameraTrajectory.txt'));

t_gt = a.x_timestamp*10^-9;
x_gt = a.p_RS_R_x_m_;
y_gt = a.p_RS_R_y_m_;
z_gt = a.p_RS_R_z_m_;

% First pose for spatial alignment
x0   = a.p_RS_R_x_m_(1);
y0   = a.p_RS_R_y_m_(1);
z0   = a.p_RS_R_z_m_(1);
Q    = quaternion(a.q_RS_w__(1), a.q_RS_x__(1), a.q_RS_y__(1), a.q_RS_z__(1));

t_sync = 46; % for MH_03

t_vo  = b.Var1(t_sync:end);
x0_vo = -b.Var3(t_sync:end); % x = -z
y0_vo = b.Var2(t_sync:end); % y = x
z0_vo = b.Var4(t_sync:end); % z = y

x_vo = zeros(size(x0_vo,1),1); y_vo = zeros(size(y0_vo,1),1); z_vo = zeros(size(z0_vo,1),1);

for i=1:size(x0_vo,1)
  inP = [x0_vo(i) y0_vo(i) z0_vo(i)];
  ouP = rotatepoint(Q, inP);
  x_vo(i) = ouP(1);
  y_vo(i) = ouP(2);
  z_vo(i) = ouP(3);
end

x_vo = x_vo + x0;
y_vo = y_vo + y0;
z_vo = z_vo + z0;

figure(1); plot(x_gt,y_gt,'r'); grid on; hold on; plot(x_vo,y_vo,'k');legend('Ground truth','ORB-SLAM2');
figure(2); plot3(x_gt,y_gt,z_gt,'r'); grid on; hold on; plot3(x_vo,y_vo,z_vo,'k'); legend('Ground truth','ORB-SLAM2');