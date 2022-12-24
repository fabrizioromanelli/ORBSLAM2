% Read and plot a dataset
dataset = 'isl_01/';
b = readtable(strcat(dataset,'CameraTrajectory.txt'));

x_vo = b.Var2;
y_vo = b.Var4;
z_vo = -b.Var3;

figure(1); grid on; hold on; plot(x_vo,y_vo,'k.');legend('ORB-SLAM2');
% figure(2); grid on; hold on; plot3(x_vo,y_vo,z_vo,'k.'); legend('ORB-SLAM2');