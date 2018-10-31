clear all;

n_axis = 6;

s = load('../results/move_traj.txt');
t = s(:,1);
move_q = s(:,2:n_axis+1);
move_dq = s(:,n_axis+2:2*n_axis+1);
move_ddq = s(:,2*n_axis+2:3*n_axis+1);
move_x = s(:,3*n_axis+2:3*n_axis+4);

traj_q = s(:,3*n_axis+9:4*n_axis+8);
traj_dq = s(:,4*n_axis+9:5*n_axis+8);
traj_ddq = s(:,5*n_axis+9:6*n_axis+8);
traj_x = s(:,6*n_axis+9:6*n_axis+11);

for i=1:3
  subplot(3,1,i), plot(t, move_q(:,i), t, traj_q(:,i));
  grid on;
end
