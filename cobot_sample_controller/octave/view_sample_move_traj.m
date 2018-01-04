clear all;

n_axis = 5;

s = load('../results/move_traj.txt');
t = s(:,1);
move_q = s(:,2:n_axis+1);
move_x = s(:,n_axis+2:n_axis+4);

traj_q = s(:,n_axis+9:n_axis*2+8);
traj_x = s(:,n_axis*2+9:n_axis*2+11);

for i=1:3
  subplot(3,1,i), plot(t, move_q(:,i), t, traj_q(:,i));
  grid on;
end
