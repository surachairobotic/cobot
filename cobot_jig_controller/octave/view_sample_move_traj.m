clear all;

n_axis = 6;

s = load('../results/move_traj.txt');
s2 = load('../results/move_vel_xyz.txt');

t = s(:,1);
move_q = s(:,2:n_axis+1);         % current jt angle 2-7 [joint_state.position]
move_q_dot = s(:,n_axis+2:n_axis*2+1);  % current jt velo 8-13 [joint_state.velocity]
move_x = s(:,n_axis*3+2:n_axis*3+4);  % current cartesian position 20-22 [pose2.position, pose2.orientation]

%traj_q = s(:,n_axis+9:n_axis*2+8);
%traj_x = s(:,n_axis*2+9:n_axis*2+11);

traj_q = s(:,n_axis*3+9:n_axis*4+8);  % target jt angle 27-32 [p.positions]
traj_q_dot = s(:,n_axis*4+9:n_axis*5+8);  % target jt vel 33-38 [p.velocities]
traj_acc = s(:,n_axis*5+9:n_axis*6+8);  % target jt acc 39-44 [p.accelerations]
traj_x = s(:,n_axis*6+9:n_axis*6+11); % target cartesian position 45-47 [pose.position, pose.orientation]

vx = s2(:,1);
vy = s2(:,2);
vz = s2(:,3);
figure(11),subplot(3,1,1),plot(vx);
grid on;
figure(11),subplot(3,1,2),plot(vy);
grid on;
figure(11),subplot(3,1,3),plot(vz);
grid on;


if(true)
  figure(1)
  for i=1:3
    plot(t, move_x(:,1), t, traj_x(:,1));
  end
  figure(2)
  for i=1:3
    plot(t, move_x(:,2), t, traj_x(:,2));
  end
  figure(3)
  for i=1:3
    plot(t, move_x(:,3), t, traj_x(:,3));
  end
end

if(true)
figure(4)
plot(move_x(:,1), move_x(:,2), traj_x(:,1), traj_x(:,2));
end;

if(false)
  figure(5)
  for i=1:3
    subplot(3,1,i), plot(t, move_q(:,i), t, traj_q(:,i));
    grid on;
    legend('move','traj');
    ylabel(['joint ',num2str(i)]);
    xlabel('time (s)');
    title('joint angle');
  end

  figure(6)
  for i=1:3
    j=i+3;
    subplot(3,1,i), plot(t, move_q(:,j), t, traj_q(:,j));
    grid on;
    legend('move','traj');
    ylabel(['joint ',num2str(j)]);
    xlabel('time (s)');
    title('joint angle');
  end
end
  
if(true)
  figure(7),plot(t,move_q_dot(:,1),';j1;*-',t,move_q_dot(:,2),';j2;*-',t,move_q_dot(:,3),';j3;*-',t,move_q_dot(:,4),';j4;*-',t,move_q_dot(:,5),';j5;*-',t,move_q_dot(:,6),';j6;*-');
  grid on;
  ylabel('move angular vel (rad/s)');
  xlabel('time (s)');
  title('joint ang velo move');

  figure(8),plot(t,traj_q_dot(:,1),';j1;*-',t,traj_q_dot(:,2),';j2;*-',t,traj_q_dot(:,3),';j3;*-',t,traj_q_dot(:,4),';j4;*-',t,traj_q_dot(:,5),';j5;*-',t,traj_q_dot(:,6),';j6;*-');
  grid on;
  ylabel('plan angular vel (rad/s)');
  xlabel('time (s)');
  title('joint ang velo plan');
end
if(false)
  figure(9),plot(t,traj_acc(:,1),';j1;*-',t,traj_acc(:,2),';j2;*-',t,traj_acc(:,3),';j3;*-',t,traj_acc(:,4),';j4;*-',t,traj_acc(:,5),';j5;*-',t,traj_acc(:,6),';j6;*-');
  grid on;
  ylabel('plan angular acc (rad/s^2)');
  xlabel('time (s)');
  title('joint ang acc plan');
end
