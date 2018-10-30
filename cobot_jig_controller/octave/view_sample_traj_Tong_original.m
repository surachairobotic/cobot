clear all;

s = load('../results/traj_original_Tong.txt');
%s = load('../results/traj_original.txt');

t = s(:,1);
q = s(:,2:7);
dq = s(:,8:13);
ddq = s(:,14:19);
%eff = s(:,20:25);

x = s(:,20:22);
o = s(:,23:26);
dx = s(:,27:32);


figure(4),subplot(3,1,1),plot(t,q(:,1),';j1;*-',t,q(:,2),';j2;*-',t,q(:,3),';j3;*-',t,q(:,4),';j4;*-',t,q(:,5),';j5;*-',t,q(:,6),';j6;*-');
grid on;
ylabel('angle(rad)');
figure(4),subplot(3,1,2),plot(t,dq(:,1),'*-',t,dq(:,2),'*-',t,dq(:,3),'*-',t,dq(:,4),'*-',t,dq(:,5),'*-',t,dq(:,6),'*-');
grid on;
ylabel('ang velo(rad/s)');
figure(4),subplot(3,1,3),plot(t,ddq(:,1),'*-',t,ddq(:,2),'*-',t,ddq(:,3),'*-',t,ddq(:,4),'*-',t,ddq(:,5),'*-',t,ddq(:,6),'*-');
grid on;
ylabel('ang acc(rad/s2)');
xlabel('time(s)');

figure(5),subplot(3,1,1),plot(t,x(:,1),';x;*-',t,x(:,2),';y;*-',t,x(:,3),';z;*-');
grid on;
ylabel('xyz(m)');
figure(5),subplot(3,1,2),plot(t,dx(:,1),'*-',t,dx(:,2),'*-',t,dx(:,3),'*-');
grid on;
ylabel('velo (m/s)');
figure(5),subplot(3,1,3),plot(t,o(:,1),'*-',t,o(:,2),'*-',t,o(:,3),'*-',t,o(:,4),'*-');
grid on;
ylabel('quat');
xlabel('time(s)');

