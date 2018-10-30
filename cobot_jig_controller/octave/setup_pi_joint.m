clear all;
close all;

#path = uigetdir(pwd, 'Select a folder');
files = dir(fullfile('../results/vel_15_i40_p0_6800/', '*.txt'));
end_err = 9999;
#for k=1:length(files)
#  s = load(strcat('../results/vel_15_i40_p0_6800/', files(k).name));
k=1
  s = load('../results/move_wave.txt');
  clear t v_target v_move mmin mmax time_a line_a_1 line_a_2 err sum_err;
  t = s(:,1);
  v_target = s(:,2);
  v_move = s(:,3);
  mmin = 0;
  mmax = max(v_move);
  for j=1:length(v_move)
    if v_move(j) >= v_target(j)
      time_a = t(j);
      indx = j;
      break;
    end;
  end;
  m = mmax/99;
  c = 1;
  for j=1:100
    line_a_1(j) = m*j;
    line_a_2(j) = time_a;
  end;
  err = abs(v_target-v_move);
  sum_err = 0.0;
#  for j=indx+1:length(err)
  for j=2:length(err)
    sum_err += abs((err(j-1)+err(j))/2*(t(j)-t(j-1)));
  end;
  if end_err > sum_err
    end_err = sum_err;
    select_file = files(k).name;
  end;
  fig = figure(k);
  plot(t, v_move, t, v_target, 'o', t, err, line_a_2, line_a_1);
  grid on;
  title(strcat('Velocity : ', ' sum error = ', num2str(sum_err), ' : ', files(k).name));
  ylabel('vel(rad/s)');
  xlabel('time(s)');
#  print -dpng foo.png
#end
#select_file