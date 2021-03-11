clc;
close all;
clear;

set_matlab_utils_path();

%% load train data
fid = FileIO('../train_data/train_data.bin');

Timed = fid.read('Timed');
Pd_data = fid.read('Pd_data');
dPd_data = fid.read('dPd_data');
Qd_data = fid.read('Qd_data');
vRotd_data = fid.read('vRotd_data');

%% load exec data
fid = FileIO('../exec_data/exec_data.bin');

Time = fid.read('Time');
P_data = fid.read('P_data');
dP_data = fid.read('dP_data');
Fp_data = fid.read('Fp_data');
Q_data = fid.read('Q_data');
vRot_data = fid.read('vRot_data');
Fo_data = fid.read('Fo_data');

x_data = fid.read('x_data');
x_dot_data = fid.read('x_dot_data');
fv_data = fid.read('fv_data');



%% Plot phase variable
figure;
subplot(3,1,1);
plot(Time, x_data, 'LineWidth',2.0 , 'Color','blue');
ylabel('$x$', 'interpreter','latex', 'fontsize',15);
subplot(3,1,2);
plot(Time, x_dot_data, 'LineWidth',2.0 , 'Color','cyan');
ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
subplot(3,1,3);
plot(Time, fv_data, 'LineWidth',2.0 , 'Color','red');
ylabel('$f_v$', 'interpreter','latex', 'fontsize',15);


%% Plot Position results

% figure;
% for i=1:3
%     subplot(3,1,i);
%     hold on;
%     plot(Time, P_data(i,:), 'LineWidth',2.0 , 'Color','blue');
%     plot(Timed, Pd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
%     ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
%     % title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
%     legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
%     axis tight;
%     hold off;
% end
% 
% figure;
% for i=1:3
%     subplot(3,1,i);
%     hold on;
%     plot(Time, dP_data(i,:), 'LineWidth',2.0, 'Color','blue');
%     plot(Timed, dPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
%     ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
%     axis tight;
%     hold off;
% end
% 
% figure;
% hold on;
% plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
% plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','blue');
% hold off;


%% Plot orientation results

% Q0 = Q_data(:, 1);
% Qd0 = Qd_data(:, 1);
% 
% Pqd_data = zeros(3, size(Qd_data,2));
% for j=1:size(Pqd_data,2)
%     Pqd_data(:,j) = math_.quatLog( math_.quatProd( Qd_data(:,j), math_.quatInv(Qd0) ) );
% end
% 
% Pq_data = zeros(3, size(Q_data,2));
% for j=1:size(Pq_data,2)
%     Pq_data(:,j) = math_.quatLog( math_.quatProd( Q_data(:,j), math_.quatInv(Q0) ) );
% end
% 
% line_width = 2.5;
%  
% figure('Position', [200 200 600 500]);
% y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
% for i=1:3
%    subplot(3,1,i);
%    hold on;
%    plot(Time, Pq_data(i,:), 'LineWidth', line_width);
%    plot(Timed, Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
%    ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
%    axis tight;
%    if (i==1), legend({'gmp', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
%    if (i==1), title('Quaternion error: $e_q = log(Q * Q_0^{-1})$', 'interpreter','latex', 'fontsize',18); end
%    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
%    hold off;
% end
% 
% 
% figure;
% hold on;
% plot3(Pq_data(1,:), Pq_data(2,:), Pq_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
% plot3(Pqd_data(1,:), Pqd_data(2,:), Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
% legend('gmp', 'demo');
% hold off;
% 
% figure;
% Q_labels = {'$\eta$','$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'};
% Qd_labels = {'$\eta_d$','$\epsilon_{d,1}$', '$\epsilon_{d,2}$', '$\epsilon_{d,3}$'};
% for i=1:4
%    subplot(4,1,i);
%    hold on;
%    plot(Time, Q_data(i,:), 'LineWidth', line_width);
%    plot(Timed, Qd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
%    legend({Q_labels{i}, Qd_labels{i}}, 'interpreter','latex', 'fontsize',15);
%    if (i==1), title('Unit Quaternion', 'interpreter','latex', 'fontsize',17); end
%    if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
%    hold off;
% end
% 
% figure;
% vRot_labels = {'$\omega_x$','$\omega_y$', '$\omega_z$'};
% vRotd_labels = {'$\omega_{d,x}$','$\omega_{d,y}$', '$\omega_{d,z}$'};
% for i=1:3
%    subplot(3,1,i);
%    hold on;
%    plot(Time, vRot_data(i,:), 'LineWidth', line_width);
%    plot(Timed, vRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
%    legend({vRot_labels{i}, vRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
%    if (i==1), title('Rotational Velocity', 'interpreter','latex', 'fontsize',17); end
%    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
%    hold off;
% end


%% ====== Plot 3D path =======

animated = true;
n_skip = 100;

axis_colors = {'red', 'gree', 'blue'};
axis_colors_d = {[1 0 0 0.5], [0 1 0 0.5], [0 0 1 0.5]};

n_demo_frames = 10;

Pg = Pd_data(:,end);

Axang = quat2axang(Q_data')';

Axangd = quat2axang(Qd_data')';

%% 3D path
ax = axes('Parent',figure());
hold(ax,'on');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color',[0 0 1 0.5], 'Parent',ax);

pl_o = cell(3,1);
for i=1:3, pl_o{i} = quiver3(ax, 0,0,0,0,0,0, 0.2, 'Color',axis_colors_d{i}, 'LineWidth',2, 'LineStyle','-', 'AutoScale','on'); end
n_data = size(Pd_data,2);
j_ind = 1:floor(n_data/n_demo_frames):n_data;
for k=1:n_demo_frames
    j = j_ind(k);
    x = Pd_data(1,j); y = Pd_data(2,j); z = Pd_data(3,j);
    u = Axangd(1,j); v = Axangd(2,j); w = Axangd(3,j); theta = Axangd(4,j);
    T = makehgtform('translate',[x y z]) * makehgtform('axisrotate',[u v w],theta);
    for i=1:3
       pl_o{i}.XData = [pl_o{i}.XData x];  pl_o{i}.YData = [pl_o{i}.YData y]; pl_o{i}.ZData = [pl_o{i}.ZData z];
       pl_o{i}.UData = [pl_o{i}.UData T(1,i)];  pl_o{i}.VData = [pl_o{i}.VData T(2,i)]; pl_o{i}.WData = [pl_o{i}.WData T(3,i)];
    end
end

plot3(Pg(1), Pg(2), Pg(3), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);

if (animated)
    pl = plot3(nan, nan, nan, 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
    pl_ee = plot3(nan, nan, nan, 'LineWidth',3, 'LineStyle','-', 'Marker','*', 'MarkerSize',14, 'Color',[0.85 0.33 0.1], 'Parent',ax);
    
    pl_o = cell(3,1);
    for i=1:3, pl_o{i} = quiver3(ax, 0,0,0,0,0,0, 0.08, 'Color',axis_colors{i}, 'LineWidth',4, 'LineStyle','-', 'AutoScale','on'); end
        
    for j=1:n_skip:size(P_data,2)
       
       x = P_data(1,j); y = P_data(2,j); z = P_data(3,j);
       u = Axang(1,j); v = Axang(2,j); w = Axang(3,j); theta = Axang(4,j);
       
       pl.XData = [pl.XData x]; pl.YData = [pl.YData y]; pl.ZData = [pl.ZData z];
       pl_ee.XData = x; pl_ee.YData = y; pl_ee.ZData = z;
       
       T = makehgtform('translate',[x y z]) * makehgtform('axisrotate',[u v w],theta);
       for i=1:3
           pl_o{i}.XData = x;  pl_o{i}.YData = y; pl_o{i}.ZData = z;
           pl_o{i}.UData = T(1,i);  pl_o{i}.VData = T(2,i); pl_o{i}.WData = T(3,i);
       end
       
       drawnow;
       pause(0.001);
    end
else
    plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);       
end
hold(ax,'off');


