% clc;
% close all;
% clear;

set_matlab_utils_path();

subj_id = 1;

filename = ['../data/s' num2str(subj_id) '_power_2.bin'];
% filename = ['../data/s' num2str(subj_id) '_power_10.bin'];
% filename = ['../data/s' num2str(subj_id) '_vel.bin'];
% filename = ['../data/s' num2str(subj_id) '_mean.bin'];
% filename = ['../data/s' num2str(subj_id) '_min.bin'];
% filename = ['../data/s' num2str(subj_id) '_max.bin'];


% filename = '../data/data.bin';

%% ================= Load data ===================

fid = FileIO(filename, FileIO.in);
% fid.printHeader();
Time_data = fid.read('Time_data');
Vh1_data = fid.read('Vh1_data');
Vh2_data = fid.read('Vh2_data');
Fh1_data = fid.read('Fh1_data');
Fh2_data = fid.read('Fh2_data');
Damp_data = fid.read('Damp_data');
fid.close();

%% ================= Trim data ===================
n_data = length(Time_data);
j1 = 1;
j2 = n_data;
vel_thres = 0.005;

for j=1:n_data
   if (norm(Vh1_data(:,j))>vel_thres || norm(Vh2_data(:,j))>vel_thres)
       j1 = j;
       break;
   end
end

for j=n_data:-1:j1
   if (norm(Vh1_data(:,j))>vel_thres || norm(Vh2_data(:,j))>vel_thres)
       j2 = j;
       break;
   end
end

Time_data = Time_data(j1:j2);
Time_data = Time_data - Time_data(1);

Vh1_data = Vh1_data(:,j1:j2);
Vh2_data = Vh2_data(:,j1:j2);
Fh1_data = Fh1_data(:,j1:j2);
Fh2_data = Fh2_data(:,j1:j2);
Damp_data = Damp_data(:,j1:j2);


%% ================= Calc Power ===================
Tf = Time_data(end);
n_data = length(Time_data);
dt = Time_data(2) - Time_data(1);

W = 0;
for j=1:n_data
   W = W + dot(Vh1_data(:,j), Fh1_data(:,j))*dt + dot(Vh2_data(:,j), Fh2_data(:,j))*dt;
end

P = W/Tf;

% fprintf('Power: %.2f , Work: %.2f , duration: %.2f \n', P, W, Tf);

%% ================= Calc Pos/Orient ===================
P = zeros(3,1);
Q = [1 0 0 0]';

V_data = Vh1_data;

P_data = zeros(3, n_data);
Q_data = zeros(4, n_data);
qLog_data = zeros(3, n_data);%     x0 = 4.4;
%     x_end = 8.8;
%     line_style = '-';
%     pos_color = [0 0.75 0.75];
%     neg_color = [0 0 1];
%     legend_prefix = '$D_{pow}: ';


R_b_b1 = [ -4.7943e-01  -8.7753e-01  -9.4745e-03;
          8.7758e-01  -4.7940e-01  -5.1759e-03;
          -7.0987e-09  -1.0796e-02   9.9994e-01 ];
R_b1_b = R_b_b1';

for j=1:n_data
    P_data(:,j) = P;
    Q_data(:,j) = Q;
    qLog_data(:,j) = math_.quatLog(Q);
    
    P = P + R_b1_b*V_data(1:3,j)*dt;
    Q = math_.quatProd( math_.quatExp(R_b1_b*V_data(4:6,j)*dt), Q);
end


%% ================= Plot data ===================

% plotWrenchNorm(Time_data, Fh1_data)
% plotWrenchNorm(Time_data, Fh2_data)
% 
% return 

% fig = plotPose(Time_data, P_data, qLog_data);
% 
% fig = plotTwist(Time_data, Vh1_data);
% fig.Position = [150 526 560 420];
% 
% fig = plotTwist(Time_data, Vh2_data);
% fig.Position = [722 521 560 420];
% 
% fig = plotWrench(Time_data, Fh1_data);
% fig.Position(3:4) = [604 408];
% 
% fig = plotWrench(Time_data, Fh2_data);
% fig.Position = [730 15 560 420];

[fig ax] = plotPower(Time_data, [Vh1_data; Vh2_data], [Fh1_data; Fh2_data]);

fig = plotDamping(Time_data, Damp_data);
% fig.Position = [1314 268 560 420];


%% ===============================================
%% ============  Utility functions  ==============

function [fig, ax] = plotPower(Time, Vel_data, F_data)
    
    x0 = 0;
    x_end = Time(end);
    line_style = '-';
    pos_color = [0 0.75 0.75];
    neg_color = [0 0 1];
    legend_prefix = '$D_{pow}: ';
    
%     x0 = 0;
%     x_end = Time(end);
%     line_style = '-';
%     pos_color = [1 0 0];
%     neg_color = [0.635 0.078 0.184]; 
%     legend_prefix = '$D_{vel}: ';
    
    
%     n_data = length(Time);
%     
%     for i=1:n_data
%         if (Time(i) >= x0), break; end  
%     end
%     i1 = i;
%     
%     for i=n_data:-1:i1
%         if (Time(i) <= x_end), break; end  
%     end
%     i2 = i;
%     
%     Time = Time(i1:i2) - Time(i1);
%     Vel_data = Vel_data(:,i1:i2);
%     F_data = F_data(:,i1:i2);

    n_data = length(Time);

    Power = zeros(1, n_data);

    for j=1:n_data
        Power(j) = dot(Vel_data(:,j), F_data(:,j) );
    end
    
%     Power = Power / max(abs(Power));
    
    Pos_Power = Power;
    Pos_Power(Power<0) = nan;
    
    Neg_Power = Power;
    Neg_Power(Power>0) = nan;
    
    dt = Time(2) - Time(1);
    
    fig = figure;
    ax = axes();
    hold on;
    
    % create legend
    plot(nan, nan, 'LineWidth',2, 'Color',pos_color);
    plot(nan, nan, 'LineWidth',2, 'Color',neg_color);
    legend({[legend_prefix 'E^+$'], [legend_prefix 'E^-$']}, 'interpreter','latex', 'fontsize',15);
    
    plot([Time(1) Time(end)], [0 0], 'LineWidth',1, 'Color',[0.1 0.1 0.1], 'LineStyle',':', 'HandleVisibility','off');
    
    plot(Time, Pos_Power, 'LineWidth',2, 'Color',pos_color, 'LineStyle',line_style, 'HandleVisibility','off');
    plot(Time, Neg_Power, 'LineWidth',2, 'Color',neg_color, 'LineStyle',line_style, 'HandleVisibility','off');
 
    ylabel('Power [$W$]', 'interpreter','latex', 'fontsize',16);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
    box on;
    
    abs_work = sum(abs(Power))*dt;
    work = sum(Power)*dt;
    pos_work = sum(Power(Power>0))*dt;
    neg_work = sum(Power(Power<0))*dt;
    
    fprintf('====================\n');
    fprintf('abs_work:  %6.3f\n',abs_work);
    fprintf('work    :  %6.3f\n',work);
    fprintf('pos_work:  %6.3f\n',pos_work);
    fprintf('neg_work:  %6.3f\n',neg_work);
    fprintf('duration:  %6.3f\n',Time(end));
    fprintf('====================\n');
    
    axis tight;
end

%% ---------------------------------------------

function fig = plotWrenchNorm(Time, Wrench)

    Force = Wrench(1:3,:);
    Torque = Wrench(4:6,:);
    
    
    n_data = size(Force,2);
    force_norm = zeros(1,n_data);
    torque_norm = zeros(1,n_data);
    for j=1:n_data
        force_norm(j) = norm(Force(:,j));
        torque_norm(j) = norm(Torque(:,j));
    end

    fig = figure;
    subplot(1,2,1);
    plot(Time, force_norm, 'LineWidth',2, 'Color','blue');
    subplot(1,2,2);
    plot(Time, torque_norm, 'LineWidth',2, 'Color','red');

end

%% ---------------------------------------------


function fig = plotPose(Time, P_data, qLog_data)

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;

    k = [1 3 5];
    for i=1:3
        ax = subplot(3,2,k(i));
        plot(Time, P_data(i,:), 'LineWidth',2, 'Color',colors{i});
        ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Cart Pos [$m$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
        ax.YLim = [ min([-0.2, ax.YLim(1)]), max([ 0.2, ax.YLim(2)]) ];
    end

    k = k+1;
    for i=1:3
        ax = subplot(3,2,k(i));
        plot(Time, qLog_data(i,:), 'LineWidth',2, 'Color',colors{i});
    %     ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('$\log(\mathbf{Q})=\mathbf{k}\theta$ [$rad$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
        ax.YLim = [ min([-0.2, ax.YLim(1)]), max([ 0.2, ax.YLim(2)]) ];
    end

end

%% ---------------------------------------------

function fig = plotTwist(Time, Twist)

    Vel = Twist(1:3,:);
    RotVel = Twist(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;

    k = [1 3 5];
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, Vel(i,:), 'LineWidth',2, 'Color',colors{i});
        ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Trans Vel [$m/s$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

    k = k+1;
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, RotVel(i,:), 'LineWidth',2, 'Color',colors{i});
    %     ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Rot Vel [$rad/s$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

end

%% ---------------------------------------------

function fig = plotWrench(Time, Wrench)

    Force = Wrench(1:3,:);
    Torque = Wrench(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;

    k = [1 3 5];
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, Force(i,:), 'LineWidth',2, 'Color',colors{i});
        ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Force [$N$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

    k = k+1;
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, Torque(i,:), 'LineWidth',2, 'Color',colors{i});
    %     ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Torque [$Nm$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

end


%% ---------------------------------------------

function fig = plotDamping(Time, Damp)
    
    Dp = Damp(1:3,:);
    Do = Damp(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;
    
    Dp(1,:) = Dp(1,:) / max(Dp(1,:));

    k = [1 3 5];
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, Dp(i,:), 'LineWidth',2, 'Color',colors{i});
        ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Pos Damping', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

    k = k+1;
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, Do(i,:), 'LineWidth',2, 'Color',colors{i});
    %     ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Orient Damping', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end
        
end


