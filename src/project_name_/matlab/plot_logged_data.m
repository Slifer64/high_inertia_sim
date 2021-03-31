% clc;
close all;
clear;

set_matlab_utils_path();

% user = 'antonis';
% user = 'dimitris';
user = 'dora';


 filename = ['../data/' user '_force_Dadapt_1.bin'];
%  filename = ['../data/' user '_power_Dadapt_1.bin'];
%  filename = ['../data/' user '_vel_Dadapt_2.bin'];
% filename = ['../data/' user '_force_Dmin_1.bin']';
% filename = ['../data/' user '_force_Dmax_1.bin'];


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

fprintf('Power: %.2f , Work: %.2f , duration: %.2f \n', P, W, Tf);

%% ================= Calc Pos/Orient ===================
P = zeros(3,1);
Q = [1 0 0 0]';

V_data = Vh1_data;

P_data = zeros(3, n_data);
Q_data = zeros(4, n_data);
qLog_data = zeros(3, n_data);

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

fig = plotPose(Time_data, P_data, qLog_data);

fig = plotTwist(Time_data, Vh1_data);
fig.Position = [150 526 560 420];

fig = plotTwist(Time_data, Vh2_data);
fig.Position = [722 521 560 420];

fig = plotWrench(Time_data, Fh1_data);
fig.Position = [156 20 560 420];

fig = plotWrench(Time_data, Fh2_data);
fig.Position = [730 15 560 420];

fig = plotDamping(Time_data, Damp_data);
fig.Position = [1314 268 560 420];


%% ===============================================
%% ============  Utility functions  ==============

function fig = plotPose(Time, P_data, qLog_data)

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;

    k = [1 3 5];
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, P_data(i,:), 'LineWidth',2, 'Color',colors{i});
        ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('Cart Pos [$m$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
    end

    k = k+1;
    for i=1:3
        subplot(3,2,k(i));
        plot(Time, qLog_data(i,:), 'LineWidth',2, 'Color',colors{i});
    %     ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',14);
        if (i==1), title('$\log(Q)=k\theta$ [$rad$]', 'interpreter','latex', 'fontsize',16); end
        if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
        axis tight;
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

function fig = plotDamping(Time, Damp)
    
    Dp = Damp(1:3,:);
    Do = Damp(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    fig = figure;

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


