clc;
close all;
clear;

set_matlab_utils_path();

filename = '../data/data.bin';

%% ================= Load data ===================

fid = FileIO(filename, FileIO.in);
fid.printHeader();
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

P
W

%% ================= Plot data ===================

plotTwist(Time_data, Vh1_data);
plotTwist(Time_data, Vh2_data);

plotWrench(Time_data, Fh1_data);
plotWrench(Time_data, Fh2_data);

plotDamping(Time_data, Damp_data);



%% ===============================================
%% ============  Utility functions  ==============

function plotTwist(Time, Twist)

    Vel = Twist(1:3,:);
    RotVel = Twist(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    figure;

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

function plotWrench(Time, Wrench)

    Force = Wrench(1:3,:);
    Torque = Wrench(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    figure;

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

function plotDamping(Time, Damp)
    
    Dp = Damp(1:3,:);
    Do = Damp(4:6,:);

    y_labels = {'$x$', '$y$', '$z$'};
    colors = {'red', [0 0.7 0], 'blue'};

    figure;

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


