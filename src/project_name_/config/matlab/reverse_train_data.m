clc;
close all;
clear;

set_matlab_utils_path();

%% ==========  LOAD ===========
fid = FileIO('../training/pih_ur_rev_train_data2.bin');

fid.printHeader()

Timed = fid.read('Timed');
Pd_data = fid.read('Pd_data');
dPd_data = fid.read('dPd_data');
Qd_data = fid.read('Qd_data');
vRotd_data = fid.read('vRotd_data');
Jpos_data = fid.read('joint_pos_data');

n_data = length(Timed);

%% ==========  TRIM ===========

P0 = Pd_data(:,1);
Pf = Pd_data(:,end);

i1 = 0;
for i=1:n_data
    if (norm(Pd_data(:,i)-P0) > 2e-3)
       i1 = i;
       break;
    end
end

i2 = i1;
for i=n_data:-1:i1
    if (norm(Pd_data(:,i)-Pf) > 5e-3)
       i2 = i;
       break;
    end
end

Timed = Timed(i1:i2) - Timed(i1);
Pd_data = Pd_data(:, i1:i2);
dPd_data = dPd_data(:, i1:i2);
Qd_data = Qd_data(:, i1:i2);
vRotd_data = vRotd_data(:, i1:i2);
Jpos_data = Jpos_data(:, i1:i2);

n_data = length(Timed);

%% ==========  SMOOTH  ===========
n_smooth = round(0.05 * n_data);

for k=1:2
    for i=1:size(Pd_data,1)
       Pd_data(i,:) = smooth(Pd_data(i,:), n_smooth, 'moving'); 
    end

    for i=1:size(Qd_data,1)
       Qd_data(i,:) = smooth(Qd_data(i,:), n_smooth, 'moving'); 
    end
end

% for i=1:size(Jpos_data,2)
%    Jpos_data(i,:) = smooth(Jpos_data(i,:), 10, 'moving'); 
% end

%% ==========  SAVE  ===========

fid = FileIO('../training/pih_ur_train_data1.bin', bitor(FileIO.out,FileIO.trunc));

fid.printHeader()

fid.write('Timed', Timed);
fid.write('Pd_data', Pd_data);
fid.write('dPd_data', dPd_data);
fid.write('Qd_data', Qd_data);
fid.write('vRotd_data', vRotd_data);
fid.write('joint_pos_data', Jpos_data);


%% ==========  REVERSE  ===========
Time_rev = Timed;
Pd_rev_data = fliplr(Pd_data);
dPd_rev_data = fliplr(-dPd_data);
Qd_rev_data = fliplr(Qd_data);
vRotd_rev_data = fliplr(-vRotd_data);
Jpos_rev_data = fliplr(Jpos_data);


%% ==========  PLOT  ===========

figure;
for i=1:3
    subplot(3,1,i); hold on;
    plot(Time_rev, Pd_rev_data(i,:), 'LineWidth',2.0, 'LineStyle','-', 'Color','blue');
    plot(Timed, Pd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    if (i==1), legend({'for','rev'}, 'interpreter','latex', 'fontsize',15); end
    axis tight;
    hold off;
end

figure;
for i=1:3
    subplot(3,1,i); hold on;
    plot(Time_rev, dPd_rev_data(i,:), 'LineWidth',2.0, 'LineStyle','-', 'Color','blue');
    plot(Timed, dPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    if (i==1), legend({'for','rev'}, 'interpreter','latex', 'fontsize',15); end
    axis tight;
    hold off;
end

figure;
hold on;
plot3(Pd_rev_data(1,:), Pd_rev_data(2,:), Pd_rev_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta');
hold off;

%% ==========  SAVE  ===========

fid = FileIO('../training/pih_ur_train_data2.bin', bitor(FileIO.out,FileIO.trunc));

fid.printHeader()

fid.write('Timed', Time_rev);
fid.write('Pd_data', Pd_rev_data);
fid.write('dPd_data', dPd_rev_data);
fid.write('Qd_data', Qd_rev_data);
fid.write('vRotd_data', vRotd_rev_data);
fid.write('joint_pos_data', Jpos_rev_data);

