clc;
close all;
clear;

set_matlab_utils_path();

prefix = '../execution/';

data_file = 'exec_data.bin';

[Dfor, Drev] = loadData([prefix data_file]);

figure;
hold on;

i = 3;

pos_labels = {'$x$ [$m$]', '$y$ [$m$]', '$z$ [$m$]'};
qlog_labels = {'$\eta_x$ [$rad$]', '$\eta_y$ [$rad$]', '$\eta_z$ [$rad$]'};

for_color = [0 0.45 0.74];
rev_color = [0 0.75 0.75];

for_np_color = [0.75 0 0.75];
rev_np_color = 'magenta';

t_switch = [Dfor.Time(end) Dfor.Time(end)];

% -------- plot position ----------
ax = subplot(4,1,1); hold on;
plot(Dfor.Time, Dfor.Pos(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
plot(Drev.Time, Drev.Pos(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off');
ylabel(pos_labels{i}, 'interpreter','latex', 'fontsize',16); 
axis tight;
% -------- plot orientation ----------
ax = subplot(4,1,2); hold on;
plot(Dfor.Time, Dfor.qlog(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
plot(Drev.Time, Drev.qlog(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off');
ylabel(qlog_labels{i}, 'interpreter','latex', 'fontsize',16); 
axis tight;
% -------- plot phase ----------
ax = subplot(4,1,3); hold on;
plot(Dfor.Time, Dfor.x, 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
plot(Drev.Time, Drev.x, 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off');

% create legend
plot(nan, nan, 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'DisplayName','Perturbed');
plot(nan, nan, 'LineWidth',3, 'LineStyle','--', 'Color',for_np_color, 'DisplayName','Unperturbed');
legend({}, 'interpreter','latex', 'fontsize',15);

ylabel('phase var', 'interpreter','latex', 'fontsize',16); 
axis tight;
% -------- plot fv ----------
ax = subplot(4,1,4); hold on;
plot(Dfor.Time, Dfor.fv, 'LineWidth',3, 'LineStyle','-', 'Color','red', 'HandleVisibility','off');
plot(Drev.Time, Drev.fv, 'LineWidth',3, 'LineStyle','-', 'Color',[0.8 0 0], 'HandleVisibility','off');
plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off');
ylabel('$f_v$', 'interpreter','latex', 'fontsize',16);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
axis tight;


%% ==============================================================

function [Dfor, Drev] = loadData(path)

    fid = FileIO(path, FileIO.in);

    % fid.printHeader();

    Time = fid.read('Time');
    x_data = fid.read('x_data');
    fv_data = fid.read('fv_data');
    P_data = fid.read('P_data');
    % dP_data = fid.read('dP_data');
    Q_data = fid.read('Q_data');
    % vRot_data = fid.read('vRot_data');
    % Fext_data = fid.read('Fext_data');
    k = fid.read('motion_finish_ind');
    
    n_data = length(Time);
    
    qlog_data = zeros(3, n_data);
    Q0 = Q_data(:,1);
    for j=1:n_data, qlog_data(:,j) = math_.quatLog(math_.quatDiff(Q_data(:,j),Q0)); end

    Dfor = struct('Time',Time(1:k), 'x',x_data(1:k), 'Pos',P_data(:,1:k), 'qlog',qlog_data(:,1:k), 'fv',fv_data(1:k));
    Drev = struct('Time',Time(k+1:end), 'x',x_data(k+1:end), 'Pos',P_data(:,k+1:end), 'qlog',qlog_data(:,k+1:end), 'fv',fv_data(k+1:end));

end

