clc;
close all;
clear;

% a = [-0.604; 0.0251; 0.3];
% b = [-0.663; 0.0205; 0.316];
% 
% a - b

global t_switch t_collide t_retreat t_retry

set_matlab_utils_path();

prefix = '../execution/';

data_files = {'target4_place2_data.bin', 'target4_place2_nom_data.bin'};

[Dfor, Drev] = loadData([prefix data_files{1}]);
[Dfor2, Drev2] = loadData([prefix data_files{2}]);

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
t_collide = [Dfor.Time(443) Dfor.Time(443)];
t_retreat = [Dfor.Time(575) Dfor.Time(575)];
t_retry = [Dfor.Time(846) Dfor.Time(846)];
Drev2.Time = Drev2.Time - Drev2.Time(1) + Dfor.Time(end);

% -------- plot position ----------
ax = subplot(3,1,1); hold on;
plot(Dfor.Time, Dfor.Pos(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
plot(Drev.Time, Drev.Pos(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
% plot(Dfor2.Time, Dfor2.Pos(i,:), 'LineWidth',3, 'LineStyle','--', 'Color',for_np_color, 'HandleVisibility','off');
% plot(Drev2.Time, Drev2.Pos(i,:), 'LineWidth',3, 'LineStyle','--', 'Color',rev_np_color, 'HandleVisibility','off');
plotTimes(ax);
ylabel(pos_labels{i}, 'interpreter','latex', 'fontsize',16); 
axis tight;
% % -------- plot orientation ----------
% ax = subplot(3,1,2); hold on;
% plot(Dfor.Time, Dfor.qlog(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
% plot(Drev.Time, Drev.qlog(i,:), 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
% plot(Dfor2.Time, Dfor2.qlog(i,:), 'LineWidth',3, 'LineStyle','--', 'Color',for_np_color, 'HandleVisibility','off');
% plot(Drev2.Time, Drev2.qlog(i,:), 'LineWidth',3, 'LineStyle','--', 'Color',rev_np_color, 'HandleVisibility','off');
% plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off');
% ylabel(qlog_labels{i}, 'interpreter','latex', 'fontsize',16); 
% axis tight;
% -------- plot phase ----------
ax = subplot(3,1,2); hold on;
plot(Dfor.Time, Dfor.x, 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'HandleVisibility','off');
plot(Drev.Time, Drev.x, 'LineWidth',3, 'LineStyle','-', 'Color',rev_color, 'HandleVisibility','off');
% plot(Dfor2.Time, Dfor2.x, 'LineWidth',3, 'LineStyle','--', 'Color',for_np_color, 'HandleVisibility','off');
% plot(Drev2.Time, Drev2.x, 'LineWidth',3, 'LineStyle','--', 'Color',rev_np_color, 'HandleVisibility','off');
plotTimes(ax);

% % create legend
% plot(nan, nan, 'LineWidth',3, 'LineStyle','-', 'Color',for_color, 'DisplayName','Perturbed');
% plot(nan, nan, 'LineWidth',3, 'LineStyle','--', 'Color',for_np_color, 'DisplayName','Unperturbed');
% legend({}, 'interpreter','latex', 'fontsize',15);

ylabel('phase var', 'interpreter','latex', 'fontsize',16); 
axis tight;
% -------- plot fv ----------
ax = subplot(3,1,3); hold on;
plot(Dfor.Time, Dfor.fv, 'LineWidth',3, 'LineStyle','-', 'Color','red', 'HandleVisibility','off');
plot(Drev.Time, Drev.fv, 'LineWidth',3, 'LineStyle','-', 'Color',[0.8 0 0], 'HandleVisibility','off');
plotTimes(ax);
ylabel('$f_v$', 'interpreter','latex', 'fontsize',16);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
axis tight;


%% ==============================================================

function [Dfor, Drev] = loadData(path)

    fid = FileIO(path, FileIO.in);

    % fid.printHeader();

    Time = fid.read('Time');
    
    Time = Time - Time(1);
    
    x_data = fid.read('x_data');
    fv_data = fid.read('fv_data');
    P_data = fid.read('P_data');
    % dP_data = fid.read('dP_data');
    Q_data = fid.read('Q_data');
    % vRot_data = fid.read('vRot_data');
    % Fext_data = fid.read('Fext_data');
    k = fid.read('motion_finish_ind');
    k = k(end);
    
    n_data = length(Time);
    
    qlog_data = zeros(3, n_data);
    Q0 = Q_data(:,1);
    for j=1:n_data, qlog_data(:,j) = math_.quatLog(math_.quatDiff(Q_data(:,j),Q0)); end

    Dfor = struct('Time',Time(1:k), 'x',x_data(1:k), 'Pos',P_data(:,1:k), 'qlog',qlog_data(:,1:k), 'fv',fv_data(1:k));
    Drev = struct('Time',Time(k+1:end), 'x',x_data(k+1:end), 'Pos',P_data(:,k+1:end), 'qlog',qlog_data(:,k+1:end), 'fv',fv_data(k+1:end));

end

function plotTimes(ax)
    
    global t_switch t_collide t_retreat t_retry
    
    plot(t_switch, ax.YLim, 'LineWidth',2, 'Color',[0.85 0.33 0.1], 'LineStyle','--', 'HandleVisibility','off', 'Parent',ax);
    plot(t_collide, ax.YLim, 'LineWidth',2, 'Color',[0.75 0 0.75], 'LineStyle','--', 'HandleVisibility','off', 'Parent',ax);
    plot(t_retreat, ax.YLim, 'LineWidth',2, 'Color',[1 0 1], 'LineStyle','--', 'HandleVisibility','off', 'Parent',ax);
    plot(t_retry, ax.YLim, 'LineWidth',2, 'Color',[0 0.6 0], 'LineStyle','--', 'HandleVisibility','off', 'Parent',ax);

end


