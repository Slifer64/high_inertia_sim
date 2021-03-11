clc;
close all;
clear;

set_matlab_utils_path();

prefix = '../execution/';

% data_files = {'target1_pick1_data.bin'};
data_files = {'target1_pick1_data.bin', 'target2_place1_data.bin', 'target3_pick2_perturb_data.bin', 'target4_place2_data.bin'};
m = length(data_files);

fig_p = init_3D_pos_fig();
fig_o = init_3D_orient_fig();


for k=1:m
    % -------- load ----------
    [Dfor, Drev] = loadData([prefix data_files{k}]);
    
    if (k~=4)
        % -------- plot position ----------
        set(0, 'CurrentFigure', fig_p);
        plot3(Dfor.Pos(1,:), Dfor.Pos(2,:), Dfor.Pos(3,:), 'LineWidth',3, 'LineStyle','-', 'Color','green', 'HandleVisibility','off');
        plot3(Drev.Pos(1,:), Drev.Pos(2,:), Drev.Pos(3,:), 'LineWidth',4, 'LineStyle',':', 'Color','red', 'HandleVisibility','off');
        Pg = [Dfor.Pos(1,end); Dfor.Pos(2,end); Dfor.Pos(3,end)];
        scatter3(Pg(1), Pg(2), Pg(3), 'LineWidth',4, 'Marker','*', 'MarkerEdgeColor','magenta', 'SizeData',200, 'HandleVisibility','off');
        P0 = [Dfor.Pos(1,1); Dfor.Pos(2,1); Dfor.Pos(3,1)];
        scatter3(P0(1), P0(2), P0(3), 'LineWidth',4, 'Marker','o', 'MarkerEdgeColor','cyan', 'SizeData',200, 'HandleVisibility','off');
        % -------- plot orientation ----------
    
    else
        i_col = 443;
        i_retr = 575;
        i_res = 846;
        
        % -------- plot position ----------
        set(0, 'CurrentFigure', fig_p);
        plot3(Dfor.Pos(1,1:i_col), Dfor.Pos(2,1:i_col), Dfor.Pos(3,1:i_col), 'LineWidth',3, 'LineStyle','-', 'Color',[0 0.45 0.74], 'HandleVisibility','off');
        %plot3(Dfor.Pos(1,i_col+1:i_retr), Dfor.Pos(2,i_col+1:i_retr), Dfor.Pos(3,i_col+1:i_retr), 'LineWidth',3, 'LineStyle','-', 'Color','cyan', 'HandleVisibility','off');
        plot3(Dfor.Pos(1,i_retr+1:i_res), Dfor.Pos(2,i_retr+1:i_res), Dfor.Pos(3,i_retr+1:i_res), 'LineWidth',3, 'LineStyle',':', 'Color',[0.93 0.69 0.13], 'HandleVisibility','off');
        plot3(Dfor.Pos(1,i_res+1:end), Dfor.Pos(2,i_res+1:end), Dfor.Pos(3,i_res+1:end), 'LineWidth',3, 'LineStyle','-', 'Color','green', 'HandleVisibility','off');
        
        
        plot3(Drev.Pos(1,:), Drev.Pos(2,:), Drev.Pos(3,:), 'LineWidth',4, 'LineStyle',':', 'Color','red', 'HandleVisibility','off');
        Pg = [Dfor.Pos(1,end); Dfor.Pos(2,end); Dfor.Pos(3,end)];
        scatter3(Pg(1), Pg(2), Pg(3), 'LineWidth',4, 'Marker','*', 'MarkerEdgeColor','magenta', 'SizeData',200, 'HandleVisibility','off');
        P0 = [Dfor.Pos(1,1); Dfor.Pos(2,1); Dfor.Pos(3,1)];
        scatter3(P0(1), P0(2), P0(3), 'LineWidth',4, 'Marker','o', 'MarkerEdgeColor','cyan', 'SizeData',200, 'HandleVisibility','off');
    end
    
    
%     quiv = cell(3,1); % three quivers, for x, y and z axis of orientation frame
%     axis_colors = {'red', 'green', 'blue'};
%     for j=1:3
%         quiv{j} = quiver3(0,0,0,0,0,0, 0.3);
%         set(quiv{j}, 'Color',axis_colors{j}, 'LineStyle','-', 'LineWidth',2, 'AutoScale','on', 'HandleVisibility','off');
%     end
%     
%     ind = getIndices(Dfor.Pos, 5);
%     
%     n = length(ind);
%     P = Dfor.Pos(:,ind);
%     Q = Dfor.Quat(:,ind);
%     Xo = zeros(3,n);
%     Yo = zeros(3,n);
%     Zo = zeros(3,n);
%     for i=1:n
%         R = quat2rotm(Q(:,i)');
%         Xo(:,i) = R(:,1);
%         Yo(:,i) = R(:,2);
%         Zo(:,i) = R(:,3);
%     end
%     orient = {Xo, Yo, Zo};
%     
%     for j=1:3
%         quiv{j}.XData = P(1,:);
%         quiv{j}.YData = P(2,:);
%         quiv{j}.ZData = P(3,:);
%         quiv{j}.UData = orient{j}(1,:);
%         quiv{j}.VData = orient{j}(2,:);
%         quiv{j}.WData = orient{j}(3,:);
%     end
     
     
    set(0, 'CurrentFigure', fig_o);
    plot3(Dfor.qlog(1,:), Dfor.qlog(2,:), Dfor.qlog(3,:), 'LineWidth',3, 'LineStyle','-', 'Color','green', 'HandleVisibility','off');
    plot3(Drev.qlog(1,:), Drev.qlog(2,:), Drev.qlog(3,:), 'LineWidth',4, 'LineStyle',':', 'Color','red', 'HandleVisibility','off');
    Pg = [Dfor.qlog(1,end); Dfor.qlog(2,end); Dfor.qlog(3,end)];
    scatter3(Pg(1), Pg(2), Pg(3), 'LineWidth',4, 'Marker','*', 'MarkerEdgeColor','magenta', 'SizeData',200, 'HandleVisibility','off');
    P0 = [Dfor.qlog(1,1); Dfor.qlog(2,1); Dfor.qlog(3,1)];
    scatter3(P0(1), P0(2), P0(3), 'LineWidth',4, 'Marker','o', 'MarkerEdgeColor','cyan', 'SizeData',200, 'HandleVisibility','off');

end


%% ==============================================================

function [Dfor, Drev] = loadData(path)

    fid = FileIO(path, FileIO.in);

    % fid.printHeader();

    Time = fid.read('Time');
    x_data = fid.read('x_data');
    P_data = fid.read('P_data');
    dP_data = fid.read('dP_data');
    Q_data = fid.read('Q_data');
    % vRot_data = fid.read('vRot_data');
    Fext_data = fid.read('Fext_data');
    k = fid.read('motion_finish_ind');
    k = k(end);
    
    n_data = length(Time);
    
    qlog_data = zeros(3, n_data);
    Q0 = Q_data(:,1);
    for j=1:n_data, qlog_data(:,j) = math_.quatLog(math_.quatDiff(Q_data(:,j),Q0)); end

    fv_data = zeros(1, n_data);
    for j=1:n_data
        dir = dP_data(:,j);
        if (norm(dir) > 1e-16), dir = dir/norm(dir);
        else, dir = zeros(3,1);
        end
        fv_data(j) = dot(dir, Fext_data(1:3,j));
    end

    Dfor = struct('Time',Time(1:k), 'x',x_data(1:k), 'Pos',P_data(:,1:k), 'Quat',Q_data, 'qlog',qlog_data(:,1:k), 'fv',fv_data(1:k));
    Drev = struct('Time',Time(k+1:end), 'x',x_data(k+1:end), 'Pos',P_data(:,k+1:end), 'Quat',Q_data, 'qlog',qlog_data(:,k+1:end), 'fv',fv_data(k+1:end));

end

function fig = init_3D_pos_fig()

    fig = figure;
    fig.Position(3:4) = [560 420];
    ax = axes();
    ax.FontSize = 13;
    hold(ax, 'on');
    % -------- make legend ----------
    plot3(nan,nan,nan, 'LineWidth',3, 'LineStyle','-', 'Color','green', 'DisplayName','forward');
    plot3(nan,nan,nan, 'LineWidth',4, 'LineStyle',':', 'Color','red', 'DisplayName','reverse');
    plot3(nan,nan,nan, 'LineWidth',4, 'LineStyle',':', 'Color',[0 0.45 0.74], 'DisplayName','collision (forward)');
    plot3(nan,nan,nan, 'LineWidth',4, 'LineStyle',':', 'Color',[0.93 0.69 0.13], 'DisplayName','collision (retraction)');
    scatter3(nan,nan,nan, 'LineWidth',4, 'Marker','*', 'MarkerEdgeColor','magenta', 'SizeData',200, 'DisplayName','target poses');
    scatter3(nan,nan,nan, 'LineWidth',4, 'Marker','o', 'MarkerEdgeColor','cyan', 'SizeData',200, 'DisplayName','initial pose');
    legend({}, 'interpreter','latex', 'fontsize',15);
    % -------- labels ----------
    xlabel('x [$m$]', 'interpreter','latex', 'fontsize',15);
    ylabel('y [$m$]', 'interpreter','latex', 'fontsize',15);
    zlabel('z [$m$]', 'interpreter','latex', 'fontsize',15);
    title('Cartesian Position', 'interpreter','latex', 'fontsize',17);
    grid on;
    % ax.CameraPosition = [3.1267 -4.8423 3.1143];
    % ax.Position = [0.1300 0.1100 0.7750 0.8150];
    % ax.XLim = [-0.9351 0.0649];
    % ax.YLim = [-0.6873 0.3127];
    % ax.ZLim = [0.3513 0.7513];

    % savefig(fig,'3D_pos.fig');

    % close(fig);

end

function fig = init_3D_orient_fig()

    fig = figure;
    fig.Position(3:4) = [560 420];
    ax = axes();
    ax.FontSize = 13;
    hold(ax, 'on');
    % -------- make legend ----------
    plot3(nan,nan,nan, 'LineWidth',3, 'LineStyle','-', 'Color','green', 'DisplayName','forward');
    plot3(nan,nan,nan, 'LineWidth',4, 'LineStyle',':', 'Color','red', 'DisplayName','reverse');
    scatter3(nan,nan,nan, 'LineWidth',4, 'Marker','*', 'MarkerEdgeColor','magenta', 'SizeData',200, 'DisplayName','target poses');
    scatter3(nan,nan,nan, 'LineWidth',4, 'Marker','o', 'MarkerEdgeColor','cyan', 'SizeData',200, 'DisplayName','initial pose');
    legend({}, 'interpreter','latex', 'fontsize',15);
    % -------- labels ----------
    xlabel('x [$rad$]', 'interpreter','latex', 'fontsize',15);
    ylabel('y [$rad$]', 'interpreter','latex', 'fontsize',15);
    zlabel('z [$rad$]', 'interpreter','latex', 'fontsize',15);
    title('Cartesian Orientation: $\eta = \log(Q*\bar{Q}_0)$', 'interpreter','latex', 'fontsize',17);
    grid on;
    % ax.CameraPosition = [3.1267 -4.8423 3.1143];
    % ax.Position = [0.1300 0.1100 0.7750 0.8150];
    % ax.XLim = [-0.9351 0.0649];
    % ax.YLim = [-0.6873 0.3127];
    % ax.ZLim = [0.3513 0.7513];

    % savefig(fig,'3D_pos.fig');

    % close(fig);

end

function ind = getIndices(Pos, n_ind)
    
    
    
    total_len = 0;
    
    n_data = size(Pos,2);
    for j=1:n_data-1
        total_len = total_len + norm(Pos(:,j+1)-Pos(:,j));
    end
    
    i_len = total_len / n_ind;
    
    ind = [1];
    
    len = 0;
    for j=1:n_data-1
        len = len + norm(Pos(:,j+1)-Pos(:,j));
        if (len >= i_len)
           ind = [ind j];
           len = 0;
        end
    end
    
    ind = [ind n_data];

end
