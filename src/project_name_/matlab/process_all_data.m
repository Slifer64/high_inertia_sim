% clc;
% close all;
% clear;

set_matlab_utils_path();

damp_methods = {'power_2', 'power_10', 'vel', 'mean', 'min', 'max'};
ids = 1:10; % subject ids

%% ==============================================

subjId_name_expOrder = { 
    ids(1),  'Sotiris',    '6     4     5     1     2     3';
    ids(2),  'Iasonas',    '6     3     1     5     2     4';
    ids(3),  'Koutras',    '1     4     3     6     2     5';
    ids(4),  'Kanakis',    '3     6     1     5     4     2';
    ids(5),  'Kiriakos',   '3     2     4     1     5     6';
    ids(6),  'Droukas',    '4     1     6     5     3     2';
    ids(7),  'Kornilia',   '5     6     4     1     2     3';
    ids(8),  'Dora',       '6     2     4     3     1     5';
    ids(9),  'Eleutheria', '4     2     1     5     6     3';
   ids(10),  'Savvas',     '2     4     3     6     5     1';
};

T_subjId_name_expOrder = cell2table(subjId_name_expOrder, 'VariableNames',{'subjId' 'Name' 'ExpOrder'});

expId_dampMethod = {};
for i=1:length(damp_methods) 
    expId_dampMethod = [expId_dampMethod; {i, damp_methods{i}}];
end
    
T_expId_dampMethod = cell2table(expId_dampMethod, 'VariableNames',{'ExpId' 'Damping Method'});

%% ==============================================

E_plus = zeros(length(damp_methods), length(ids));
E_minus = zeros(size(E_plus));

for k1=1:length(ids)

    subj_id = ids(k1);

    for i_d=1:length(damp_methods)


        filename = ['../data/s' num2str(subj_id) '_' damp_methods{i_d} '.bin'];


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


        [e_plus, e_minus] = calcEnergy(Time_data, [Vh1_data; Vh2_data], [Fh1_data; Fh2_data]);

        E_plus(i_d, subj_id) = e_plus;
        E_minus(i_d, subj_id) = e_minus;

    end


end

figure;
plot(E_plus)
legend(damp_methods);

save('all_data.mat', 'T_expId_dampMethod', 'T_subjId_name_expOrder', 'E_plus', 'E_minus');

%% ===============================================
%% ============  Utility functions  ==============

function [E_plus, E_minus] = calcEnergy(Time, Vel_data, F_data)
    
    n_data = length(Time);

    Power = zeros(1, n_data);

    for j=1:n_data
        Power(j) = dot(Vel_data(:,j), F_data(:,j) );
    end
    
    Pos_Power = Power;
    Pos_Power(Power<0) = nan;
    
    Neg_Power = Power;
    Neg_Power(Power>0) = nan;
    
    dt = Time(2) - Time(1);

    pos_work = sum(Power(Power>0))*dt;
    neg_work = sum(Power(Power<0))*dt;
    
    E_plus = pos_work;
    E_minus = neg_work;
    
end

%% ---------------------------------------------

