clc;
close all;
clear;

filename = '../execution/p8p_1_data.bin';

fid = FileIO(filename, FileIO.in);

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
% k = k(end);

k

figure;
plot(P_data(3,:));



i_pick_for = 1215
i_picK_rev = 2474
i_place_for = 3033

i_pick_for = 1215
i_picK_rev = 2474
i_place_for = 3033






