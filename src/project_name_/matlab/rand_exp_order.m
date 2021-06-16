clc;
close all;
clear;

exp_name = cell(6,1);
exp_name{1} = 'power 2';
exp_name{2} = 'power 10';
exp_name{3} = 'vel';
exp_name{4} = 'mean';
exp_name{5} = 'min';
exp_name{6} = 'max';


p = randperm(6)
disp(exp_name(p))


% 1: 'power 2'
% 2: 'power 10'
% 3: 'vel'
% 4: 'mean'
% 5: 'min'
% 6: 'max'
%
% s1 - Sotiris:    6     4     5     1     2     3
% s2 - Iasonas:    6     3     1     5     2     4
% s3 - Koutras:    1     4     3     6     2     5
% s4 - George:     3     6     1     5     4     2
% s5 - Kiriakos:   3     2     4     1     5     6
% s6 - Droukas:    4     1     6     5     3     2
% s7 - Kornilia:   5     6     4     1     2     3
% s8 - Dora    :   6     2     4     3     1     5
% s9 - Eleuthe :   4     2     1     5     6     3
% s10 -Savvas  :   2     4     3     6     5     1
