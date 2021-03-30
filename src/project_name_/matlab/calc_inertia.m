clc;
close all;
clear;

m = 40;

w = 0.35; % x
l = 0.8;  % y
h = 0.05; % z


J = diag((m/12)*[(l^2+h^2), (w^2+h^2), (l^2+w^2)]);

J