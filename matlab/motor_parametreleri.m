%% DC Motor Parametreleri
clear; clc; close all;

J = 0.01; % Rotor ataleti [kg.m^2]
b = 0.1; % Viskoz surtunme katsayisi [N.m.s]
K = 0.01; % Motor sabiti (Kt = ke) [N.m/A]
R = 1; % Armatur direnci [Ohm]
L = 0.5; % Armatur enduktansi [H]

%% Transfer Fonksiyonu: Gerilim (V) -> Hiz (rad/s)
s = tf('s');
G = K / ((J*s + b)*(L*s + R) + K^2);

%% 1 V basamak girise acik cevrim cevabi
figure;
step(G);
grid on;
title('DC Motor Acik Cevrim Hiz Cevabi (1 V Giris)');

%% Karakteristik Bilgiler
bilgi = stepinfo(G);
fprintf('Yukselme suresi: %.2f s\n', bilgi.RiseTime);
fprintf('Yerlesme suresi: %.2f s\n', bilgi.SettlingTime);
fprintf('Kalici durum hizi: %.3f rad/s\n', dcgain(G));