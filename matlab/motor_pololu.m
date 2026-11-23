%% Pololu 37D 12V 30:1 - Datasheet'ten motor modeli
clear; clc; close all;

% --- Datasheet degerleri ---
V_nom = 12; % Nominal Gerilim V
I_stall = 5.5; % Klitli rotor akimi A
I_nl = 0.2; % Bosta akim A
rpm_nl = 330; % Bosta hiz rpm
T_stall = 14 * 0.0981; % Kilitli tork N.m (14 kg.cm)

w_nl = rpm_nl * 2*pi/60; % rom -> rad/s

% --- Hesaplanan parametreler ---
R = V_nom / I_stall; % Direnc ohm
K = (V_nom - R*I_nl) / w_nl; % Motor sabiti V.S/rad
b = K * I_nl / w_nl; % Surtunme N.m.s/rad

% --- Tahmini parametreler ---
L = 0.002; % Enduktans H -> tipik tahmin
J = 0.01; % Atalet kg.m^2 - motor + disli + kucuk yuk

fprintf('R = %.2f Ohm | K = %.3f | b = %.5f\n', R, K, b);

%% Dogrulama: Model datasheet'i tutuyor mu?
s = tf('s');
G = K / ((J*s + b)*(L*s + R) + K^2);
fprintf('Bosta hiz -> Model: %.0f rpm | Datasheet: %d rpm\n', ...
    dcgain(G)*12*60/(2*pi), rpm_nl);
fprintf('Kilitli tork -> Model: %.2f N.m | Datasheet: %.2f N.m\n', ...
    K*I_stall, T_stall);

%% PI kontrolcuyu MATLAB otomatik ayarlasin
C = pidtune(G, 'PI')