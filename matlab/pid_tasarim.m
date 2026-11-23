%% PID ile  hiz kontrolu - P, PI, PID karsilastirmasi
clear; clc; close all;

J = 0.01; b = 0.1; K = 0.01; R = 1; L = 0.5;
s = tf('s');
G = K / ((J*s + b)*(L*s + R) + K^2); % Motor (plant)

% Kontrolculer
C_P = pid(100); % Sadece P
C_PI = pid(100, 200); % P + I
C_PID = pid(100, 200, 10); % P + I + D

% Kapali cevrim: geri besleme ile T = C*G / (1 + C*G)
T_P = feedback(C_P*G, 1);
T_PI = feedback(C_PI*G, 1);
T_PID = feedback(C_PID*G, 1);

% Grafik
figure;
step(T_P, T_PI, T_PID, 3);
grid on;
legend('P (Kp=100)', 'PI (Kp=100, Ki=200)', ...
    'PID (Kp=100, Ki=200, Kd=10)', 'Location', 'southeast');
title('Kapali Cevrim Hiz Cevabi (hedef: 1 rad/s)');

% Sayisal Karsilastirma
isimler   = {'P', 'PI', 'PID'};
sistemler = {T_P, T_PI, T_PID};
for i = 1:3
    bilgi = stepinfo(sistemler{i});
    hata  = 1 - dcgain(sistemler{i});
    fprintf('%-4s -> Asim: %5.1f %% | Yerlesme: %.2f s | Kalici hata: %.3f\n', ...
        isimler{i}, bilgi.Overshoot, bilgi.SettlingTime, hata);
end
