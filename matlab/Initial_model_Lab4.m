%% LABORATORIO 4: OBSERVADORES Y ESTIMADORES DE ESTADO - CONTROL DIGITAL
% Script de preparación y discretización del modelo del Motor DC
clear; clc; close all;


%% 1. DEFINICIÓN DEL MODELO CONTINUO EN ESPACIO DE ESTADOS
% De acuerdo con la identificación del reporte (Sección 4):
% Estados: x = [i_a (Corriente); w (Velocidad angular)]
% Entrada: u = V_a (Voltaje de armadura)


A_c = [-1099.06,  -0.16; ...
        12385.05,  -1.86];


B_c = [15.11; ...
        0];


% Medimos únicamente la velocidad (w) según la guía de laboratorio
C_c = [0, 1]; 
D_c = 0;


% Creación del sistema continuo en espacio de estados
sys_c = ss(A_c, B_c, C_c, D_c);


%% 2. TIEMPO DE MUESTREO Y DISCRETIZACIÓN (ZOH)
% Tiempo de muestreo seleccionado en la identificación previa
Ts = 0.000452; % 0.452 ms (Ajusta este valor si usaste otro Ts, e.g., 0.05)


% Discretización por Zero-Order Hold (ZOH)
sys_d = c2d(sys_c, Ts, 'zoh');


% Extracción de las matrices discretas A, B, C, D
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;


disp('=== MATRICES CONTINUAS (c) ===');
disp('A_c ='); disp(A_c);
disp('B_c ='); disp(B_c);


disp('=== MATRICES DISCRETAS (d) - ZOH ===');
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);
disp('Cd ='); disp(Cd);


%% 3. SIMULACIÓN Y VERIFICACIÓN A ENTRADA NOMINAL (12V)
t_sim = 0:Ts:1.5;            % Vector de tiempo para la simulación
u_sim = 12 * ones(size(t_sim)); % Escalón nominal de 12V


% Matriz de salida completa para graficar tanto Corriente (i_a) como Velocidad (w)
C_full = eye(2); 
sys_c_full = ss(A_c, B_c, C_full, [0; 0]);
sys_d_full = c2d(sys_c_full, Ts, 'zoh');


% Respuesta del sistema continuo y discreto
[y_c, t_c] = lsim(sys_c_full, u_sim, t_sim);
[y_d, t_d] = lsim(sys_d_full, u_sim, t_sim);


% Graficación
figure('Name', 'Validacion del Modelo con 12V Nominal', 'Color', 'w');


subplot(2,1,1);
plot(t_c, y_c(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_d, y_d(:,1), 'r--', 'LineWidth', 1.2);
grid on;
ylabel('Corriente i_a (A)');
title('Respuesta del Modelo del Motor DC a una entrada de 12 V');
legend('Continuo', 'Discreto (ZOH)');


subplot(2,1,2);
plot(t_c, y_c(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t_d, y_d(:,2), 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo (s)');
ylabel('Velocidad \omega (rad/s)');
legend('Continuo', 'Discreto (ZOH)');