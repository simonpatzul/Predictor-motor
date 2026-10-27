%% =====================================================
%% PUNTO 2 DE LA GUÍA: ESPACIO DE ESTADOS Y SIMULACIÓN
%% =====================================================

% 1. Crear el sistema multisalida: Salida 1 = Velocidad (Gw), Salida 2 = Corriente (Gi)
Sys_tf = [Gw; Gi];

% 2. Convertir a Espacio de Estados (A, B, C, D)
Sys_ss = ss(Sys_tf);

disp('=== MATRICES EN ESPACIO DE ESTADOS ===');
disp('Matriz A:'); disp(Sys_ss.A);
disp('Matriz B:'); disp(Sys_ss.B);
disp('Matriz C:'); disp(Sys_ss.C);
disp('Matriz D:'); disp(Sys_ss.D);

% 3. Simulación con entrada escalón de 12V
t_sim = 0:0.001:3;
u_sim = 12 * ones(size(t_sim));

[y_out, t_out] = lsim(Sys_ss, u_sim, t_sim);

% Separación de variables
speed_rad_s = y_out(:,1); % Velocidad en rad/s
current_A   = y_out(:,2); % Corriente en A

% 4. Gráficas separadas de Corriente y Velocidad
figure('Name', 'Punto 2: Espacio de Estados - Respuestas Separadas', 'Color', 'w');

subplot(2,1,1);
plot(t_out, speed_rad_s, 'b', 'LineWidth', 1.8);
grid on; box on;
title('Simulación en Espacio de Estados: Entrada 12V');
ylabel('Velocidad \omega (rad/s)');
legend('Velocidad', 'Location', 'southeast');

subplot(2,1,2);
plot(t_out, current_A, 'r', 'LineWidth', 1.8);
grid on; box on;
xlabel('Tiempo (s)');
ylabel('Corriente i_a (A)');
legend('Corriente', 'Location', 'southeast');

speed_rpm = speed_rad_s * (60 / (2*pi));

figure('Name', 'Punto 2: Espacio de Estados - Velocidad en RPM', 'Color', 'w');

subplot(2,1,1);
plot(t_out, speed_rpm, 'b', 'LineWidth', 1.8);
grid on; box on;
title('Simulación en Espacio de Estados: Entrada 12V');
ylabel('Velocidad (RPM)');
legend('Velocidad', 'Location', 'southeast');