%% =====================================================
%% PUNTO 3 DE LA GUÍA: OBSERVADOR DE LUENBERGER
%% =====================================================


% Se asume que el Workspace ya contiene las funciones de transferencia 
% Gw, Gi y el tiempo de muestreo Ts = 0.01 del script lab2_control.m


% 1. Generación del sistema en Espacio de Estados y Discretización
Sys_tf = [Gw; Gi]; % Salida 1 = Velocidad, Salida 2 = Corriente
Sys_ss = ss(Sys_tf);
Sys_ss_d = c2d(Sys_ss, Ts, 'zoh');


Ad = Sys_ss_d.A;
Bd = Sys_ss_d.B;
Cd = Sys_ss_d.C;
% Dd = Sys_ss_d.D; % Se asume D = 0 para el predictor


% 2. Matriz de medición para el Observador
% Se requiere observar estados basados EXCLUSIVAMENTE en la medición de velocidad
C_m = Cd(1, :); % Se extrae solo la fila correspondiente a la velocidad (Salida 1)


% 3. Diseño de la matriz de Ganancia del Observador (L)
% Se busca que los polos del observador sean más rápidos que los del sistema en lazo abierto.
% Polos actuales de la planta discreta:
p_sys = eig(Ad);


% Asignación de polos más cercanos al origen en el plano Z (respuesta más rápida)
p_obs = [0.3, 0.4]; 


% Cálculo de L usando asignación de polos (Fórmula de Ackermann)
L = place(Ad', C_m', p_obs)';


disp('=== MATRIZ DE GANANCIA DEL OBSERVADOR (L) ===');
disp(L);


% 4. Condiciones Iniciales y Vectores de Simulación
t_sim = 0:Ts:3;
N_steps = length(t_sim);
u = 12 * ones(1, N_steps); % Entrada escalón de 12V (Voltaje nominal)


x_real = zeros(size(Ad,1), N_steps);
x_est  = zeros(size(Ad,1), N_steps);
y_real = zeros(2, N_steps);
y_est  = zeros(2, N_steps);


% Condición inicial intencionalmente desfasada para visualizar la convergencia del estimador
x_real(:,1) = [0; 0];
x_est(:,1)  = [2; 2]; 


y_real(:,1) = Cd * x_real(:,1);
y_est(:,1)  = Cd * x_est(:,1);


% 5. Bucle del Observador Predictor de Luenberger 
for k = 2:N_steps
    % --- Planta Real ---
    % Simulación del comportamiento real del motor
    x_real(:, k) = Ad * x_real(:, k-1) + Bd * u(k-1);
    y_real(:, k) = Cd * x_real(:, k);
    
    % --- Observador de Luenberger ---
    % La medición de retroalimentación (y_meas) usa solo el dato de velocidad del instante anterior
    y_meas = y_real(1, k-1); 
    
    % Ecuación de predicción de orden completo: x_est(k) = (A-LC)*x_est(k-1) + L*y(k-1) + B*u(k-1)
    x_est(:, k) = (Ad - L * C_m) * x_est(:, k-1) + L * y_meas + Bd * u(k-1);
    
    % Estimación final de todas las salidas (Velocidad y Corriente) a partir de los estados
    y_est(:, k) = Cd * x_est(:, k);
end


% Conversión de unidades para visualización de la Velocidad
speed_real_rpm = y_real(1,:) * (60 / (2*pi));
speed_est_rpm  = y_est(1,:) * (60 / (2*pi));


current_real = y_real(2,:);
current_est  = y_est(2,:);


%% =====================================================
%% GRÁFICAS DE VALIDACIÓN
%% =====================================================


figure('Name', 'Punto 3: Observador de Luenberger', 'Color', 'w', 'Position', [100, 100, 800, 600]);


% Gráfica de Velocidad (Observada vs Real)
subplot(2,1,1);
plot(t_sim, speed_real_rpm, 'b', 'LineWidth', 2); hold on;
plot(t_sim, speed_est_rpm, 'r--', 'LineWidth', 1.8);
grid on; box on;
title('Estimación de Velocidad usando Luenberger (Entrada 12V)');
ylabel('Velocidad (RPM)');
legend('Velocidad Real', 'Velocidad Estimada', 'Location', 'southeast');


% Gráfica de Corriente (Observada vs Real)
subplot(2,1,2);
plot(t_sim, current_real, 'b', 'LineWidth', 2); hold on;
plot(t_sim, current_est, 'g--', 'LineWidth', 1.8);
grid on; box on;
title('Estimación de Corriente basada en la Medición de Velocidad');
xlabel('Tiempo (s)');
ylabel('Corriente (A)');
legend('Corriente Real', 'Corriente Estimada', 'Location', 'southeast');