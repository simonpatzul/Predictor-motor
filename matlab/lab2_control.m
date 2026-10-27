clear
clc
close all

%% =====================================================
%% ORIGINAL MODEL
%% =====================================================

%% MODELO EN EL EJE DE SALIDA: REDUCCION 50:1

N = 50;

% Corregido CON polo rapido (tau = 0.04 s, Kv igual)
Gw     = tf(155212.362859*14/N, [1 988.607399 24090.185]);
Gtheta = tf(155212.362859*14/N, [1 988.607399 24090.185 0]);

% Corriente / voltaje (A por V), mismo denominador
Gi     = tf([452.705 401.5], [1 988.607399 24090.185]);


%% =====================================================
%% EXPERIMENTAL VALUES
%% =====================================================

rpm_target = 200;


%% =====================================================
%% SIMULATION
%% =====================================================

Vstep = 12;

t = 0:0.001:3;

%% Original responses

[w_original,tw] = step(Vstep*Gw,t);

[pos_original,tp] = step(Vstep*Gtheta,t);




%% =====================================================
%% CONVERT TO RPM
%% =====================================================

rpm_original_response = w_original*60/(2*pi);



%% =====================================================
%% FINAL VALUES
%% =====================================================

rpm_final_original = dcgain(Gw)*Vstep*60/(2*pi);






%% =====================================================
%% FIGURE
%% =====================================================

figure('Color','w','Position',[100 50 1200 1000])

%% =====================================================
%% SPEED
%% =====================================================

subplot(3,1,1)

plot(tw,rpm_original_response,...
    'b','LineWidth',2)



yline(rpm_target,...
    ':k',...
    'Experimental Value',...
    'LineWidth',2)

grid on
box on

title('Output Shaft Speed Response')

xlabel('Time [s]')
ylabel('Speed [RPM]')

legend('Original Model',...
       'Tuned Model',...
       'Experimental Value',...
       'Location','southeast')

%% =====================================================
%% POSITION
%% =====================================================

subplot(3,1,2)

plot(tp,pos_original,...
    'b','LineWidth',2)

hold on


grid on
box on

title('Output Shaft Position Response')

xlabel('Time [s]')
ylabel('Position [rad]')

legend('Original Model',...
       'Tuned Model',...
       'Location','northwest')


%% =====================================================
%% GLOBAL TITLE
%% =====================================================

sgtitle('DC Motor Model: Original vs Tuned Response')

%% =====================================================
%% RESULTS
%% =====================================================

fprintf('\n');
fprintf('========================\n');
fprintf('ORIGINAL MODEL\n');
fprintf('========================\n');
fprintf('Speed   = %.2f RPM\n',rpm_final_original);


fprintf('\n');
fprintf('========================\n');
fprintf('TUNED MODEL\n');
fprintf('========================\n');

fprintf('\n');
%% discretization 

Ts = 0.01;  % 10 ms, frecuencia de muestreo de 100 Hz

%% DISCRETIZACION DEL MODELO COMPLETO
Gw_d = c2d(Gw,Ts,'zoh');
Gtheta_d = c2d(Gtheta,Ts,'zoh');

Gw_d
Gtheta_d

%% COMPARACION DE RESPUESTAS
figure
step(12*Gw,12*Gw_d,3)
grid on
legend('Continuo','Discreto')
title('Comparacion de velocidad: continuo y discreto')


%%Corriente

figure

step(6*Gi)
grid on
legend('Continuo','Discreto')
title('Comparacion de velocidad: continuo y discreto')
