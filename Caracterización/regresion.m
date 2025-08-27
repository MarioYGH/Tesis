% Grafica ambas seniales, las medidas con multimetro y las que fueron con ESP, despues se saca la regresion con tool y basic fitting en matlab y seleccionamos la ecuacion que
% mejor se ajuste a la curva real, en este caso como es un convertidor ADC, queremos linealidad en sus mediciones, cosa que obtenemos al poner una regresion lineal simple
% Estraemos los puntos que en este caso son, p1 = 1, p2 = -2.8247e-17
clc;
clear all;
close all;

% Carga los datos
T = readmatrix("adc_multimetro_vs_esp32.csv");

multimetro = T(:,1);   % Voltajes reales
esp32      = T(:,2);   % Lecturas del ESP32

% Graficar como dos líneas
figure;
plot(multimetro, multimetro, 'bo-','LineWidth',1.5); hold on; % Línea ideal (y=x)
plot(multimetro, esp32, 'ro-','LineWidth',1.5);              % Medidas ESP32

grid on;
xlabel('Voltaje Multímetro [V]');
ylabel('Voltaje [V]');
title('Comparación Multímetro vs ESP32');
legend('Ideal (Multímetro)','ESP32','Location','northwest');
