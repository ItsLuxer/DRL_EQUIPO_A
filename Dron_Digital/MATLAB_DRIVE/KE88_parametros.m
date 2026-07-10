%% KE88_parametros.m
% Parametros fisicos del dron KIWO KE88 (Escuadron 201)
% Fuente: mediciones del equipo (COMPONENTES_DRON_.pdf) + estimaciones
% marcadas como [EST] que deben refinarse con mediciones reales.
%
% Este script se ejecuta automaticamente al iniciar la simulacion
% (esta registrado en el InitFcn del modelo KE88_Sim).
%
% Convencion de ejes (mundo y cuerpo):
%   x = adelante, y = izquierda, z = arriba
%   Angulos de Euler ZYX: phi (alabeo/roll), theta (cabeceo/pitch), psi (guinada/yaw)

global P    % P queda disponible para Simulink y para KE88_dinamica.m

% ---------- Masa y gravedad (MEDIDO) ----------
P.m  = 0.087;          % kg - dron completo CON proteccion de helices (87 g)
P.g  = 9.81;           % m/s^2

% ---------- Geometria [EST - medir con regla y actualizar] ----------
% El KE88 (clon del E88) desplegado mide ~13 cm de motor a motor en diagonal.
P.dx = 0.046;          % m - distancia del centro a cada motor en x
P.dy = 0.046;          % m - distancia del centro a cada motor en y

% ---------- Inercias [EST] ----------
% Estimadas tratando los 4 conjuntos motor+brazo como masas puntuales
% (placa con motores = 15 g medido) mas el cuerpo central (bateria 17 g,
% camara 1 g, placa 2 g, carcasa). Refinar con el banco de inercia (Semana 2/3).
P.Ixx = 6.2e-5;        % kg*m^2 - alabeo
P.Iyy = 6.2e-5;        % kg*m^2 - cabeceo
P.Izz = 1.1e-4;        % kg*m^2 - guinada

% ---------- Motores y helices [EST] ----------
% Motores brushed coreless (medido: 2.95 V, 58.7 mA en vacio).
% Empuje maximo tipico de este tipo de dron: relacion empuje/peso ~2.1
P.Tmax  = 0.45;        % N - empuje maximo POR MOTOR (4x0.45=1.8 N vs peso 0.853 N)
P.tau_m = 0.05;        % s - constante de tiempo del motor (retardo de 1er orden)
P.c     = 0.010;       % m - coef. de par de arrastre: par_motor = c * empuje

% ---------- Arrastre aerodinamico [EST] ----------
P.kd_lin = 0.08;       % N*s/m   - arrastre lineal (frena la traslacion)
P.kd_ang = 5e-6;       % N*m*s   - arrastre rotacional

% ---------- Mezcladora (mixer) ----------
% Numeracion de motores (vista superior, x adelante, y izquierda):
%   M1 = delantero-izq (+x,+y)   M2 = delantero-der (+x,-y)
%   M3 = trasero-der   (-x,-y)   M4 = trasero-izq   (-x,+y)
% [Ftotal; tau_x; tau_y; tau_z] = A * [T1;T2;T3;T4]
A = [ 1      1      1      1     ;    % empuje total
      P.dy  -P.dy  -P.dy   P.dy  ;    % alabeo  (izq arriba = +)
     -P.dx  -P.dx   P.dx   P.dx  ;    % cabeceo (adelante = +)
     -P.c    P.c   -P.c    P.c   ];   % guinada (izq = +)
P.Mix = inv(A);        % convierte [F;tau] deseados -> empujes por motor

% ---------- Ganancias PID (verificadas numericamente: ~6% sobrepaso, t95~0.5s) ----------
% PID 2-DOF con derivada sobre la medicion (c=0), como en controladores reales
P.Kp_ang = 0.002;      % PID de angulo (roll y pitch) -> par (N*m)
P.Ki_ang = 0.0005;
P.Kd_ang = 0.0006;
P.N      = 40;         % filtro del derivativo
P.Kp_yaw = 0.0012;     % PI de velocidad de guinada -> par (N*m)
P.Ki_yaw = 0.0005;

% ---------- Estado inicial ----------
% x = [pos(3); vel(3); euler(3); vel_angular_cuerpo(3)]
P.x0 = zeros(12,1);    % en el suelo, en reposo

% ---------- Dato util ----------
% Empuje de hover = m*g = 0.853 N  ->  throttle de hover ~ 47.4 %
