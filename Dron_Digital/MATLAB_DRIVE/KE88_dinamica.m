function xdot = KE88_dinamica(u)
%KE88_DINAMICA  Dinamica 6DOF del dron KE88 con restriccion robusta de suelo.
%
% Entrada u (16x1):
%   u(1:12) = estado x = [pos(3); vel(3); euler(3); omega_cuerpo(3)]
%   u(13:16)= empujes de motores [T1;T2;T3;T4] en Newtons
% Salida xdot (12x1) = derivada del estado
%
% CAMBIOS vs version original:
%   1. Suelo con resorte-amortiguador: en lugar de solo anular xdot,
%      se aplica una fuerza de reaccion proporcional a la penetracion (z<0)
%      y una fuerza de amortiguamiento de impacto. Esto es fisicamente
%      correcto y evita que el integrador Simulink acumule error negativo en z.
%   2. Se satura xdot(3) con max(...,0) SOLO cuando el dron ya esta en suelo
%      y con velocidad negativa, como capa adicional de seguridad.
%   3. Friccion horizontal realista cuando esta apoyado.

global P
if isempty(P)
    KE88_parametros;
end

% ---------- desempacar estado y empujes ----------
x   = u(1:12);
T   = u(13:16);
pos = x(1:3);   v   = x(4:6);
eul = x(7:9);   om  = x(10:12);
phi = eul(1);   th  = eul(2);   psi = eul(3);

% ---------- matriz de rotacion cuerpo->mundo (ZYX, z arriba) ----------
cph=cos(phi); sph=sin(phi);
cth=cos(th);  sth=sin(th);
cps=cos(psi); sps=sin(psi);
R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph ;
      sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph ;
     -sth    ,  cth*sph              ,  cth*cph               ];

% ---------- fuerzas aereas (marco mundo) ----------
Ftot = sum(T);
acc  = [0;0;-P.g] + R*[0;0;Ftot]/P.m - (P.kd_lin/P.m)*v;

% ---------- pares (marco cuerpo) ----------
tau = [ P.dy*( T(1) - T(2) - T(3) + T(4));
       -P.dx*( T(1) + T(2) - T(3) - T(4));
        P.c *(-T(1) + T(2) - T(3) + T(4))]...
      - P.kd_ang*om;

I     = diag([P.Ixx P.Iyy P.Izz]);
omdot = I \ (tau - cross(om, I*om));

% ---------- cinematica de Euler ----------
W = [1, sph*tan(th), cph*tan(th);
     0, cph        , -sph       ;
     0, sph/cth    , cph/cth    ];
euldot = W*om;

% ---------- ensamblar derivada libre ----------
xdot = [v; acc; euldot; omdot];

% ==========================================================
%  RESTRICCION DE SUELO  (CAMBIO PRINCIPAL)
%  Modelo resorte-amortiguador:
%    F_suelo = -Kp * penetracion  -  Kd * velocidad_vertical
%  Solo actua cuando z < 0 (penetra el suelo).
%  Los parametros Kp y Kd se eligieron para que:
%    - frenen el impacto sin rebotar en exceso
%    - sean compatibles con el paso maximo de integracion (0.02 s)
% ==========================================================
z  = pos(3);
vz = v(3);

if z < 0
    % Profundidad de penetracion (positiva cuando z<0)
    penetracion = -z;   % > 0

    % Coeficientes del modelo de contacto con el suelo
    Kp_suelo = 500;     % N/m  - rigidez (evita penetracion profunda)
    Kd_suelo = 60;      % N*s/m - amortiguamiento de impacto

    % Fuerza de reaccion normal (hacia arriba, en marco mundo)
    F_reaccion = Kp_suelo * penetracion - Kd_suelo * vz;
    F_reaccion = max(F_reaccion, 0);   % nunca jala hacia abajo

    % Convertir a aceleracion y sumarla al estado vertical
    a_reaccion = F_reaccion / P.m;
    xdot(6) = xdot(6) + a_reaccion;   % aceleracion z

    % Friccion horizontal de rodadura/deslizamiento cuando z < 0
    % (solo cuando el dron esta realmente apoyado, no en vuelo)
    if vz <= 0.05
        xdot(4:5) = xdot(4:5) - 8.0 * v(1:2);    % frena traslacion XY
        xdot(10:12) = xdot(10:12) - 12.0 * om;    % frena rotacion
        xdot(7:8)   = xdot(7:8) - 3.0 * eul(1:2); % tiende a nivelarse
    end
end

% ---------- Capa de seguridad adicional ----------
% Si z <= 0 y el dron va hacia abajo, bloquear la derivada de posicion z.
% Esto es un "hard floor" que complementa el modelo de resorte:
% aunque el integrador acumule un poco de error, la derivada de posicion z
% nunca empujara al dron mas abajo del suelo.
if pos(3) <= 0 && v(3) < 0
    xdot(3) = max(xdot(3), 0);   % vel z no puede bajar mas
end

end