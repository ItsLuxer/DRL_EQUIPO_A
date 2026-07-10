function KE88_animacion(out)
%KE88_ANIMACION  Reproduce en 3D el vuelo simulado del KE88.
% Uso: despues de correr la simulacion en Simulink, escribe en la consola:
%       KE88_animacion(out)
%
% VERSION CON MODELO STL:
%   En vez de dibujar el dron como un esquema de lineas y marcadores, esta
%   version carga el modelo 3D "DronNaya.STL" (debe estar en la misma
%   carpeta que este archivo) y lo mueve/rota cuadro a cuadro usando un
%   hgtransform.
%
%   Se sigue aplicando el clamping pos(3) = max(pos(3), 0) en cada frame,
%   y el eje Z minimo de la camara siempre empieza en 0.
%
% AJUSTES SI EL MODELO SE VE MAL ORIENTADO:
%   ESCALA_STL      -> tamano del modelo. [] = automatico (ver mas abajo).
%   YAW_OFFSET_DEG  -> gira el modelo sobre su eje vertical si el morro
%                      no apunta en la direccion de vuelo (prueba 45, 90,
%                      -45, 135, etc).

if nargin < 1
    out = evalin('base','out');
end
t = out.x_log.time;
X = out.x_log.signals.values;      % N x 12

global P
if isempty(P), KE88_parametros; end

% ---------- parametros de ajuste del modelo STL ----------
ESCALA_STL     = 0.020;
YAW_OFFSET_DEG = 0;

% ---------- cargar y preparar el modelo STL ----------
ruta_stl = fullfile(fileparts(mfilename('fullpath')), 'DronNaya.STL');
TR = stlread(ruta_stl);
V  = TR.Points;
F  = TR.ConnectivityList;

c = (max(V,[],1) + min(V,[],1)) / 2;
V = V - c;

% remapeo de ejes nativos del STL a ejes cuerpo (x=adelante, y=izq, z=arriba)
% (el eje vertical del dron en este STL es la Y nativa; ver KE88_dibujo.m)
Vb = [V(:,3), V(:,1), V(:,2)];

cy = cosd(YAW_OFFSET_DEG); sy = sind(YAW_OFFSET_DEG);
Ry = [cy -sy 0; sy cy 0; 0 0 1];
Vb = (Ry * Vb')';

diag_real = 2*sqrt(P.dx^2 + P.dy^2);
span      = Vb(:,[1 2]);
diag_stl  = norm(max(span,[],1) - min(span,[],1));
if isempty(ESCALA_STL)
    escala = diag_real / diag_stl;
else
    escala = ESCALA_STL;
end
Vb = Vb * escala;

% ---------- figura ----------
fig = figure('Name','KE88 - Gemelo Digital','Color','w');
ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); view(ax,45,20);
axis(ax,'equal');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Simulacion KE88');

% suelo
[gx,gy] = meshgrid(-3:0.5:3);
surf(ax,gx,gy,0*gx,'FaceColor',[0.92 0.95 0.92],'EdgeColor',[0.8 0.85 0.8]);

hT = hgtransform('Parent',ax);
patch('Parent',hT,'Faces',F,'Vertices',Vb, ...
    'FaceColor',[0.25 0.55 0.85],'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.4);
camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull');

h_trail = plot3(ax,nan,nan,nan,'b-','LineWidth',1);

% submuestreo para ~25 FPS de vuelo
paso = max(1, round(numel(t)/(t(end)*25)));

for k = 1:paso:numel(t)
    if ~isvalid(fig), return; end

    pos = X(k,1:3)';
    eul = X(k,7:9)';

    % *** CLAMPING: el dron nunca se dibuja bajo el suelo ***
    pos(3) = max(pos(3), 0);

    R = rotZYX(eul);
    Tm = eye(4); Tm(1:3,1:3) = R; Tm(1:3,4) = pos;
    set(hT,'Matrix',Tm);

    % Estela: usar Z clamped para que no vaya bajo tierra
    trail_z = max(X(1:k,3)', 0);
    set(h_trail,'XData',X(1:k,1),'YData',X(1:k,2),'ZData',trail_z);

    % Camara: Z siempre >= 0
    axis(ax,[pos(1)-1.5 pos(1)+1.5  pos(2)-1.5 pos(2)+1.5  0  max(2, pos(3)+1)]);
    title(ax,sprintf('KE88  t = %.1f s   altura = %.2f m', t(k), pos(3)));
    drawnow limitrate
end
end

function R = rotZYX(eul)
phi=eul(1); th=eul(2); psi=eul(3);
cph=cos(phi); sph=sin(phi); cth=cos(th); sth=sin(th); cps=cos(psi); sps=sin(psi);
R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph ;
      sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph ;
     -sth    ,  cth*sph              ,  cth*cph               ];
end
