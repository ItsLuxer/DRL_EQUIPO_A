function y = KE88_dibujo(x)
%KE88_DIBUJO  Vista 3D en vivo del KE88 durante la simulacion.
%
% VERSION CON MODELO STL:
%   En vez de dibujar el dron como un esquema de lineas y marcadores,
%   esta version carga el modelo 3D "DronNaya.STL" (debe estar en la
%   misma carpeta que este archivo) y lo mueve/rota en cada paso usando
%   un hgtransform. El STL se lee una sola vez (persistent) porque leer
%   y re-dibujar la malla completa en cada paso seria muy lento.
%
%   pos(3) = max(pos(3), 0): la posicion vertical usada para dibujar
%   nunca es negativa (el dron nunca se dibuja bajo el suelo).
%
% AJUSTES SI EL MODELO SE VE MAL ORIENTADO (variables mas abajo):
%   ESCALA_STL      -> tamano del modelo. [] = se calcula automaticamente
%                      para que el ancho del modelo coincida con la
%                      separacion diagonal motor-motor definida en
%                      KE88_parametros.m (P.dx, P.dy). Si el dron se ve
%                      demasiado grande o chico, fija aqui un numero
%                      (metros por unidad del STL) en vez de [].
%   YAW_OFFSET_DEG  -> gira el modelo alrededor de su eje vertical (Z).
%                      Si el dron "vuela de lado" o "para atras" respecto
%                      a la trayectoria, prueba con 45, 90, -45, 135, etc.
%                      hasta que el morro apunte en la direccion de vuelo.

persistent ax hT tr X t0
y = 0; x = x(:);

% ---------- parametros de ajuste del modelo STL ----------
ESCALA_STL     = 0.020;
YAW_OFFSET_DEG = 0;

if isempty(ax) || ~isgraphics(ax)
    f = figure(99); clf(f); set(f,'Name','KE88 en vivo (STL)','Color','w');
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on'); view(ax,45,20);
    axis(ax,'equal');
    [gx,gy] = meshgrid(-3:1:3);
    surf(ax,gx,gy,0*gx,'FaceColor',[0.9 0.95 0.9],'EdgeColor',[0.8 0.85 0.8]);

    % --- cargar el modelo STL (una sola vez) ---
    ruta_stl = fullfile(fileparts(mfilename('fullpath')), 'DronNaya.STL');
    TR = stlread(ruta_stl);
    V  = TR.Points;                 % vertices en coordenadas nativas del STL
    F  = TR.ConnectivityList;

    % centrar el modelo en el centro de su caja envolvente
    c = (max(V,[],1) + min(V,[],1)) / 2;
    V = V - c;

    % --- remapear ejes nativos del STL a ejes cuerpo (x=adelante, y=izq, z=arriba) ---
    % En DronNaya.STL el eje vertical del dron (el mas "delgado") es la
    % Y nativa, y el plano de las helices es el plano X-Z nativo.
    % Esta es una rotacion propia (no espeja el modelo).
    Vb = [V(:,3), V(:,1), V(:,2)];   % body_x=Z_nativa, body_y=X_nativa, body_z=Y_nativa

    % rotacion de ajuste de guinada (ver YAW_OFFSET_DEG arriba)
    cy = cosd(YAW_OFFSET_DEG); sy = sind(YAW_OFFSET_DEG);
    Ry = [cy -sy 0; sy cy 0; 0 0 1];
    Vb = (Ry * Vb')';

    % --- escalar el modelo al tamano real del dron ---
    global P
    if isempty(P), KE88_parametros; end
    diag_real = 2*sqrt(P.dx^2 + P.dy^2);        % separacion diagonal motor-motor [m]
    span      = Vb(:,[1 2]);
    diag_stl  = norm(max(span,[],1) - min(span,[],1));
    if isempty(ESCALA_STL)
        escala = diag_real / diag_stl;
    else
        escala = ESCALA_STL;
    end
    Vb = Vb * escala;

    hT = hgtransform('Parent',ax);
    patch('Parent',hT,'Faces',F,'Vertices',Vb, ...
        'FaceColor',[0.25 0.55 0.85],'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.4);
    camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull');

    tr = plot3(ax,nan,nan,nan,'b-');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    X = []; t0 = tic;
end

if toc(t0) < 0.08, return; end
t0 = tic;

pos = x(1:3);

% *** CLAMPING: nunca dibujar bajo el suelo ***
pos(3) = max(pos(3), 0);

ph = x(7); th = x(8); ps = x(9);
R = [cos(ps)*cos(th), cos(ps)*sin(th)*sin(ph)-sin(ps)*cos(ph), cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph);
     sin(ps)*cos(th), sin(ps)*sin(th)*sin(ph)+cos(ps)*cos(ph), sin(ps)*sin(th)*cos(ph)-cos(ps)*sin(ph);
     -sin(th), cos(th)*sin(ph), cos(th)*cos(ph)];

Tm = eye(4); Tm(1:3,1:3) = R; Tm(1:3,4) = pos;
set(hT,'Matrix',Tm);

X(:,end+1) = pos;
if size(X,2) > 400, X = X(:,2:end); end

% Clamping de estela
set(tr,'XData',X(1,:),'YData',X(2,:),'ZData',max(X(3,:),0));

% Camara: Z siempre desde 0
axis(ax,[pos(1)-1.5 pos(1)+1.5  pos(2)-1.5 pos(2)+1.5  0  max(2, pos(3)+1)]);
title(ax,sprintf('KE88 en vivo (STL)  |  altura z = %.2f m', pos(3)));
drawnow limitrate
end
