%% DALTONICS_construir_modelo.m
% Construye automaticamente el gemelo digital del "Dron Daltonics"
% (dron propio del equipo, 530 g, ensamblado):
%
%   1. DALTONICS_control.slx        - control virtual (sliders de vuelo)
%   2. DALTONICS_dron.slx           - controlador PID + motores + dinamica 6DOF
%   3. DALTONICS_visualizacion.slx  - scopes y registro de datos
%   4. DALTONICS_Arquitectura.slx   - modelo de ARQUITECTURA en System Composer
%                                     (primera opcion: "Architecture Model")
%                                     que conecta los 3 componentes anteriores.
%
% Mismo patron que KE88_construir_modelo.m (Dron_Digital/MATLAB_DRIVE),
% pero apuntando a DALTONICS_dinamica.m / DALTONICS_parametros.m. No
% depende de ningun archivo de Dron_Digital.


clc;
fprintf('=== Construyendo gemelo digital DALTONICS ===\n');
DALTONICS_parametros;                  % carga la estructura global P


m1 = 'DALTONICS_control';
cerrar_si_abierto(m1);
new_system(m1); load_system(m1);
set_param(m1,'InitFcn','DALTONICS_parametros');

cmds = { % nombre        valor  min    max     etiqueta
        'Throttle'      '40'   '0'    '100'   'Acelerador  [%]';
        'Roll'          '0'    '-25'  '25'    'Alabeo: derecha +  [deg]';
        'Pitch'         '0'    '-25'  '25'    'Cabeceo: adelante +  [deg]';
        'YawRate'       '0'    '-120' '120'   'Guinada: izquierda +  [deg/s]'};
outNames = {'thr_pct','roll_deg','pitch_deg','yawrate_dps'};

for k = 1:4
    y = 80 + (k-1)*110;
    cte = [m1 '/' cmds{k,1}];
    add_block('simulink/Sources/Constant', cte, ...
        'Value',cmds{k,2}, 'Position',[260 y 320 y+30]);
    add_block('simulink/Sinks/Out1', [m1 '/' outNames{k}], ...
        'PortDimensions','1','Position',[420 y+5 450 y+25]);
    add_line(m1, [cmds{k,1} '/1'], [outNames{k} '/1'], 'autorouting','on');
    try
        sl = [m1 '/Slider_' cmds{k,1}];
        add_block('simulink/Dashboard/Slider', sl, ...
            'Position',[60 y-20 200 y+60]);
        psi_ = Simulink.HMI.ParamSourceInfo(Simulink.BlockPath(cte));
        psi_.ParamName = 'Value';
        set_param(sl,'Binding',psi_);
        set_param(sl,'Minimum',cmds{k,3},'Maximum',cmds{k,4});
        set_param(sl,'Label','Top');
    catch ME
        warning('Slider %s: conectalo manualmente (%s)',cmds{k,1},ME.message);
    end
end
try
    a = Simulink.Annotation(m1,'CONTROL VIRTUAL DALTONICS - mueve los sliders durante la simulacion');
    a.Position = [60 15];
catch, end
save_system(m1);
fprintf('  [OK] %s.slx\n', m1);


m2 = 'DALTONICS_dron';
cerrar_si_abierto(m2);
new_system(m2); load_system(m2);
set_param(m2,'InitFcn','DALTONICS_parametros');

% --- entradas ---
inN = {'thr_pct','roll_deg','pitch_deg','yawrate_dps'};
for k = 1:4
    y = 60 + (k-1)*90;
    add_block('simulink/Sources/In1',[m2 '/' inN{k}], ...
        'PortDimensions','1','Position',[40 y 70 y+20]);
end

% --- conversion de comandos ---
add_block('simulink/Math Operations/Gain',[m2 '/Thr_a_Newtons'], ...
    'Gain','4*P.Tmax/100','Position',[140 55 210 90]);
add_line(m2,'thr_pct/1','Thr_a_Newtons/1','autorouting','on');

d2r = {'D2R_roll','D2R_pitch','D2R_yaw'};
for k = 1:3
    y = 150 + (k-1)*90;
    add_block('simulink/Math Operations/Gain',[m2 '/' d2r{k}], ...
        'Gain','pi/180','Position',[140 y 200 y+30]);
    add_line(m2,[inN{k+1} '/1'],[d2r{k} '/1'],'autorouting','on');
end

% --- realimentacion de estados (selectores) ---
sels = { % nombre       indices  y
        'Sel_phi'      '7'      150;
        'Sel_theta'    '8'      240;
        'Sel_r'        '12'     330};
for k = 1:3
    y = sels{k,3};
    add_block('simulink/Signal Routing/Selector',[m2 '/' sels{k,1}], ...
        'InputPortWidth','12','Indices',sels{k,2}, ...
        'Position',[140 y+45 200 y+75]);
end

% --- lazos PID de actitud ---
add_block('simulink/Continuous/PID Controller (2DOF)',[m2 '/PID_Roll'], ...
    'P','P.Kp_ang','I','P.Ki_ang','D','P.Kd_ang','N','P.N', ...
    'b','1','c','0','Position',[320 150 390 190]);
add_block('simulink/Continuous/PID Controller (2DOF)',[m2 '/PID_Pitch'], ...
    'P','P.Kp_ang','I','P.Ki_ang','D','P.Kd_ang','N','P.N', ...
    'b','1','c','0','Position',[320 240 390 280]);
add_block('simulink/Math Operations/Sum',[m2 '/e_yaw'], ...
    'Inputs','+-','Position',[250 335 280 365]);
add_block('simulink/Continuous/PID Controller',[m2 '/PID_Yaw'], ...
    'Controller','PI','P','P.Kp_yaw','I','P.Ki_yaw', ...
    'Position',[320 330 390 370]);

add_line(m2,'D2R_roll/1' ,'PID_Roll/1' ,'autorouting','on');
add_line(m2,'Sel_phi/1'  ,'PID_Roll/2' ,'autorouting','on');
add_line(m2,'D2R_pitch/1','PID_Pitch/1','autorouting','on');
add_line(m2,'Sel_theta/1','PID_Pitch/2','autorouting','on');
add_line(m2,'D2R_yaw/1'  ,'e_yaw/1'    ,'autorouting','on');
add_line(m2,'Sel_r/1'    ,'e_yaw/2'    ,'autorouting','on');
add_line(m2,'e_yaw/1'    ,'PID_Yaw/1'  ,'autorouting','on');

% --- mezcladora + saturacion + motores ---
add_block('simulink/Signal Routing/Mux',[m2 '/Mux_Ftau'], ...
    'Inputs','4','Position',[440 150 445 370]);
add_line(m2,'Thr_a_Newtons/1','Mux_Ftau/1','autorouting','on');
add_line(m2,'PID_Roll/1'  ,'Mux_Ftau/2','autorouting','on');
add_line(m2,'PID_Pitch/1' ,'Mux_Ftau/3','autorouting','on');
add_line(m2,'PID_Yaw/1'   ,'Mux_Ftau/4','autorouting','on');

add_block('simulink/Math Operations/Gain',[m2 '/Mezcladora'], ...
    'Gain','P.Mix','Multiplication','Matrix(K*u)', ...
    'Position',[480 235 550 285]);
add_block('simulink/Discontinuities/Saturation',[m2 '/Sat_Motores'], ...
    'LowerLimit','0','UpperLimit','P.Tmax','Position',[580 240 630 280]);
add_block('simulink/Continuous/State-Space',[m2 '/Motores'], ...
    'A','-eye(4)/P.tau_m','B','eye(4)/P.tau_m', ...
    'C','eye(4)','D','zeros(4,4)','Position',[660 235 730 285]);
add_line(m2,'Mux_Ftau/1'   ,'Mezcladora/1' ,'autorouting','on');
add_line(m2,'Mezcladora/1' ,'Sat_Motores/1','autorouting','on');
add_line(m2,'Sat_Motores/1','Motores/1'    ,'autorouting','on');

% --- dinamica 6DOF: [estados; empujes] -> xdot -> integrador ---
add_block('simulink/Signal Routing/Mux',[m2 '/Mux_xu'], ...
    'Inputs','2','Position',[790 230 795 300]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [m2 '/Dinamica_6DOF'],'MATLABFcn','DALTONICS_dinamica(u)', ...
    'OutputDimensions','12','Position',[830 240 930 290]);
add_block('simulink/Continuous/Integrator',[m2 '/Integrador'], ...
    'InitialCondition','P.x0','Position',[970 245 1010 285]);
add_line(m2,'Motores/1'     ,'Mux_xu/2'       ,'autorouting','on');
add_line(m2,'Mux_xu/1'      ,'Dinamica_6DOF/1','autorouting','on');
add_line(m2,'Dinamica_6DOF/1','Integrador/1'  ,'autorouting','on');
add_line(m2,'Integrador/1'  ,'Mux_xu/1'       ,'autorouting','on');  % lazo
add_line(m2,'Integrador/1'  ,'Sel_phi/1'      ,'autorouting','on');
add_line(m2,'Integrador/1'  ,'Sel_theta/1'    ,'autorouting','on');
add_line(m2,'Integrador/1'  ,'Sel_r/1'        ,'autorouting','on');

% --- salidas ---
add_block('simulink/Sinks/Out1',[m2 '/estados'],'PortDimensions','12','Position',[1060 255 1090 275]);
add_block('simulink/Sinks/Out1',[m2 '/empujes'],'PortDimensions','4','Position',[1060 355 1090 375]);
add_line(m2,'Integrador/1','estados/1','autorouting','on');
add_line(m2,'Motores/1'   ,'empujes/1','autorouting','on');
save_system(m2);
fprintf('  [OK] %s.slx\n', m2);


m3 = 'DALTONICS_visualizacion';
cerrar_si_abierto(m3);
new_system(m3); load_system(m3);
set_param(m3,'InitFcn','DALTONICS_parametros');

add_block('simulink/Sources/In1',[m3 '/estados'],'PortDimensions','12','Position',[40 100 70 120]);
add_block('simulink/Sources/In1',[m3 '/empujes'],'PortDimensions','4','Position',[40 300 70 320]);

add_block('simulink/Signal Routing/Selector',[m3 '/Posicion_XYZ'], ...
    'InputPortWidth','12','Indices','[1 2 3]','Position',[150 70 220 100]);
add_block('simulink/Signal Routing/Selector',[m3 '/Angulos_Euler'], ...
    'InputPortWidth','12','Indices','[7 8 9]','Position',[150 150 220 180]);
add_block('simulink/Math Operations/Gain',[m3 '/R2D'], ...
    'Gain','180/pi','Position',[260 150 310 180]);

add_block('simulink/Sinks/Scope',[m3 '/Scope_Posicion'],'Position',[380 65 430 105]);
add_block('simulink/Sinks/Scope',[m3 '/Scope_Angulos'],'Position',[380 145 430 185]);
add_block('simulink/Sinks/Scope',[m3 '/Scope_Motores'],'Position',[380 290 430 330]);

add_block('simulink/Sinks/To Workspace',[m3 '/log_estados'], ...
    'VariableName','x_log','SaveFormat','Structure With Time', ...
    'Position',[380 220 450 250]);
add_block('simulink/Sinks/To Workspace',[m3 '/log_empujes'], ...
    'VariableName','T_log','SaveFormat','Structure With Time', ...
    'Position',[380 360 450 390]);

add_line(m3,'estados/1','Posicion_XYZ/1' ,'autorouting','on');
add_line(m3,'estados/1','Angulos_Euler/1','autorouting','on');
add_line(m3,'estados/1','log_estados/1'  ,'autorouting','on');
add_line(m3,'Posicion_XYZ/1' ,'Scope_Posicion/1','autorouting','on');
add_line(m3,'Angulos_Euler/1','R2D/1'           ,'autorouting','on');
add_line(m3,'R2D/1'          ,'Scope_Angulos/1' ,'autorouting','on');
add_line(m3,'empujes/1','Scope_Motores/1','autorouting','on');
add_line(m3,'empujes/1','log_empujes/1'  ,'autorouting','on');
save_system(m3);
fprintf('  [OK] %s.slx\n', m3);


archOK = false;
try
    ma = 'DALTONICS_Arquitectura';
    cerrar_si_abierto(ma);
    modelo = systemcomposer.createModel(ma);
    arq    = modelo.Architecture;

    cCtrl = addComponent(arq,'Control_Virtual');
    cDron = addComponent(arq,'Dron_Daltonics');
    cVis  = addComponent(arq,'Visualizacion');

    linkToModel(cCtrl,'DALTONICS_control');
    linkToModel(cDron,'DALTONICS_dron');
    linkToModel(cVis ,'DALTONICS_visualizacion');

    senales = {'thr_pct','roll_deg','pitch_deg','yawrate_dps'};
    for k = 1:4
        connect(arq, getPort(cCtrl,senales{k}), getPort(cDron,senales{k}));
    end
    connect(arq, getPort(cDron,'estados'), getPort(cVis,'estados'));
    connect(arq, getPort(cDron,'empujes'), getPort(cVis,'empujes'));

    set_param(ma,'StopTime','120','MaxStep','0.02', ...
                 'InitFcn','DALTONICS_parametros');
    try
        set_param(ma,'EnablePacing','on','PacingRate','1');
    catch, end
    save(modelo);
    Simulink.BlockDiagram.arrangeSystem(ma);
    save_system(ma);
    open_system(ma);
    archOK = true;
    fprintf('  [OK] %s.slx (System Composer)\n', ma);
catch ME
    warning('System Composer no disponible o fallo: %s', ME.message);
end

%% ------------------------------------------------------------------
%  RESPALDO: si no hay System Composer, modelo Simulink equivalente
%% ------------------------------------------------------------------
if ~archOK
    ms = 'DALTONICS_Sim';
    cerrar_si_abierto(ms);
    new_system(ms); load_system(ms);
    set_param(ms,'StopTime','120','MaxStep','0.02','InitFcn','DALTONICS_parametros');
    try set_param(ms,'EnablePacing','on','PacingRate','1'); catch, end

    add_block('simulink/Ports & Subsystems/Model',[ms '/Control_Virtual'], ...
        'ModelName','DALTONICS_control','Position',[60 100 220 260]);
    add_block('simulink/Ports & Subsystems/Model',[ms '/Dron_Daltonics'], ...
        'ModelName','DALTONICS_dron','Position',[320 100 480 260]);
    add_block('simulink/Ports & Subsystems/Model',[ms '/Visualizacion'], ...
        'ModelName','DALTONICS_visualizacion','Position',[580 130 740 230]);
    for k = 1:4
        add_line(ms,sprintf('Control_Virtual/%d',k), ...
                    sprintf('Dron_Daltonics/%d',k),'autorouting','on');
    end
    add_line(ms,'Dron_Daltonics/1','Visualizacion/1','autorouting','on');
    add_line(ms,'Dron_Daltonics/2','Visualizacion/2','autorouting','on');
    save_system(ms); open_system(ms);
    fprintf('  [OK] %s.slx (respaldo sin System Composer)\n', ms);
end

fprintf('\n=== Listo. Presiona RUN y vuela con los sliders de Control_Virtual ===\n');
fprintf('    Hover ~47.6%% de throttle. Al terminar: DALTONICS_animacion(out)\n');
fprintf('    RECUERDA: ganancias PID son punto de partida, no definitivas.\n');

%% ------------------------------------------------------------------
function cerrar_si_abierto(nombre)
if bdIsLoaded(nombre), close_system(nombre,0); end
if exist([nombre '.slx'],'file'), delete([nombre '.slx']); end
end
