function Control_Vuelo_Daltonics()
% CONTROL_VUELO_DALTONICS - Mando de tierra con despegue/aterrizaje por tecla
%
% Pareja del firmware Firmware/Daltonics_Vuelo/Daltonics_Vuelo.ino.
% Usa el mismo ESP32_PRUEBATIERRA.ino de siempre (sin cambios): la trama
% es la misma, solo que el campo A ahora manda el MODO (0-3).
%
%   TECLAS DE MODO:
%     E       -> ARMAR: motores en idle, girando sin despegar
%     T       -> DESPEGUE: rampa suave hasta gas de hover, se queda nivelado
%     L       -> ATERRIZAR: rampa descendente automatica hasta idle
%     Q       -> DESARMAR (corte total; tambien limpia una emergencia)
%
%   TECLAS DE VUELO:
%     W / S   -> Trim de gas de hover +/- 0.5% (solo importa en vuelo)
%     Flechas -> Roll / Pitch momentaneo
%     A / D   -> Yaw momentaneo
%     ESPACIO -> Nivelar (roll/pitch/yaw a 0)

    global CMD MODO

    % Puerto COM del ESP32 de Tierra (ajustar segun tu laptop)
    puertoCOM = "COM10";
    baudios = 115200;

    HOVER_PCT = 45.0;   % gas inicial de hover (1450us medido en banco --
                        % con fuente dudosa: ajustar con W/S en el aire)
    PASO_TRIM = 0.5;    % W/S: +/- 0.5% por pulsacion

    % Mismas unidades que el gemelo digital (DALTONICS_mando.m):
    % flechas = +/-12 grados de inclinacion, A/D = +/-60 dps de yaw.
    % El firmware mapea 100% de stick a 25 deg y 120 dps, por eso la
    % conversion a porcentaje de stick:
    ANG_MOVER_DEG = 12;  PASO_RP  = ANG_MOVER_DEG / 25 * 100;   % = 48%
    YAW_GIRO_DPS  = 60;  PASO_YAW = YAW_GIRO_DPS / 120 * 100;   % = 50%

    % [Throttle; Roll; Pitch; Yaw] en porcentaje
    CMD = [0; 0; 0; 0];
    MODO = 0;  % 0=desarmado 1=idle 2=vuelo 3=aterrizaje

    % Ultima telemetria recibida del dron (via Tierra: lineas "TL,...")
    telem = struct('ok',false,'roll',0,'pitch',0,'thr',0,'modo',0,...
                   'emg',false,'fs',false,'atz',false,'mpu',false,'t',0);

    % Abrir puerto serial
    try
        espTierra = serialport(puertoCOM, baudios);
        fprintf('==================================================\n');
        fprintf('  CONECTADO AL CONTROL DE TIERRA EN %s\n', puertoCOM);
        fprintf('==================================================\n');
    catch
        error('No se pudo abrir el puerto %s. Revisa la conexion USB.', puertoCOM);
    end

    % Ventana interactiva
    f = figure(100); clf(f);
    set(f,'Name','ESCUDERIA DALTONICS - CONTROL DE VUELO','Color','w','NumberTitle','off');
    txt = uicontrol(f,'Style','text','Units','normalized',...
        'Position',[0.05 0.05 0.9 0.9],'FontSize',12,'FontName','FixedWidth',...
        'HorizontalAlignment','left','BackgroundColor','w');

    set(f,'KeyPressFcn',@tecla,'KeyReleaseFcn',@suelta);

    % Envio periodico a 50 Hz
    tSend = timer('ExecutionMode', 'fixedRate', 'Period', 0.02, ...
                  'TimerFcn', @(~,~) enviarTramaSerial(espTierra, txt));
    start(tSend);
    set(f, 'CloseRequestFcn', @(~,~) cerrarPrograma(tSend, espTierra, f));

    % --- TECLADO ---
    function tecla(~,e)
        switch e.Key
            % ----- MODOS -----
            case 'e'
                if MODO == 0
                    MODO = 1; CMD = [0; 0; 0; 0];
                    fprintf('>> ARMADO: motores en IDLE\n');
                end
            case 't'
                if MODO == 1 || MODO == 3
                    MODO = 2;
                    CMD(1) = HOVER_PCT; CMD(2:4) = 0;
                    fprintf('>> DESPEGUE: rampa hacia %.1f%% de gas\n', CMD(1));
                end
            case 'l'
                if MODO == 2
                    MODO = 3; CMD(2:4) = 0;
                    fprintf('>> ATERRIZANDO (rampa automatica en el dron)\n');
                end
            case 'q'
                MODO = 0; CMD = [0; 0; 0; 0];
                fprintf('>> DESARMADO / CORTE\n');

            % ----- VUELO (mismas unidades que el gemelo digital) -----
            case 'w',          if MODO == 2, CMD(1) = min(70, CMD(1) + PASO_TRIM); end
            case 's',          if MODO == 2, CMD(1) = max(20, CMD(1) - PASO_TRIM); end
            case 'r',          if MODO == 2, CMD(1) = HOVER_PCT; end % reset hover
            case 'uparrow',    CMD(3) =  PASO_RP;   % adelante  (+12 deg)
            case 'downarrow',  CMD(3) = -PASO_RP;   % atras     (-12 deg)
            case 'rightarrow', CMD(2) =  PASO_RP;   % derecha   (+12 deg)
            case 'leftarrow',  CMD(2) = -PASO_RP;   % izquierda (-12 deg)
            case 'a',          CMD(4) =  PASO_YAW;  % girar izq (+60 dps)
            case 'd',          CMD(4) = -PASO_YAW;  % girar der (-60 dps)
            case 'space',      CMD(2:4) = 0;
        end
    end

    function suelta(~,e)
        switch e.Key
            case {'uparrow','downarrow'},    CMD(3) = 0;
            case {'leftarrow','rightarrow'}, CMD(2) = 0;
            case {'a','d'},                  CMD(4) = 0;
        end
    end

    function enviarTramaSerial(puerto, textoUI)
        % --- Leer telemetria del dron (lineas "TL,roll,pitch,thr,modo,flags") ---
        while puerto.NumBytesAvailable > 0
            linea = readline(puerto);
            if startsWith(linea, "TL,")
                v = sscanf(linea, 'TL,%f,%f,%f,%d,%d');
                if numel(v) == 5
                    telem.roll  = v(1);  telem.pitch = v(2);
                    telem.thr   = v(3);  telem.modo  = v(4);
                    fl          = v(5);
                    telem.emg   = bitand(fl, 1) > 0;
                    telem.fs    = bitand(fl, 2) > 0;
                    telem.atz   = bitand(fl, 4) > 0;
                    telem.mpu   = bitand(fl, 8) > 0;
                    telem.ok    = true;
                    telem.t     = tic;
                end
            end
        end

        u_throttle = round(1000 + (CMD(1) * 10));
        u_roll     = round(1500 + (CMD(2) * 5));
        u_pitch    = round(1500 + (CMD(3) * 5));
        u_yaw      = round(1500 + (CMD(4) * 5));

        trama = sprintf('T%04dR%04dP%04dY%04dA%d', ...
                        u_throttle, u_roll, u_pitch, u_yaw, MODO);
        writeline(puerto, trama);

        nombresModo = {'DESARMADO','IDLE (motores girando)','VUELO / HOVER','ATERRIZANDO'};

        % --- Bloque de telemetria del dron ---
        if telem.ok && toc(telem.t) < 1.0
            alerta = '';
            if telem.emg,      alerta = '  *** EMERGENCIA: >45 deg, motores cortados. Q y de nuevo E ***';
            elseif ~telem.mpu, alerta = '  *** SIN IMU: no puede armar ***';
            elseif telem.atz,  alerta = '  (aterrizado, en idle)';
            end
            lineaTelem = sprintf(['  DRON >> roll:%6.1f deg  pitch:%6.1f deg  gas:%5.1f %%  modo:%s\n%s'],...
                telem.roll, telem.pitch, telem.thr, nombresModo{telem.modo+1}, alerta);
        elseif telem.ok
            lineaTelem = '  DRON >> (telemetria perdida hace >1 s)';
        else
            lineaTelem = '  DRON >> (sin telemetria aun - esperando enlace)';
        end

        set(textoUI,'String',sprintf([...
            '=== ESCUDERIA DALTONICS - CONTROL DE VUELO ===\n\n',...
            '  MODO PEDIDO: %s\n\n',...
            '  Throttle: %5.1f %%\n  Roll:     %5.1f deg\n',...
            '  Pitch:    %5.1f deg\n  Yaw:      %5.1f dps\n\n',...
            '%s\n\n',...
            '  TRAMA (50 Hz): %s\n\n',...
            '  E: Armar (idle)   T: Despegue   L: Aterrizar   Q: DESARMAR\n',...
            '  W/S gas   R hover   flechas mover   A/D girar   ESPACIO nivelar\n\n',...
            '  LED del dron: verde=idle  azul=volando  amarillo=aterrizando\n',...
            '                celeste=aterrizado  rojo=EMERGENCIA (Q y de nuevo E)'],...
            nombresModo{MODO+1}, CMD(1), CMD(2)*25/100, CMD(3)*25/100, CMD(4)*120/100, ...
            lineaTelem, trama));
    end

    function cerrarPrograma(timerObj, puertoObj, fig)
        stop(timerObj);
        delete(timerObj);
        clear puertoObj;
        delete(fig);
        fprintf('\nPrograma finalizado y puerto COM liberado correctamente.\n');
    end
end
