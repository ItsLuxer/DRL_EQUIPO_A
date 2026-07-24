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
    PASO_MANIOBRA = 50.0;

    % [Throttle; Roll; Pitch; Yaw] en porcentaje
    CMD = [0; 0; 0; 0];
    MODO = 0;  % 0=desarmado 1=idle 2=vuelo 3=aterrizaje

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

            % ----- VUELO -----
            case 'w',          if MODO == 2, CMD(1) = min(70, CMD(1) + PASO_TRIM); end
            case 's',          if MODO == 2, CMD(1) = max(20, CMD(1) - PASO_TRIM); end
            case 'uparrow',    CMD(3) = min(100, PASO_MANIOBRA);
            case 'downarrow',  CMD(3) = max(-100, -PASO_MANIOBRA);
            case 'rightarrow', CMD(2) = min(100, PASO_MANIOBRA);
            case 'leftarrow',  CMD(2) = max(-100, -PASO_MANIOBRA);
            case 'a',          CMD(4) = -PASO_MANIOBRA;
            case 'd',          CMD(4) =  PASO_MANIOBRA;
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
        u_throttle = round(1000 + (CMD(1) * 10));
        u_roll     = round(1500 + (CMD(2) * 5));
        u_pitch    = round(1500 + (CMD(3) * 5));
        u_yaw      = round(1500 + (CMD(4) * 5));

        trama = sprintf('T%04dR%04dP%04dY%04dA%d', ...
                        u_throttle, u_roll, u_pitch, u_yaw, MODO);
        writeline(puerto, trama);

        nombresModo = {'DESARMADO','IDLE (motores girando)','VUELO / HOVER','ATERRIZANDO'};
        set(textoUI,'String',sprintf([...
            '=== ESCUDERIA DALTONICS - CONTROL DE VUELO ===\n\n',...
            '  MODO: %s\n\n',...
            '  Gas hover: %5.1f %%   Roll: %5.1f   Pitch: %5.1f   Yaw: %5.1f\n\n',...
            '  TRAMA (50 Hz): %s\n\n',...
            '  E: Armar (idle)   T: Despegue   L: Aterrizar   Q: DESARMAR\n',...
            '  W/S: Trim gas     Flechas: Roll/Pitch   A/D: Yaw\n',...
            '  ESPACIO: Nivelar\n\n',...
            '  LED del dron: verde=idle  azul=volando  amarillo=aterrizando\n',...
            '                celeste=aterrizado  rojo=EMERGENCIA (Q y de nuevo E)'],...
            nombresModo{MODO+1}, CMD(1), CMD(2), CMD(3), CMD(4), trama));
    end

    function cerrarPrograma(timerObj, puertoObj, fig)
        stop(timerObj);
        delete(timerObj);
        clear puertoObj;
        delete(fig);
        fprintf('\nPrograma finalizado y puerto COM liberado correctamente.\n');
    end
end
