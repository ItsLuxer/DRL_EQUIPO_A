xfunction Enviar_Mando_ESP32_Tierra()
    global CMD

    % Puerto COM de tu ESP32 de Tierra (Ajustar segun tu laptop)
    puertoCOM = "COM10"; 
    baudios = 115200;

    % 1. Inicializacion del vector de mando [Throttle; Roll; Pitch; Yaw]
    if isempty(CMD), CMD = [0; 0; 0; 0]; end % Empezamos con gas en 0% por seguridad

    % 2. Abrir puerto Serial con el ESP32 de Tierra
    try
        espTierra = serialport(puertoCOM, baudios);
        fprintf('==================================================\n');
        fprintf('  CONECTADO AL CONTROL DE TIERRA EN %s\n', puertoCOM);
        fprintf('==================================================\n');
    catch
        error('No se pudo abrir el puerto %s. Revisa la conexion USB.', puertoCOM);
    end

    % 3. Crear ventana interactiva para capturar el teclado
    f = figure(100); clf(f);
    set(f,'Name','ESCUDERIA DALTONICS - CONTROL TECLADO','Color','w','NumberTitle','off');
    txt = uicontrol(f,'Style','text','Units','normalized',...
        'Position',[0.05 0.05 0.9 0.9],'FontSize',12,'FontName','FixedWidth',...
        'HorizontalAlignment','left','BackgroundColor','w');

    set(f,'KeyPressFcn',@tecla,'KeyReleaseFcn',@suelta);

    % 4. Timer para envio periodico a 50 Hz (cada 20 ms)
    % NOTA: los callbacks de timer/figure siempre llaman con 2 argumentos
    % (objeto, evento). "@(,)" no es sintaxis valida en MATLAB -> "@(~,~)".
    tSend = timer('ExecutionMode', 'fixedRate', 'Period', 0.02, ...
                  'TimerFcn', @(~,~) enviarTramaSerial(espTierra, txt));
    start(tSend);

    % Detener el timer al cerrar la ventana
    set(f, 'CloseRequestFcn', @(~,~) cerrarPrograma(tSend, espTierra, f));

    % --- FUNCIONES INTERNAS DE TECLADO ---
    function tecla(~,e)
        paso_gas = 2.0;
        paso_maniobra = 50.0; % Inclinacion moderada de 50% al presionar

        switch e.Key
            case 'w',          CMD(1) = min(100, CMD(1) + paso_gas);
            case 's',          CMD(1) = max(0,   CMD(1) - paso_gas);
            case 'uparrow',    CMD(3) = min(100, CMD(3) + paso_maniobra);
            case 'downarrow',  CMD(3) = max(-100, CMD(3) - paso_maniobra);
            case 'rightarrow', CMD(2) = min(100, CMD(2) + paso_maniobra);
            case 'leftarrow',  CMD(2) = max(-100, CMD(2) - paso_maniobra);
            case 'a',          CMD(4) = -paso_maniobra;
            case 'd',          CMD(4) =  paso_maniobra;
            case 'space',      CMD(2:4) = 0; % Nivelar
            % TODO: 45% viene del PWM de hover medido en banco (1450 us,
            % ver potenciaVuelo del sketch de Aire) -> (1450-1000)/10 = 45.
            % Ajustar si cambia bateria/props/peso.
            case 'r',          CMD(1) = 45.0; % Reset a Hover (medido en banco)
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
        % Conversion de porcentajes a microsegundos (1000 - 2000)
        u_throttle = round(1000 + (CMD(1) * 10));
        u_roll     = round(1500 + (CMD(2) * 5));
        u_pitch    = round(1500 + (CMD(3) * 5));
        u_yaw      = round(1500 + (CMD(4) * 5));
        armable    = uint8(CMD(1) > 5); % Armado automatico si Throttle > 5%

        % Formatear la trama estandar para ESP-NOW (sin "\n": writeline ya
        % agrega su propio terminador de linea; ponerlo aqui ademas
        % mandaba una linea vacia extra en cada envio).
        trama = sprintf('T%04dR%04dP%04dY%04dA%d', ...
                        u_throttle, u_roll, u_pitch, u_yaw, armable);

        % Enviar por USB al ESP32 de Tierra
        writeline(puerto, trama);

        % Actualizar interfaz visual
        set(textoUI,'String',sprintf([...
            '=== ESCUDERIA DALTONICS - CONTROL DE TIERRA ===\n\n',...
            '  PORCENTAJES DE TECLADO:\n',...
            '  Throttle: %5.1f %%\n  Roll:     %5.1f %%\n',...
            '  Pitch:    %5.1f %%\n  Yaw:      %5.1f %%\n\n',...
            '  TRAMA SERIAL DISPARADA A ESP-NOW (50 Hz):\n  %s\n\n',...
            '  CONTROLES:\n',...
            '  W/S: Acelerador | Flechas: Roll/Pitch | A/D: Yaw\n',...
            '  ESPACIO: Nivelar | R: Hover (45%%, medido en banco)'], CMD, trama));
    end

    function cerrarPrograma(timerObj, puertoObj, fig)
        stop(timerObj);
        delete(timerObj);
        clear puertoObj;
        delete(fig);
        fprintf('\nPrograma finalizado y puerto COM liberado correctamente.\n');
    end
end
