function KE88_mando
global CMD
if isempty(CMD), CMD = [47.4;0;0;0]; end
f = figure(100); clf(f);
set(f,'Name','MANDO KE88','Color','w','NumberTitle','off');
txt = uicontrol(f,'Style','text','Units','normalized',...
    'Position',[0.05 0.05 0.9 0.9],'FontSize',13,'FontName','FixedWidth',...
    'HorizontalAlignment','left','BackgroundColor','w');
set(f,'KeyPressFcn',@tecla,'KeyReleaseFcn',@suelta);
refresca();
    function tecla(~,e)
        switch e.Key
            case 'w',          CMD(1) = min(100, CMD(1)+1);
            case 's',          CMD(1) = max(0,   CMD(1)-1);
            case 'uparrow',    CMD(3) =  12;
            case 'downarrow',  CMD(3) = -12;
            case 'rightarrow', CMD(2) =  12;
            case 'leftarrow',  CMD(2) = -12;
            case 'a',          CMD(4) =  60;
            case 'd',          CMD(4) = -60;
            case 'space',      CMD(2:4) = 0;
            case 'r',          CMD(1) = 47.4;
        end
        refresca();
    end
    function suelta(~,e)
        switch e.Key
            case {'uparrow','downarrow'},    CMD(3) = 0;
            case {'leftarrow','rightarrow'}, CMD(2) = 0;
            case {'a','d'},                  CMD(4) = 0;
        end
        refresca();
    end
    function refresca()
        set(txt,'String',sprintf(['   MANDO KE88 (deja el foco aqui)\n\n',...
            '   Throttle: %5.1f %%\n   Roll:     %5.1f deg\n',...
            '   Pitch:    %5.1f deg\n   Yaw:      %5.1f dps\n\n',...
            '   W/S gas   R hover   flechas mover\n',...
            '   A/D girar   ESPACIO nivelar'], CMD));
    end
end
