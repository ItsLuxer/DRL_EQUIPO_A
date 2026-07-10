function y = KE88_lee_mando(~)
global CMD
if isempty(CMD), CMD = [47.4;0;0;0]; end
y = CMD(:);
end
