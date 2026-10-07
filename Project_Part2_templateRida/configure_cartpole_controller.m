function configure_cartpole_controller()
%CONFIGURE_CARTPOLE_CONTROLLER Configure the Simulink MATLAB Function block
% for iLQR trajectory tracking followed by upright LQR stabilization.
%
% Run this after setup_cartpole has created Ts, K_lqr, Fmax, Knom, Unom,
% Xnom and N in the base workspace.

model = 'cartpole';
blockPath = [model '/MATLAB Function'];

load_system(model);

% Locate the MATLAB Function chart embedded in the Simulink block.
rt = sfroot;
chart = find(rt,'-isa','Stateflow.EMChart','Path',blockPath);
if isempty(chart)
    error('Could not find MATLAB Function block at %s.', blockPath);
end
chart = chart(1);

% First reduce the function signature to the two true signal inputs.  The
% remaining quantities are model parameters read from the base workspace.
chart.Script = sprintf([ ...
    'function u = controller(x,t)\n' ...
    'u = 0.0;\n' ...
    'end\n']);

% Number of optimized control samples (normally 800 for T=8 s, Ts=0.01 s).
N = evalin('base','N');

% Ensure the chart symbols have fixed, explicit dimensions.  This avoids
% MATLAB Function code-generation errors where u is inferred variable-size.
setData(chart,'x','Input','[4 1]');
setData(chart,'t','Input','1');
setData(chart,'u','Output','1');
setData(chart,'Ts','Parameter','1');
setData(chart,'K_lqr','Parameter','[1 4]');
setData(chart,'Fmax','Parameter','1');
setData(chart,'Knom','Parameter',sprintf('[1 4 %d]',N));
setData(chart,'Unom','Parameter',sprintf('[1 %d]',N));
setData(chart,'Xnom','Parameter',sprintf('[4 %d]',N+1));

% Tracking controller.  During the optimized horizon it follows the iLQR
% nominal trajectory.  After the horizon it switches to the upright LQR.
chart.Script = sprintf([ ...
    'function u = controller(x,t)\n' ...
    '%% x = [p; v; theta; omega]\n' ...
    'u = 0.0;\n' ...
    'u_raw = 0.0;\n' ...
    'Nctrl = size(Unom,2);\n' ...
    'k = floor(t/Ts) + 1;\n' ...
    'if k < 1\n' ...
    '    k = 1;\n' ...
    'end\n' ...
    'if k <= Nctrl\n' ...
    '    x_nom = Xnom(:,k);\n' ...
    '    u_nom = Unom(1,k);\n' ...
    '    K_nom = reshape(Knom(:,:,k),1,4);\n' ...
    '    e = x - x_nom;\n' ...
    '    e(3) = atan2(sin(e(3)),cos(e(3)));\n' ...
    '    u_raw = u_nom - K_nom*e;\n' ...
    'else\n' ...
    '    theta = atan2(sin(x(3)),cos(x(3)));\n' ...
    '    x_lqr = [x(1); x(2); theta; x(4)];\n' ...
    '    u_raw = -K_lqr*x_lqr;\n' ...
    'end\n' ...
    'u = min(max(u_raw,-Fmax),Fmax);\n' ...
    'end\n']);

% Give the upright LQR a few seconds after the 8 s swing-up trajectory.
set_param(model,'StopTime','12');

save_system(model);

fprintf('Configured %s for iLQR tracking + upright LQR.\n', blockPath);
end

function setData(chart,name,scope,sizeString)
% Create or update one MATLAB Function chart data symbol.
d = chart.find('-isa','Stateflow.Data','Name',name);
if isempty(d)
    d = Stateflow.Data(chart);
    d.Name = name;
else
    d = d(1);
end

d.Scope = scope;
d.Props.Array.Size = sizeString;
end
