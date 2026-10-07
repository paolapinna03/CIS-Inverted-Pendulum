function configure_cartpole_controller()
%CONFIGURE_CARTPOLE_CONTROLLER Configure the Simulink controller for
% iLQR trajectory tracking followed by upright LQR stabilization.
%
% The optimized quantities are passed to the MATLAB Function block through
% Constant blocks. The feedback law is written using scalar indexing so the
% MATLAB Function coder can prove that the output u is always scalar.

model = 'cartpole';
blockName = 'MATLAB Function';
blockPath = [model '/' blockName];

load_system(model);

% Find the MATLAB Function chart.
rt = sfroot;
chart = find(rt,'-isa','Stateflow.EMChart','Path',blockPath);
if isempty(chart)
    error('Could not find MATLAB Function block at %s.', blockPath);
end
chart = chart(1);

% Remove old parameter declarations created by previous attempts. These
% variables will be ordinary input ports instead.
extraNames = {'Ts','K_lqr','Fmax','Knom','Knom_sim','Unom','Xnom'};
for i = 1:numel(extraNames)
    d = chart.find('-isa','Stateflow.Data','Name',extraNames{i});
    for j = 1:numel(d)
        delete(d(j));
    end
end

% IMPORTANT: keep every intermediate entering u scalar. Avoid dynamic
% column extraction and matrix products, which can make Simulink infer a
% variable-size output even when mathematically the result is scalar.
chart.Script = sprintf([ ...
    'function u = controller(x,t,Ts,K_lqr,Fmax,Knom_sim,Unom,Xnom)\n' ...
    '%% x = [p; v; theta; omega]\n' ...
    'u = 0.0;\n' ...
    'u_raw = 0.0;\n' ...
    'Nctrl = size(Unom,2);\n' ...
    'k = floor(t/Ts) + 1;\n' ...
    'if k < 1\n' ...
    '    k = 1;\n' ...
    'end\n' ...
    'if k <= Nctrl\n' ...
    '    e1 = x(1) - Xnom(1,k);\n' ...
    '    e2 = x(2) - Xnom(2,k);\n' ...
    '    e3 = x(3) - Xnom(3,k);\n' ...
    '    e4 = x(4) - Xnom(4,k);\n' ...
    '    e3 = atan2(sin(e3),cos(e3));\n' ...
    '    feedback = Knom_sim(1,k)*e1 + Knom_sim(2,k)*e2 + ...\n' ...
    '               Knom_sim(3,k)*e3 + Knom_sim(4,k)*e4;\n' ...
    '    u_raw = Unom(1,k) - feedback;\n' ...
    'else\n' ...
    '    theta = atan2(sin(x(3)),cos(x(3)));\n' ...
    '    feedback = K_lqr(1)*x(1) + K_lqr(2)*x(2) + ...\n' ...
    '               K_lqr(3)*theta + K_lqr(4)*x(4);\n' ...
    '    u_raw = -feedback;\n' ...
    'end\n' ...
    'if u_raw > Fmax\n' ...
    '    u = Fmax;\n' ...
    'elseif u_raw < -Fmax\n' ...
    '    u = -Fmax;\n' ...
    'else\n' ...
    '    u = u_raw;\n' ...
    'end\n' ...
    'end\n']);

% Explicitly declare the output as scalar.
uData = chart.find('-isa','Stateflow.Data','Name','u');
if ~isempty(uData)
    uData(1).Props.Array.Size = '1';
end

% Refresh the block ports.
set_param(model,'SimulationCommand','update');

% Add/update Constant blocks for controller data. Their values are variable
% names populated in the base workspace by setup_cartpole.m.
values = {'Ts','K_lqr','Fmax','Knom_sim','Unom','Xnom'};
positions = [ ...
     30 500 100 530; ...
     30 545 100 575; ...
     30 590 100 620; ...
     30 635 100 665; ...
     30 680 100 710; ...
     30 725 100 755];

for i = 1:numel(values)
    srcName = ['cfg_' values{i}];
    srcPath = [model '/' srcName];

    if getSimulinkBlockHandle(srcPath) == -1
        add_block('simulink/Sources/Constant',srcPath, ...
            'Value',values{i}, ...
            'Position',positions(i,:));
    else
        set_param(srcPath,'Value',values{i});
    end
end

% Remove obsolete cfg_Knom from an earlier version.
oldPath = [model '/cfg_Knom'];
if getSimulinkBlockHandle(oldPath) ~= -1
    oldPH = get_param(oldPath,'PortHandles');
    oldLine = get_param(oldPH.Outport,'Line');
    if oldLine ~= -1
        delete_line(oldLine);
    end
    delete_block(oldPath);
end

% Connect ports 3..8. Ports 1 and 2 are the existing state x and time t.
set_param(model,'SimulationCommand','update');
ph = get_param(blockPath,'PortHandles');

if numel(ph.Inport) < 8
    error('Controller has %d input ports, expected 8.',numel(ph.Inport));
end

for port = 3:8
    lineHandle = get_param(ph.Inport(port),'Line');
    if lineHandle ~= -1
        delete_line(lineHandle);
    end

    srcName = ['cfg_' values{port-2}];
    add_line(model,[srcName '/1'],[blockName '/' num2str(port)], ...
        'autorouting','on');
end

set_param(model,'StopTime','12');
save_system(model,[],'OverwriteIfChangedOnDisk',true);

fprintf('Configured %s with scalar iLQR/LQR control law.\n',blockPath);
end
