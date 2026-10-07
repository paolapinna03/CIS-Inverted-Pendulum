function configure_cartpole_controller()
%CONFIGURE_CARTPOLE_CONTROLLER Configure the Simulink controller for
% iLQR trajectory tracking followed by upright LQR stabilization.
%
% The optimized quantities are passed to the MATLAB Function block through
% Constant blocks. This avoids MATLAB Function parameter-scope issues and
% makes all controller dependencies explicit in the Simulink diagram.

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
% variables will become ordinary input ports instead.
extraNames = {'Ts','K_lqr','Fmax','Knom','Unom','Xnom'};
for i = 1:numel(extraNames)
    d = chart.find('-isa','Stateflow.Data','Name',extraNames{i});
    for j = 1:numel(d)
        delete(d(j));
    end
end

% Use explicit inputs. u is initialized as a scalar so code generation
% cannot infer a variable-size output.
chart.Script = sprintf([ ...
    'function u = controller(x,t,Ts,K_lqr,Fmax,Knom,Unom,Xnom)\n' ...
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

% Refresh the block ports.
set_param(model,'SimulationCommand','update');

% Add/update Constant blocks for controller data. Their values are variable
% names populated in the base workspace by setup_cartpole.m.
values = {'Ts','K_lqr','Fmax','Knom','Unom','Xnom'};
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

% Refresh after adding blocks, then connect ports 3..8. Ports 1 and 2 are
% the existing state vector x and simulation time t and are left untouched.
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

% Leave enough time to observe the upright LQR after the 8 s swing-up.
set_param(model,'StopTime','12');

save_system(model,[],'OverwriteIfChangedOnDisk',true);

fprintf('Configured %s with explicit iLQR/LQR inputs.\n',blockPath);
end
