function build_simulink_model(mlfs_mode)
% BUILD_SIMULINK_MODEL  Programmatically construct the multi-hazard
% closed-loop Simulink model and save it as mpc_multihazard.slx.
%
% Usage:
%   build_simulink_model              % defaults to per_floor
%   build_simulink_model('base_only') % single-MLFS variant
%
% After running, open the generated .slx, configure the disturbance
% scenario via the workspace variable `scenario`, and click Run.
%
% The model wires:
%   Disturbance Generator -> Plant (state-space) -> Scopes
%   Plant state           -> MPC Controller -> Plant input
%   FAHFS reference       -> MPC Controller (tracking)
%
% Plant matrices (Ad, Bd, Ed, Q, R, x_lim, u_lim, ref_traj) are pulled from
% the base workspace (populate by running prep_simulink_workspace.m first).

if nargin < 1; mlfs_mode = 'per_floor'; end

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));

modelName = 'mpc_multihazard';
slxPath = fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']);

% Close + delete any prior version
if bdIsLoaded(modelName); close_system(modelName, 0); end
if exist(slxPath, 'file'); delete(slxPath); end

% Create
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'FixedStepDiscrete', ...
                     'FixedStep', 'Ts', ...
                     'StopTime', 'T_sim');

% ---------------------------------------------------------------------- %
% Block layout (left -> right):
%   1. Clock + scenario selector subsystem
%   2. Disturbance generator (MATLAB Function)
%   3. Reference generator (MATLAB Function)
%   4. MPC controller (MATLAB Function)  -- uses live FISTA QP
%   5. Plant (Discrete State-Space)
%   6. Scopes for displacement, force, elevation
% ---------------------------------------------------------------------- %

% ---- Clock + scenario constant
add_block('simulink/Sources/Clock', [modelName '/Clock'], ...
    'Position', [30 30 60 60]);

% ---- Disturbance Generator (MATLAB Function) ----
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/DisturbanceGen'], 'Position', [120 20 240 80]);
S = sfroot;
b = S.find('Path', [modelName '/DisturbanceGen']);
b.Script = [
    "function [ag, h_now, v_now] = fcn(t)" newline ...
    "  coder.extrinsic('build_disturbance_at');" newline ...
    "  ag = 0; h_now = 0; v_now = 0;" newline ...
    "  [ag, h_now, v_now] = build_disturbance_at(t);" newline ...
    "end"];

% ---- Reference Generator ----
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/ReferenceGen'], 'Position', [280 20 400 80]);
b = S.find('Path', [modelName '/ReferenceGen']);
b.Script = [
    "function r = fcn(t)" newline ...
    "  coder.extrinsic('build_reference_at');" newline ...
    "  r = zeros(8,1);" newline ...
    "  r = build_reference_at(t);" newline ...
    "end"];

% ---- MPC Controller (MATLAB Function calling solve_box_qp) ----
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/MPC_Controller'], 'Position', [440 100 600 200]);
b = S.find('Path', [modelName '/MPC_Controller']);
% Uses persistent warm-start
b.Script = [
    "function u = fcn(x, t, ag, h, v, r)" newline ...
    "  coder.extrinsic('mpc_step');" newline ...
    "  u = zeros(NU,1);" newline ...
    "  u = mpc_step(x, t, ag, h, v, r);" newline ...
    "end"];

% ---- Plant Discrete State-Space ----
add_block('simulink/Discrete/Discrete State-Space', ...
    [modelName '/Plant'], 'Position', [660 80 820 200]);
% Combined input vector: [u; d] where d = [ag; F_f1; F_f2; F_f3]
% Matrices Ad, [Bd Ed] populated from workspace
set_param([modelName '/Plant'], ...
    'A', 'Ad', ...
    'B', '[Bd Ed]', ...
    'C', 'eye(8)', ...
    'D', 'zeros(8, NU+ND)', ...
    'X0', 'zeros(8,1)', ...
    'SampleTime', 'Ts');

% ---- Fluid force computation block (recompute online vs z_base) ----
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/FluidForce'], 'Position', [440 240 600 320]);
b = S.find('Path', [modelName '/FluidForce']);
b.Script = [
    "function Ff = fcn(h, v, x)" newline ...
    "  coder.extrinsic('compute_fluid_force');" newline ...
    "  Ff = zeros(3,1);" newline ...
    "  z_base = x(7);" newline ...
    "  Ff = compute_fluid_force(h, v, z_base, get_params())'';" newline ...
    "end"];

% ---- Mux for plant disturbance input [ag; F_f1..3] ----
add_block('simulink/Signal Routing/Mux', [modelName '/MuxDist'], ...
    'Inputs', '2', 'Position', [620 240 640 280]);

% ---- Mux for plant input [u; d] ----
add_block('simulink/Signal Routing/Mux', [modelName '/MuxIn'], ...
    'Inputs', '2', 'Position', [630 130 650 180]);

% ---- Demux state output ----
add_block('simulink/Signal Routing/Demux', [modelName '/DemuxX'], ...
    'Outputs', '4', 'Position', [840 80 860 200]);
% outputs: q (1:3), q' (4:6), z_base (7), z_base' (8) packaged via Selector

% ---- Scopes ----
add_block('simulink/Sinks/Scope', [modelName '/Scope_q'], ...
    'Position', [900 60 940 100], 'NumInputPorts', '1');
add_block('simulink/Sinks/Scope', [modelName '/Scope_u'], ...
    'Position', [900 140 940 180], 'NumInputPorts', '1');
add_block('simulink/Sinks/Scope', [modelName '/Scope_z'], ...
    'Position', [900 220 940 260], 'NumInputPorts', '1');

% ---- To Workspace logging ----
add_block('simulink/Sinks/To Workspace', [modelName '/Log_x'], ...
    'VariableName', 'x_log', 'SaveFormat', 'Array', ...
    'Position', [900 300 980 340]);
add_block('simulink/Sinks/To Workspace', [modelName '/Log_u'], ...
    'VariableName', 'u_log', 'SaveFormat', 'Array', ...
    'Position', [900 360 980 400]);

% ---------------------------------------------------------------------- %
% Wire it (line connections)
% ---------------------------------------------------------------------- %
add_line(modelName, 'Clock/1',          'DisturbanceGen/1', 'autorouting', 'on');
add_line(modelName, 'Clock/1',          'ReferenceGen/1',   'autorouting', 'on');
add_line(modelName, 'Clock/1',          'MPC_Controller/2', 'autorouting', 'on');

add_line(modelName, 'DisturbanceGen/1', 'MPC_Controller/3', 'autorouting', 'on');  % ag
add_line(modelName, 'DisturbanceGen/2', 'MPC_Controller/4', 'autorouting', 'on');  % h
add_line(modelName, 'DisturbanceGen/3', 'MPC_Controller/5', 'autorouting', 'on');  % v
add_line(modelName, 'ReferenceGen/1',   'MPC_Controller/6', 'autorouting', 'on');

add_line(modelName, 'MPC_Controller/1', 'MuxIn/1',          'autorouting', 'on');
add_line(modelName, 'MuxDist/1',        'MuxIn/2',          'autorouting', 'on');
add_line(modelName, 'MuxIn/1',          'Plant/1',          'autorouting', 'on');

add_line(modelName, 'DisturbanceGen/1', 'MuxDist/1',        'autorouting', 'on');  % ag
add_line(modelName, 'FluidForce/1',     'MuxDist/2',        'autorouting', 'on');  % F_fluid

add_line(modelName, 'DisturbanceGen/2', 'FluidForce/1',     'autorouting', 'on');
add_line(modelName, 'DisturbanceGen/3', 'FluidForce/2',     'autorouting', 'on');
add_line(modelName, 'Plant/1',          'FluidForce/3',     'autorouting', 'on');

add_line(modelName, 'Plant/1',          'MPC_Controller/1', 'autorouting', 'on');
add_line(modelName, 'Plant/1',          'DemuxX/1',         'autorouting', 'on');
add_line(modelName, 'DemuxX/1',         'Scope_q/1',        'autorouting', 'on');
add_line(modelName, 'DemuxX/3',         'Scope_z/1',        'autorouting', 'on');
add_line(modelName, 'MPC_Controller/1', 'Scope_u/1',        'autorouting', 'on');
add_line(modelName, 'Plant/1',          'Log_x/1',          'autorouting', 'on');
add_line(modelName, 'MPC_Controller/1', 'Log_u/1',          'autorouting', 'on');

% Save
save_system(modelName, slxPath);
fprintf('Saved Simulink model: %s\n', slxPath);
fprintf('Run prep_simulink_workspace(''%s'') BEFORE simulating.\n', mlfs_mode);
end
