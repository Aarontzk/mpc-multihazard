function p = get_params()
% GET_PARAMS  Helper for FluidForce block to access sim parameters.
p = evalin('base', 'params');
end
