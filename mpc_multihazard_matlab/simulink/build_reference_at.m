function r = build_reference_at(t)
% BUILD_REFERENCE_AT  Sample reference vector at time t.
persistent ref_cached t_cached
if isempty(ref_cached)
    ref_cached = evalin('base', 'ref_data');
    t_cached   = evalin('base', 'dist_data').t;
end
r = zeros(8, 1);
for i = 1:8
    r(i) = interp1(t_cached, ref_cached(i, :), t, 'linear', 0);
end
end
