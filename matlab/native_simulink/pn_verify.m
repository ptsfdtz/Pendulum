function report=pn_verify(rebuild)
% Verify saved models without rebuilding unless explicitly requested.
if nargin<1, rebuild=false; end
if rebuild, pn_build(); end
report=pn_verify_interface();
end
