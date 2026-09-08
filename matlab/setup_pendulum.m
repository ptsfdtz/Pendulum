function root = setup_pendulum()
%SETUP_PENDULUM Add only the offline model sources to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'double_pendulum'));
addpath(fullfile(root,'single_pendulum'));
end
