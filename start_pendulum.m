function result=start_pendulum(order)
%START_PENDULUM Start physical single/double swing-up from MATLAB.
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'matlab'));
if nargin<1
    selection=input('1: single / 2: double / Q: quit > ','s');
    if strcmpi(strtrim(selection),'q'), result=[]; return; end
    order=str2double(selection);
end
result=run_hardware(order);
end
