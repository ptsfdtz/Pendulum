function folder = dp_output_dir()
%DP_OUTPUT_DIR Generated artifacts are kept outside source directories.
folder = fullfile(fileparts(fileparts(mfilename('fullpath'))),'output','double_pendulum');
if ~isfolder(folder), mkdir(folder); end
end
