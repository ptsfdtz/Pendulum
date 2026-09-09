function pn_io_sfun(block)
% MATLAB vendor I/O only; the controller is in Native_Control.
% Outputs acquire inputs; Update applies the already computed native command.
here=fileparts(mfilename('fullpath'));
addpath(here,fullfile(here,'..','hardware'));
block.NumDialogPrms=1; order=block.DialogPrm(1).Data; C=ph_config();
block.NumInputPorts=1; block.NumOutputPorts=3;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).SamplingMode='Sample';
for k=1:3, block.OutputPort(k).SamplingMode='Sample'; end
block.InputPort(1).Dimensions=13; block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=order+1; block.OutputPort(2).Dimensions=2; block.OutputPort(3).Dimensions=1;
block.SampleTimes=[C.dt(order) 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Start',@start); block.RegBlockMethod('Outputs',@read);
block.RegBlockMethod('Update',@write); block.RegBlockMethod('Terminate',@stop);
end
function start(b)
ctx=pn_run_context(b.DialogPrm(1).Data);
ctx.start();
end
function read(b)
ctx=pn_context('get'); [counts,limits,warningLimit]=ctx.read(b.CurrentTime);
b.OutputPort(1).Data=counts; b.OutputPort(2).Data=double(limits); b.OutputPort(3).Data=warningLimit;
end
function write(b)
ctx=pn_context('get'); ctx.write(b.InputPort(1).Data,b.CurrentTime);
end
function stop(~)
ctx=pn_context('get'); if ~isempty(ctx), ctx.finish(); end
end
