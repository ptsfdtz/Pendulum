classdef PnGraph < handle
    % Build a named signal graph entirely from standard Simulink blocks.
    properties
        Path; Index=0; Section=''; Groups=struct();
    end
    methods
        function g=PnGraph(path), g.Path=path; end
        function y=block(g,kind,label,args,varargin)
            g.Index=g.Index+1; name=sprintf('%s_%03d',label,g.Index);
            y=[name '/1']; p=[g.Path '/' name];
            col=mod(g.Index-1,8); row=floor((g.Index-1)/8);
            add_block(['simulink/' kind],p,'Position',[100+col*150 60+row*85 165+col*150 95+row*85],varargin{:});
            if ~isempty(g.Section)
                if ~isfield(g.Groups,g.Section), g.Groups.(g.Section)={}; end
                g.Groups.(g.Section){end+1}=p;
            end
            for k=1:numel(args), add_line(g.Path,args{k},[name '/' num2str(k)],'autorouting','on'); end
        end
        function y=c(g,v,label)
            if nargin<3, label='Constant'; end
            y=g.block('Sources/Constant',label,{},'Value',mat2str(v,17));
        end
        function y=in(g,label,port,width)
            y=g.block('Ports & Subsystems/In1',label,{},'Port',num2str(port),'PortDimensions',num2str(width));
        end
        function out(g,label,port,u)
            g.block('Ports & Subsystems/Out1',label,{u},'Port',num2str(port));
        end
        function y=add(g,varargin), y=g.block('Math Operations/Sum','Add',varargin,'Inputs',repmat('+',1,numel(varargin))); end
        function y=sub(g,a,b), y=g.block('Math Operations/Sum','Subtract',{a,b},'Inputs','+-'); end
        function y=mul(g,varargin), y=g.block('Math Operations/Product','Multiply',varargin,'Inputs',num2str(numel(varargin))); end
        function y=div(g,a,b), y=g.block('Math Operations/Product','Divide',{a,b},'Inputs','*/'); end
        function y=gain(g,a,k), y=g.block('Math Operations/Gain','Scale',{a},'Gain',mat2str(k,17)); end
        function y=abs(g,a), y=g.block('Math Operations/Abs','Magnitude',{a}); end
        function y=sign(g,a), y=g.block('Math Operations/Sign','Sign',{a}); end
        function y=trig(g,a,op), y=g.block('Math Operations/Trigonometric Function',op,{a},'Operator',op); end
        function y=wrap(g,a)
            y=g.block('Math Operations/Trigonometric Function','WrapAngle',{g.trig(a,'sin'),g.trig(a,'cos')},'Operator','atan2');
        end
        function y=log(g,a), y=g.block('Math Operations/Math Function','NaturalLog',{a},'Operator','log'); end
        function y=cmp(g,a,op,b), y=g.block('Logic and Bit Operations/Relational Operator','Compare',{a,b},'Operator',op); end
        function y=logic(g,op,varargin)
            y=g.block('Logic and Bit Operations/Logical Operator',op,varargin,'Operator',op,'Inputs',num2str(numel(varargin)));
        end
        function y=choose(g,test,yes,no)
            y=g.block('Signal Routing/Switch','Select',{yes,test,no},'Criteria','u2 ~= 0');
        end
        function y=sat(g,a,lo,hi)
            y=g.block('Discontinuities/Saturation','Clamp',{a},'LowerLimit',mat2str(lo,17),'UpperLimit',mat2str(hi,17));
        end
        function y=minmax(g,op,varargin)
            y=g.block('Math Operations/MinMax',op,varargin,'Function',op,'Inputs',num2str(numel(varargin)));
        end
        function y=delay(g,label,initial)
            y=g.block('Discrete/Unit Delay',label,{},'InitialCondition',mat2str(initial,17),'SampleTime','-1');
        end
        function bind(g,delay,source), add_line(g.Path,source,delay,'autorouting','on'); end
        function y=mux(g,varargin), y=g.block('Signal Routing/Mux','Vector',varargin,'Inputs',num2str(numel(varargin))); end
        function y=double(g,u), y=g.block('Signal Attributes/Data Type Conversion','AsDouble',{u},'OutDataTypeStr','double'); end
        function section(g,name), g.Section=name; end
        function organize(g)
            names=fieldnames(g.Groups);
            for k=1:numel(names)
                handles=cellfun(@(p)get_param(p,'Handle'),g.Groups.(names{k}));
                Simulink.BlockDiagram.createSubsystem(handles,'Name',names{k});
                Simulink.BlockDiagram.arrangeSystem([g.Path '/' names{k}]);
                set_param([g.Path '/' names{k}],'BackgroundColor','lightBlue');
            end
            Simulink.BlockDiagram.arrangeSystem(g.Path);
        end
        function ys=split(g,u,n)
            y=g.block('Signal Routing/Demux','Components',{u},'Outputs',num2str(n));
            ys=arrayfun(@(k) [y(1:end-1) num2str(k)],1:n,'UniformOutput',false);
        end
    end
end
