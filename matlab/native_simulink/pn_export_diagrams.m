function pn_export_diagrams()
here=fileparts(mfilename('fullpath')); folder=fullfile(here,'..','output','native_simulink');
for order=1:2
    model=sprintf('Pendulum_Native_%d',order); load_system(model);
    print(['-s' model],'-dpng','-r140',fullfile(folder,sprintf('model_%d.png',order)));
    print(['-s' model '/Native_Control'],'-dpng','-r100',fullfile(folder,sprintf('controller_%d.png',order)));
end
end
