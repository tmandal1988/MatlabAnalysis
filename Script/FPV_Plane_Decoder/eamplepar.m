f = figure;

prompt = {'Enter matrix size:','Enter colormap name:'};
dlg_title = 'Input';
num_lines = 1;
def = {'20','hsv'};
answer = inputdlg(prompt,dlg_title,num_lines,def,'Callback','uiresume(gcbf)');

% h = uicontrol('Position',[20 20 200 40],'String','Continue',...
%               'Callback',);
disp('This will print immediately');
uiwait(gcf); 
disp('This will print after you click Continue');
close(f);