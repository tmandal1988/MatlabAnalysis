clear all
close all
clc

if(~isempty(instrfind))
    fclose(instrfind);
end

serialObject=serial('COM25','Baudrate',115200);

serialObject.InputBufferSize=4096;
serialObject.BytesAvailableFcnCount = 50;
serialObject.BytesAvailableFcnMode = 'byte';
serialObject.BytesAvailableFcn = {@readData};
fopen(serialObject);

% while(1)
%     
%     pause(0.1);
% end


