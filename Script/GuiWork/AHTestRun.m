clear all
close all
clc

[FileName,PathName] = uigetfile('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Flight_Data\FPV_Plane','Select data file');
if FileName==0, return, end

load( fullfile(PathName,FileName) );

% [EKF,localGPS]=PIRATEKFv1(Flight_Data,GPS);
% save(fullfile(PathName,'EKF.mat'),'EKF','localGPS','GPS');

subplot(1,2,1)
exampleGui
subplot(1,2,2)
ArtificialHorizonInit




% 
for i=60000:length(EKF.x)    
    ModifyDisplay(EKF.theta(i)*pi/180,EKF.phi(i)*pi/180,EKF.vx(i),-EKF.z(i));
    pause(0.01)  
end