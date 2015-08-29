clear all
close all
clc

EKFstates=zeros(9,1);
cdeg2rad = 0.0175;
g=+9.8;

% Q=[0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0;0,0,0,2.3453e-5,0,0,0,0,0;0,0,0,0,3.2589e-5,0,0,0,0;0,0,0,0,0,2.8023e-4,0,0,0;0,0,0,0,0,0,3.6204e-7,0,0;0,0,0,0,0,0,0,3.6204e-7,0;0,0,0,0,0,0,0,0,6.3775e-7];
% P=Q;
% 
%  R=diag([0.0738,0.0036,0.0638,0.0017,0.0011,0.0078]);
%     R=eye(6);

% ALPHA=2;
% R=diag([ALPHA*.1023,ALPHA*.1060,ALPHA*.1120,1,1,1]);

Q=diag([0,0,0,7.88e-4,8.937e-4,8.87e-4,3.047e-6,4.059e-6,4.675e-6]);
% ALPHA=2;
% R=diag([ALPHA*.1023,ALPHA*.5060,ALPHA*.5120,ALPHA*3.89e-4,ALPHA*1.113e-4,ALPHA*9.84e-4]);

ALPHA=2;
R=diag([ALPHA*.1023,ALPHA*.5060,ALPHA*.5120,ALPHA*3.89e-4,ALPHA*1.113e-4,ALPHA*9.84e-4]);
% ALPHA=2;
% R=diag([ALPHA*0.1023,ALPHA*0.5060,ALPHA*0.5120,ALPHA*3.89e-4,ALPHA*1.113e-4,ALPHA*9.84e-4]);
% ALPHA=2;
% R=diag([ALPHA*0.3074,ALPHA*0.1414,ALPHA*1.7186,ALPHA*7.5638e-4,ALPHA*5.8469e-4,0.0023]);


P=eye(9);
% 
% Rot=[0.104399541484715,-0.621882832203930,-0.776120144531631,-4.6566e-09;
%     0.986199751670064,0.165559807338325,2.775557561562891e-17,2.328306436538696e-09;
%     0.128494301600050,-0.765409493803229,0.630585062661811,-6.369913497063834e+06;
%     0,0,0,1];
%Rot=[0.104399541484715,-0.621882832203930,-0.776120144531631;0.986199751670064,0.165559807338325,0;0.128494301600050,-0.765409493803229,0.630585062661811];


load('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Flight_Data\Organized_Flight_Data_2012\Flight_2012_10_13\Green_2\PC104\data.dat');
load('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Flight_Data\Organized_Flight_Data_2012\Flight_2012_10_13\Green_2\PC104\bestxyz.dat');
load('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Flight_Data\Organized_Flight_Data_2012\Flight_2012_10_13\Green_2\PC104\EKF9.mat');


Flight_Data.Ax=data(:,1)*(3.33e-3)*g;
Flight_Data.Ay=-data(:,2)*(3.33e-3)*g;
Flight_Data.Az=data(:,3)*(3.33e-3)*g;

Flight_Data.P=-data(:,4)*0.025;
Flight_Data.Q=data(:,5)*0.025;
Flight_Data.R=-data(:,6)*0.025;

biasQ=mean(Flight_Data.Q(1:5000))*cdeg2rad;
biasP=mean(Flight_Data.P(1:5000))*cdeg2rad;
biasR=mean(Flight_Data.R(1:5000))*cdeg2rad;

GPS.x=bestxyz(:,7);
GPS.y=bestxyz(:,8);
GPS.z=bestxyz(:,9);

GPS.vx=bestxyz(:,13);
GPS.vy=bestxyz(:,14);
GPS.vz=bestxyz(:,15);

dataSize=length(GPS.x);



localGPS(:,1)=GPS.x;
localGPS(:,2)=-GPS.y;
localGPS(:,3)=-GPS.z;

localGPS(:,4)=GPS.vx;
localGPS(:,5)=-GPS.vy;
localGPS(:,6)=-GPS.vz;

H=[1 0 0 0 0 0 0 0 0;0 1 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0;0 0 0 0 1 0 0 0 0;0 0 0 0 0 1 0 0 0];

for i=2:1:dataSize
    
    if(abs(localGPS(i,1)-localGPS(i-1,1))>50)
        localGPS(i,1)=localGPS(i-1);
    end
    
    if(abs(localGPS(i,2)-localGPS(i-1,2))>50)
        localGPS(i,2)=localGPS(i-1,2);
    end
    
    if(abs(localGPS(i,3)-localGPS(i-1,3))>50)
         localGPS(i,3)=localGPS(i-1,3);
    end
    
    if(abs(localGPS(i,4)-localGPS(i-1,4))>20)
         localGPS(i,4)=localGPS(i-1,4);
    end
    
    if(abs(localGPS(i,5)-localGPS(i-1,5))>20)
         localGPS(i,5)=localGPS(i-1,5);
    end
    
    if(abs(localGPS(i,6)-localGPS(i-1,6))>20)
        localGPS(i,6)=localGPS(i-1,6);
    end
    
    if(abs(Flight_Data.Ax(i)-Flight_Data.Ax(i-1))>50)
        Flight_Data.Ax(i)=Flight_Data.Ax(i-1);
    end

    if(abs(Flight_Data.Ay(i)-Flight_Data.Ay(i-1))>50)
        Flight_Data.Ay(i)=Flight_Data.Ay(i-1);
    end
    
    if(abs(Flight_Data.Az(i)-Flight_Data.Az(i-1))>50)
        Flight_Data.Az(i)=Flight_Data.Az(i-1);
    end
    
    if(abs(Flight_Data.P(i)-Flight_Data.P(i-1))>150)
        Flight_Data.P(i)=Flight_Data.P(i-1);
    end
    
    if(abs(Flight_Data.Q(i)-Flight_Data.Q(i-1))>150)
        Flight_Data.Q(i)=Flight_Data.Q(i-1);
    end
    
    if(abs(Flight_Data.R(i)-Flight_Data.R(i-1))>150)
        Flight_Data.R(i)=Flight_Data.R(i-1);
    end
end


% dt=Flight_Data.Delta_Time;
dt=zeros(dataSize,1)+0.02;


EKF.phi=zeros(dataSize,1);
EKF.theta=zeros(dataSize,1);
EKF.psi=zeros(dataSize,1);

EKF.x=zeros(dataSize,1);
EKF.y=zeros(dataSize,1);
EKF.z=zeros(dataSize,1);

EKF.vx=zeros(dataSize,1);
EKF.vy=zeros(dataSize,1);
EKF.vz=zeros(dataSize,1);

EKFstates(1)=localGPS(2,1);
EKFstates(2)=localGPS(2,2);
EKFstates(3)=localGPS(2,3);

EKFstates(4)=localGPS(2,4);
EKFstates(5)=localGPS(2,5);
EKFstates(6)=localGPS(2,6);

Flight_Data.P=-data(:,4)*0.025*cdeg2rad;
Flight_Data.Q=data(:,5)*0.025*cdeg2rad;
Flight_Data.R=-data(:,6)*0.025*cdeg2rad;

gpsFlag=bestxyz(:,24);
imuFlag=data(:,27);

intgPitch=zeros(dataSize,1);
pitchRes=zeros(dataSize,1);
Res=zeros(9,1);

roll=zeros(dataSize,1);
pitch=zeros(dataSize,1);
yaw=zeros(dataSize,1);


for i=2:1:dataSize
    
  
    s_phi=sin(EKFstates(7));
	c_phi=cos(EKFstates(7));

	s_theta=sin(EKFstates(8));
	c_theta=cos(EKFstates(8));
	sec_theta=sec(EKFstates(8));
    t_theta=tan(EKFstates(8));

	s_psi=sin(EKFstates(9));
	c_psi=cos(EKFstates(9));
    
    s_phiint=sin(roll(i-1));
    c_phiint=cos(roll(i-1));
    
    s_thetaint=sin(pitch(i-1));
	c_thetaint=cos(pitch(i-1));
	sec_thetaint=sec(pitch(i-1));
    t_thetaint=tan(pitch(i-1));
    
    s_psiint=sin(yaw(i-1));
	c_psiint=cos(yaw(i-1));

    
     if(imuFlag(i)>=1 && imuFlag(i)<10)
        
    EKFstates(1)=EKFstates(1)+dt(i)*EKFstates(4);
    EKFstates(2)=EKFstates(2)+dt(i)*EKFstates(5);
    EKFstates(3)=EKFstates(3)+dt(i)*EKFstates(6);
        
    EKFstates(4)=EKFstates(4) + dt(i)*(Flight_Data.Ax(i)*c_theta*c_psi + Flight_Data.Ay(i)*(-c_phi*s_psi + s_phi*s_theta*c_psi) + Flight_Data.Az(i)*(s_phi*s_psi + c_phi*s_theta*c_psi));
	EKFstates(5)=EKFstates(5) + dt(i)*(Flight_Data.Ax(i)*c_theta*s_psi + Flight_Data.Ay(i)*(c_phi*c_psi + s_phi*s_theta*s_psi) + Flight_Data.Az(i)*(-s_phi*c_psi + c_phi*s_theta*s_psi));
	EKFstates(6)=EKFstates(6) + dt(i)*(-Flight_Data.Ax(i)*s_theta + (Flight_Data.Ay(i)*s_phi + Flight_Data.Az(i)*c_phi)*c_theta)+g*dt(i);
    
    EKFstates(7)=EKFstates(7) + dt(i)*(Flight_Data.P(i)+(Flight_Data.Q(i)*s_phi + Flight_Data.R(i)*c_phi)*t_theta);
    EKFstates(8)=EKFstates(8) + dt(i)*(Flight_Data.Q(i)*c_phi - Flight_Data.R(i)*s_phi);
	EKFstates(9)=EKFstates(9) + dt(i)*(Flight_Data.Q(i)*s_phi + Flight_Data.R(i)*c_phi)*sec_theta;
    

    
    
    
    A=[1 0 0 dt(i) 0 0 0 0 0;
        0 1 0 0 dt(i) 0 0 0 0;
        0 0 1 0 0 dt(i) 0 0 0;
        0 0 0 1 0 0 dt(i)*(Flight_Data.Ay(i)*(s_phi*s_psi + c_phi*s_theta*c_psi) + Flight_Data.Az(i)*(c_phi*s_psi - s_phi*s_theta*c_psi)) dt(i)*(Flight_Data.Ax(i)*(-s_theta*c_psi) + Flight_Data.Ay(i)*(c_theta*c_psi*s_phi) + Flight_Data.Az(i)*(c_phi*c_theta*c_psi)) dt(i)*(Flight_Data.Ax(i)*(-c_theta*s_psi) - Flight_Data.Ay(i)*(c_phi*c_psi+s_phi*s_theta*s_psi)+ Flight_Data.Az(i)*(s_phi*c_psi-c_phi*s_theta*s_psi));
        0 0 0 0 1 0 dt(i)*(Flight_Data.Ay(i)*(-s_phi*c_psi + c_phi*s_theta*s_psi) - Flight_Data.Az(i)*(c_phi*c_psi + s_phi*s_theta*s_psi)) dt(i)*(Flight_Data.Ax(i)*(-s_theta*s_psi) + Flight_Data.Ay(i)*(c_theta*s_phi*s_psi) + Flight_Data.Az(i)*(c_phi*c_theta*s_psi)) dt(i)*(Flight_Data.Ax(i)*(c_theta*c_psi) + Flight_Data.Ay(i)*(-c_phi*s_psi + s_phi*s_theta*c_psi) + Flight_Data.Az(i)*(s_phi*s_psi + c_phi*s_theta*c_psi));
        0 0 0 0 0 1 dt(i)*(Flight_Data.Ay(i)*(c_phi*c_theta) - Flight_Data.Az(i)*(s_phi*c_theta))                                          -dt(i)*(Flight_Data.Ax(i)*(c_theta) + Flight_Data.Ay(i)*(s_theta*s_phi) + Flight_Data.Az(i)*(c_phi*s_theta))                                                                                         0                                                                                        ;
        0 0 0 0 0 0 1+dt(i)*t_theta*(c_phi*Flight_Data.Q(i)- s_phi*Flight_Data.R(i))                                        dt(i)*sec_theta*sec_theta*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                                                                                                         0                                                                                        ;
        0 0 0 0 0 0 -dt(i)*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                 1                                                                                                                                                                                                                  0                                                                                        ;
        0 0 0 0 0 0 dt(i)*sec_theta*(c_phi*Flight_Data.Q(i) - s_phi*Flight_Data.R(i))                                         dt(i)*sec_theta*t_theta*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                                                                                                         1                                                                                        ];                            
    
    
    
    P=A*P*A' + Q;
      end
    
     intgPitch(i)=EKFstates(8)*180/pi;
    
    if(gpsFlag(i)>=1 && gpsFlag(i)<10)
        
    K=P*H'/(H*P*H'+R);
    Res=K*([localGPS(i,1);localGPS(i,2);localGPS(i,3);localGPS(i,4);localGPS(i,5);localGPS(i,6)]-EKFstates(1:6));
    EKFstates=EKFstates + Res;  
    P=(eye(9,9)-K*H)*P; 
    end     
  

    if(EKFstates(7)>2*pi)
        EKFstates(7)=EKFstates(7)-2*pi;
    elseif(EKFstates(7)<-2*pi)
        EKFstates(7)=EKFstates(7)+2*pi;
    end
    
    if(EKFstates(8)>pi/2)
        EKFstates(8)=EKFstates(8)-pi/2;
    elseif(EKFstates(8)<-pi/2)
        EKFstates(8)=EKFstates(8)+pi/2;
    end
    
    if(EKFstates(9)>2*pi)
        EKFstates(9)=EKFstates(9)-2*pi;
    elseif(EKFstates(9)<0)
        EKFstates(9)=EKFstates(9)+2*pi;
    end
    
    EKF.phi(i)=EKFstates(7)*180/pi;
    EKF.theta(i)=EKFstates(8)*180/pi;
    EKF.psi(i)=EKFstates(9)*180/pi;
    
    EKF.x(i)=EKFstates(1);
    EKF.y(i)=EKFstates(2);
    EKF.z(i)=EKFstates(3);
    
    EKF.vx(i)=EKFstates(4);
    EKF.vy(i)=EKFstates(5);
    EKF.vz(i)=EKFstates(6);   
    
   
    pitchRes(i)= Res(8,1);
    
    roll(i)=roll(i-1) + dt(i)*((Flight_Data.P(i)-biasP)+((Flight_Data.Q(i)-biasQ)*s_phiint + (Flight_Data.R(i)-biasQ)*c_phiint)*t_thetaint);
    pitch(i)=pitch(i-1) + dt(i)*((Flight_Data.Q(i)-biasQ)*c_phiint - (Flight_Data.R(i)-biasR)*s_phiint);
	yaw(i)=yaw(i-1) + dt(i)*((Flight_Data.Q(i)-biasQ)*s_phiint + (Flight_Data.R(i)-biasR)*c_phiint)*sec_thetaint;
    
end
% 
% % figure
% % plot(EKF.phi,'LineWidth',2)
% % ylim([-90 90])
% % figure
plot(EKF.theta,'r','LineWidth',2)
ylim([-90 90])
hold on
plot(EKF9.Pitch(2:90001),'LineWidth',2)
grid on
hold on
% %plot(Flight_Data.Elevator*0.05-45,'b','LineWidth',3)
% % figure
% % plot(EKF.psi,'k','LineWidth',2)
% % ylim([-180 180])
% 
% % clear all
% % close all
% % clc
% % 
% % EKFstates=zeros(6,1);
% % P=eye(6);
% % Q=[2.465100000000000e-07 0 0 0 0 0;0 3.437200000000000e-07 0 0 0 0;0 0 2.965500000000000e-06 0 0 0;0 0 0 3.337800000000000e-07 0 0;0 0 0 0 3.614900000000000e-07 0;0 0 0 0 0 6.377500000000000e-07];
% % 
% % R=[3.389e-6 0 0;0 1.113e-6 0;0 0 9.84e-6];
% % % R=[0.001 0 0;0 0.001 0;0 0 0.001];
% % localRot=[0.104399541484715,-0.621882832203930,-0.776120144531631,-2.101142787965760e+04;0.986199751670064,0.165559807338325,0,-83.047219284344460;0.128494301600050,-0.765409493803229,0.630585062661811,-6.369913497063834e+06;0,0,0,1];
% % H=[0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
% % 
% % [FileName,PathName] = uigetfile('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Script\Gyro_Data_Collection','Select Mat file');
% % if FileName==0, return, end
% % 
% % load(fullfile(PathName,FileName));
% % 
% % accelData.x=Flight_Data.Ax;
% % accelData.y=Flight_Data.Ay;
% % accelData.z=Flight_Data.Az;
% % 
% % gyroData.y=Flight_Data.Q;
% % gyroData.x=Flight_Data.P;
% % gyroData.z=Flight_Data.R;
% % 
% % dataSize=length(accelData.x);
% % localGPS=zeros(dataSize,3);
% % A=zeros(6,6);
% % 
% % %convert GPS velocity to local velocity
% % for i=1:1:dataSize;
% %     localGPS(i,:)=[1 0 0 0;0 -1 0 0;0 0 -1 0]*localRot*[GPS.vx(i);GPS.vy(i);GPS.vz(i);0];
% % end
% % 
% % 
% % %starting EKF
% % dt=0.01;
% % cdeg2rad = 0.017;
% % g=9.8;
% % phi=zeros(dataSize,1);
% % theta=zeros(dataSize,1);
% % psi=zeros(dataSize,1);
% % 
% % for i=1:1:dataSize
% %     
% %     s_phi=sin(EKFstates(1));
% % 	c_phi=cos(EKFstates(1));
% % 
% % 	s_theta=sin(EKFstates(2));
% % 	c_theta=cos(EKFstates(2));
% % 	sec_theta=1/c_theta;
% %     t_theta=s_theta/c_theta;
% % 
% % 	s_psi=sin(EKFstates(3));
% % 	c_psi=cos(EKFstates(3));
% %             
% %     EKFstates(1)=EKFstates(1) + dt*(gyroData.x(i)*cdeg2rad+(gyroData.y(i)*cdeg2rad*s_phi + gyroData.z(i)*cdeg2rad*c_phi)*t_theta);
% %     EKFstates(2)=EKFstates(2) + dt*(gyroData.y(i)*cdeg2rad*c_phi - gyroData.z(i)*cdeg2rad*s_phi);
% % 	EKFstates(3)=EKFstates(3) + dt*(gyroData.y(i)*cdeg2rad*s_phi + gyroData.z(i)*cdeg2rad*c_phi)*sec_theta;
% % 
% % 	EKFstates(4)=EKFstates(4) + dt*(g*accelData.x(i)*c_theta*c_psi + g*accelData.y(i)*(-c_phi*s_psi + s_phi*s_theta*c_psi) + g*accelData.z(i)*(s_phi*s_psi + c_phi*s_theta*c_psi));
% % 	EKFstates(5)=EKFstates(5) + dt*(g*accelData.x(i)*c_theta*s_psi + g*accelData.y(i)*(c_phi*c_psi + s_phi*s_theta*s_psi) + g*accelData.z(i)*(-s_phi*c_psi + c_phi*s_theta*s_psi));
% % 	EKFstates(6)=EKFstates(6) + dt*(-g*accelData.x(i)*s_theta + (g*accelData.y(i)*s_phi + g*accelData.z(i)*c_phi)*c_theta)+g*dt;
% %     
% %     A(1,1)=1+dt*t_theta*(c_phi*gyroData.y(i)*cdeg2rad - s_phi*gyroData.z(i)*cdeg2rad);
% %     A(1,2)=dt*sec_theta*sec_theta*(s_phi*gyroData.y(i)*cdeg2rad + c_phi*gyroData.z(i)*cdeg2rad);
% %     
% %     A(2,1)=-dt*(s_phi*gyroData.y(i)*cdeg2rad + c_phi*gyroData.z(i)*cdeg2rad);
% %     A(2,2)=1;
% %     
% %     A(3,1)=dt*sec_theta*(c_phi*gyroData.y(i)*cdeg2rad - s_phi*gyroData.z(i)*cdeg2rad);
% %     A(3,2)=dt*sec_theta*t_theta*(s_phi*gyroData.y(i)*cdeg2rad + c_phi*gyroData.z(i)*cdeg2rad);
% %     A(3,3)=1;
% %     
% %     A(4,1)=dt*(g*accelData.y(i)*(s_phi*s_psi + c_phi*s_theta*c_psi) + g*accelData.z(i)*(c_phi*s_psi - s_phi*s_theta*c_psi));
% %     A(4,2)=dt*(g*accelData.x(i)*(-s_theta*c_psi) + g*accelData.y(i)*(c_theta*c_psi*s_phi) + g*accelData.z(i)*(c_phi*c_theta*c_psi));
% %     A(4,3)=dt*(g*accelData.x(i)*(-c_theta*s_psi)-g*accelData.y(i)*(c_phi*c_psi+s_phi*s_theta*s_psi)+g*accelData.z(i)*(s_phi*s_psi-s_phi*s_theta*s_psi));
% %     A(4,4)=1;
% %     
% %     A(5,1)=dt*(g*accelData.y(i)*(-s_phi*c_psi + c_phi*s_theta*s_psi) - g*accelData.z(i)*(c_phi*c_psi - s_phi*s_theta*s_psi));
% %     A(5,2)=dt*(g*accelData.x(i)*(-s_theta*s_psi) + g*accelData.y(i)*(c_theta*s_phi*s_psi) + g*accelData.z(i)*(c_phi*c_theta*s_psi));
% %     A(5,3)=dt*(g*accelData.x(i)*(c_theta*c_psi) + g*accelData.y(i)*(-c_phi*s_psi + s_phi*s_theta*c_psi) + g*accelData.z(i)*(s_phi*s_psi + c_phi*s_theta*c_psi));
% %     A(5,5)=1;
% %     
% %     A(6,1)=dt*(g*accelData.y(i)*(c_phi*c_theta) - g*accelData.z(i)*(s_phi*c_theta));
% %     A(6,2)=-dt*(g*accelData.x(i)*(c_theta) + g*accelData.y(i)*(s_theta*s_phi) + g*accelData.z(i)*(c_phi*s_theta));
% %     A(6,6)=1;
% %     
% %     P=A*P*A'+Q;
% %     
% %     K=P*H'/(H*P*H'+R);
% %     
% %     EKFstates=EKFstates + K*([localGPS(i,1);localGPS(i,2);localGPS(i,3)]-H*EKFstates);
% %     
% %     if(EKFstates(1)>pi/2)
% %         EKFstates(1)=EKFstates(1)-pi/2;
% %     elseif(EKFstates(1)<-pi/2)
% %         EKFstates(1)=EKFstates(1)+pi/2;
% %     end
% %     
% %     if(EKFstates(2)>pi/2)
% %         EKFstates(2)=EKFstates(2)-pi/2;
% %     elseif(EKFstates(2)<-pi/2)
% %         EKFstates(2)=EKFstates(2)+pi/2;
% %     end
% %     
% %     if(EKFstates(3)>2*pi)
% %         EKFstates(3)=EKFstates(3)-2*pi;
% %     elseif(EKFstates(3)<-2*pi)
% %         EKFstates(3)=EKFstates(3)+2*pi;
% %     end
% %     
% %     phi(i)=EKFstates(1);
% %     theta(i)=EKFstates(2);
% %     psi(i)=EKFstates(3);
% %     P=(eye(6)-K*H)*P;   
% %     
% %     
% %     
% % end
