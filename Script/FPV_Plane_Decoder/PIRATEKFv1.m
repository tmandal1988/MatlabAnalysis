function [EKFfunc,funclocalGPS]=PIRATEKFv1(Flight_Data,GPS);

cdeg2rad=0.0175;
g=9.8;%m/s^s



Flight_Data.Ax=Flight_Data.Ax*g;
Flight_Data.Ay=Flight_Data.Ay*g;
Flight_Data.Az=Flight_Data.Az*g;

Flight_Data.P=Flight_Data.P*cdeg2rad;
Flight_Data.Q=Flight_Data.Q*cdeg2rad;
Flight_Data.R=Flight_Data.R*cdeg2rad;

%  Rot=[0.104399541484715,-0.621882832203930,-0.776120144531631,-2.1055e4;
%      0.986199751670064,0.165559807338325,2.775557561562891e-17,2.328306436538696e-09;
%     0.128494301600050,-0.765409493803229,0.630585062661811,-6.369913497063834e+06;
%     0,0,0,1];

atJacksonMill=1;

if atJacksonMill==1
    Rot=[0.1040 -0.6194 -0.7782 10.5057;
        0.9862 0.1656 0 4.190951585769653e-09;
        0.1289 -0.7675 0.6280 -6.3699e+06;
        0 0 0 1];
else
    Rot=[0.1107 -0.6258 -0.7721 124.9459;
    0.9847 0.1741 0 3.0268e-09;
    0.1344 -0.7603 0.6355 -6.3698e6;
    0 0 0 1];
    
end

% Rot=[0.1107 -0.6258 -0.7721;0.9847 0.1741 0;0.1344 -0.7603 0.6355];

Dimu=diff(Flight_Data.NB_Counter)';
Dgps=diff(GPS.ms)';

dataSize=length(Flight_Data.Ax);
localGPS=zeros(dataSize,6);

for i=1:1:dataSize
    
    localGPSP=[1 0 0 0;0 -1 0 0;0 0 -1 0]*Rot*[GPS.x(i);GPS.y(i);GPS.z(i);1]+[-157.2;-108.2;222];
    localGPSV=[1 0 0 0;0 -1 0 0;0 0 -1 0]*Rot*[GPS.vx(i);GPS.vy(i);GPS.vz(i);0];
    
    localGPS(i,:)=[localGPSP' localGPSV'];
    
end

ALPHAQ=1;
Q=diag([0,0,0,2.3453e-6,3.2589e-6,2.8023e-5,3.6204e-7,3.6204e-7,6.3775e-7])*ALPHAQ;

% ALPHA=1;
% R=diag([ALPHA*0.3074,ALPHA*0.1414,ALPHA*1.7186,ALPHA*7.5638e-4,ALPHA*5.8469e-4,0.0023]);

ALPHA=2;
R=diag([ALPHA*.1023,ALPHA*.5060,ALPHA*.5120,ALPHA*3.89e-4,ALPHA*1.113e-4,ALPHA*9.84e-4]);

P=eye(9);

H=zeros(6,9);
H(1,1)=1;H(2,2)=1;H(3,3)=1;H(4,4)=1;H(5,5)=1;H(6,6)=1;

dt=Flight_Data.Delta_Time;

EKF.phi=zeros(dataSize,1);
EKF.theta=zeros(dataSize,1);
EKF.psi=zeros(dataSize,1);

EKF.x=zeros(dataSize,1);
EKF.y=zeros(dataSize,1);
EKF.z=zeros(dataSize,1);

EKF.vx=zeros(dataSize,1);
EKF.vy=zeros(dataSize,1);
EKF.vz=zeros(dataSize,1);

EKFstates=zeros(9,1);

EKFstates(1)=localGPS(100,1);
EKFstates(2)=localGPS(100,2);
EKFstates(3)=localGPS(100,3);

EKFstates(4)=localGPS(100,4);
EKFstates(5)=localGPS(100,5);
EKFstates(6)=localGPS(100,6);

for i=500:1:dataSize-500
    
    if(Dimu(i)==1 || Dimu(i)==-255)
    
        s_phi=sin(EKFstates(7));
        c_phi=cos(EKFstates(7));

        s_theta=sin(EKFstates(8));
        c_theta=cos(EKFstates(8));
        sec_theta=sec(EKFstates(8));
        t_theta=tan(EKFstates(8));

        s_psi=sin(EKFstates(9));
        c_psi=cos(EKFstates(9));
    
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
        0 0 0 0 0 1 dt(i)*(Flight_Data.Ay(i)*(c_phi*c_theta) - Flight_Data.Az(i)*(s_phi*c_theta))                                          -dt(i)*(Flight_Data.Ax(i)*(c_theta) + Flight_Data.Ay(i)*(s_theta*s_phi) + Flight_Data.Az(i)*(c_phi*s_theta))                                                                                         0                                                                                         ;
        0 0 0 0 0 0 1+dt(i)*t_theta*(c_phi*Flight_Data.Q(i)- s_phi*Flight_Data.R(i))                                        dt(i)*sec_theta*sec_theta*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                                                                               0                                                                                        ;
        0 0 0 0 0 0 -dt(i)*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                 1                                                                                                                                                                                                           0                                                                                        ;
        0 0 0 0 0 0 dt(i)*sec_theta*(c_phi*Flight_Data.Q(i) - s_phi*Flight_Data.R(i))                                         dt(i)*sec_theta*t_theta*(s_phi*Flight_Data.Q(i) + c_phi*Flight_Data.R(i))                                                                                                                 1                                                                                        ];                
    
    
    
        P=A*P*A' + Q;
    end
    
    if(Dgps(i)==20)    

        K=P*H'/(H*P*H'+R);
    
        EKFstates=EKFstates + K*([localGPS(i,1);localGPS(i,2);localGPS(i,3);localGPS(i,4);localGPS(i,5);localGPS(i,6)]-EKFstates(1:6));     
    
    
    
        P=(eye(9)-K*H)*P; 
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
    
    
end

EKFfunc=EKF;
funclocalGPS=localGPS;
end


