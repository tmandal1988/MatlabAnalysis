clear all
close all
clc

EKFstates=ones(5,1);
EKFstates(5)=30;
P=eye(5);
Q1=diag([10,10,10,10,10]);
R=0.1;
H=[1 0 0 0 0];

dt=0.02;

load 'AssembledLandingDataMod.Mat'
dataNumber=11;
Pitch=AssembledLandingDatathetaMod{dataNumber};
Roll=AssembledLandingDataRollMod{dataNumber};
Q=AssembledLandingDataQMod{dataNumber};
R1=AssembledLandingDataRMod{dataNumber};
p=AssembledLandingDataElevMod{dataNumber};

PilotParameters.K=zeros(1,lengthLim);
PilotParameters.Lead=zeros(1,lengthLim);
PilotParameters.Lag=zeros(1,lengthLim);
PilotParameters.tau=zeros(1,lengthLim);
PilotParameters.estCommand=zeros(1,lengthLim);
PilotParameters.CalcCommand=zeros(1,lengthLim);

PitchRateErrorA=Q*cosd(Roll)-R1*sind(Roll);
pvar=zeros(1,lengthLim);
Leadvar=zeros(1,lengthLim);
Lagvar=zeros(1,lengthLim);
Kvar=zeros(1,lengthLim);
tauvar=zeros(1,lengthLim);


for i=40:1:lengthLim-20
    delayIndex=i-EKFstates(5);
    n=fix(delayIndex);
    d=abs(delayIndex-n);
    
%     if n==0;
%         PitchError=meanTheta-(functiongx(2-d)*Pitch(i-n-2)+functionfx(1-d)*Pitch(i-n-1)+functionfx(d)*Pitch(i-n));
%         PitchRateError=(functiongx(2-d)*PitchRateErrorA(i-n-2)+functionfx(1-d)*PitchRateErrorA(i-n-1)+functionfx(d)*PitchRateErrorA(i-n));
%         PitchRateErrorRate=functiongxdash(2-d)*PitchRateErrorA(i-n-2)+functionfxdash(1-d)*PitchRateErrorA(i-n-1)+functionfxdash(d)*PitchRateErrorA(i-n);
%     else
        PitchError=(meanTheta-(functiongx(1+d)*Pitch(n-1)+functionfx(d)*Pitch(n)+functionfx(1-d)*Pitch(n+1)+functiongx(2-d)*Pitch(n+2)));
        PitchRateError=functiongx(1+d)*PitchRateErrorA(n-1)+functionfx(d)*PitchRateErrorA(n)+functionfx(1-d)*PitchRateErrorA(n+1)+functiongx(2-d)*PitchRateErrorA(n+2);
        
        PitchRateErrorRate=functiongxdash(1+d)*PitchRateErrorA(n-1)+functionfxdash(d)*PitchRateErrorA(n)+functionfxdash(1-d)*PitchRateErrorA(n+1)+functiongxdash(2-d)*PitchRateErrorA(n+2);
%     end
        
   
    
%     PitchError=meanTheta-Pitch(i-EKFstates(5));
%     PitchRateError=Q(i-EKFstates(5))*cosd(Roll(i-EKFstates(5)))...
%     -R1(i-EKFstates(5))*sind(Roll(i-EKFstates(5)));
%     PitchRateError1=Q(i-EKFstates(5)+1)*cosd(Roll(i-EKFstates(5)+1))...
%     -R1(i-EKFstates(5)+1)*sind(Roll(i-EKFstates(5))+1);
%     PitchRateErrorRate=(PitchRateError1-PitchRateError)/dt;
    
    EKFstates(1)=EKFstates(1)+dt*(EKFstates(4)*EKFstates(2)*PitchRateError+EKFstates(4)*PitchError-EKFstates(1))/EKFstates(3);
    EKFstates(2)=EKFstates(2);
    EKFstates(3)=EKFstates(3);
    EKFstates(4)=EKFstates(4);
    EKFstates(5)=EKFstates(5);
    
    A=[1-(dt/EKFstates(3)) dt*EKFstates(4)*PitchRateError/EKFstates(3) -dt*(EKFstates(4)*EKFstates(2)*PitchRateError+EKFstates(4)*PitchError-EKFstates(1))/(EKFstates(3)*EKFstates(3))...
        dt*(EKFstates(2)*PitchRateError+PitchError)/EKFstates(3) dt*(EKFstates(4)*EKFstates(2)*PitchRateErrorRate +EKFstates(4)*PitchRateError)/EKFstates(3);
        0 1 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0;
        0 0 0 0 1];
    
    P=A*P*A' + Q1;
    
    K=P*H'/(H*P*H'+R);
    
    EKFstates=EKFstates + K*(p(i)-EKFstates(1));
    
    P=(eye(5)-K*H)*P; 
    
    PilotParameters.Lead(i)=EKFstates(2);
    PilotParameters.Lag(i)=EKFstates(3);
    PilotParameters.K(i)=EKFstates(4);
%     if(EKFstates(5)<0)
%         EKFstates(5)=0;
%     end
%     EKFstates(5)=round(EKFstates(5));
    PilotParameters.tau(i)=EKFstates(5);
    PilotParameters.estCommand(i)=EKFstates(1);
    
    pvar(i)=P(1,1);
    Leadvar(i)=P(2,2);
    Lagvar(i)=P(3,3);
    Kvar(i)=P(4,4);
    tauvar(i)=P(5,5);   
    
end

PilotParameters.CalcCommand=PilotParameters.estCommand;
for i=42:1:lengthLim-20
    delayIndex=i-PilotParameters.tau(i);
    n=fix(delayIndex);
    d=abs(delayIndex-n);
    
    PitchError=(meanTheta-(functiongx(1+d)*Pitch(n-1)+functionfx(d)*Pitch(n)+functionfx(1-d)*Pitch(n+1)+functiongx(2-d)*Pitch(n+2)));
    PitchRateError=functiongx(1+d)*PitchRateErrorA(n-1)+functionfx(d)*PitchRateErrorA(n)+functionfx(1-d)*PitchRateErrorA(n+1)+functiongx(2-d)*PitchRateErrorA(n+2);
    PilotParameters.CalcCommand(i)=PilotParameters.CalcCommand(i-1)+dt*(PilotParameters.K(i)*PilotParameters.Lead(i)*PitchRateError+PilotParameters.K(i)*PitchError-PilotParameters.CalcCommand(i-1))/PilotParameters.Lag(i);
end

EstimatedData=[PilotParameters.Lead'  PilotParameters.Lag' PilotParameters.K' PilotParameters.tau'];
EstimatedDatavar=[Leadvar' Lagvar' Kvar' tauvar'];
colors = distinguishable_colors(15);
dataName={'Lead','Lag','K','tau'};

figure()
hold on
grid on
for i=1:4
    h(i)=plot(EstimatedData(:,i),'Color',colors(i,:),'LineWidth',2);
end
legend(h,dataName);
hold off

figure()
hold on
grid on
for i=1:4
    h(i)=plot(EstimatedDatavar(:,i),'Color',colors(i,:),'LineWidth',2);
end
legend(h,dataName);
hold off

figure()
hold on
grid on
h1(1)=plot(PilotParameters.estCommand,'Color',colors(1,:),'LineWidth',2);
h1(2)=plot(p,'Color',colors(2,:),'LineWidth',2);
h1(3)=plot(PilotParameters.CalcCommand,'Color',colors(3,:),'LineWidth',2);

legend(h1,{'Est.','Truth','Calculated'});
hold off