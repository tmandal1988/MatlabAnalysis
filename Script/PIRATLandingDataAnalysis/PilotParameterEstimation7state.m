clear all
close all
clc

EKFstates=ones(7,1);
EKFstates(5)=3;
EKFstates(7)=3;
P=eye(7);
Q1=diag([0.01,0.01,0.01,0.01,0.01,0.01,0.01]);
R=0.01;
H=[1 0 0 0 0 0 0];

dt=0.02;

load 'AssembledLandingDataMod.Mat'
dataNumber=11;
Pitch=AssembledLandingDatathetaMod{dataNumber};
Roll=AssembledLandingDataRollMod{dataNumber};
Q=AssembledLandingDataQMod{dataNumber};
R1=AssembledLandingDataRMod{dataNumber};
p=AssembledLandingDataElevMod{dataNumber};

PilotParameters.K1=zeros(1,lengthLim);
PilotParameters.Lead=zeros(1,lengthLim);
PilotParameters.Lag=zeros(1,lengthLim);
PilotParameters.tau1=zeros(1,lengthLim);
PilotParameters.K2=zeros(1,lengthLim);
PilotParameters.tau2=zeros(1,lengthLim);
PilotParameters.estCommand=zeros(1,lengthLim);
PilotParameters.CalcCommand=zeros(1,lengthLim);

PitchRateErrorA=Q*cosd(Roll)-R1*sind(Roll);
pvar=zeros(1,lengthLim);
Leadvar=zeros(1,lengthLim);
Lagvar=zeros(1,lengthLim);
K1var=zeros(1,lengthLim);
tau1var=zeros(1,lengthLim);
K2var=zeros(1,lengthLim);
tau2var=zeros(1,lengthLim);

for i=40:1:lengthLim-20
    delayIndex1=i-EKFstates(5);
    n1=fix(delayIndex1);
    d1=abs(delayIndex1-n1);
    
    delayIndex2=i-EKFstates(7);
    n2=fix(delayIndex2);
    d2=abs(delayIndex2-n2);
    
    PitchError1=(meanTheta-(functiongx(1+d1)*Pitch(n1-1)+functionfx(d1)*Pitch(n1)+functionfx(1-d1)*Pitch(n1+1)+functiongx(2-d1)*Pitch(n1+2)));
    PitchRateError1=functiongx(1+d1)*PitchRateErrorA(n1-1)+functionfx(d1)*PitchRateErrorA(n1)+functionfx(1-d1)*PitchRateErrorA(n1+1)+functiongx(2-d1)*PitchRateErrorA(n1+2);          
    PitchRateErrorRate1=functiongxdash(1+d1)*PitchRateErrorA(n1-1)+functionfxdash(d1)*PitchRateErrorA(n1)+functionfxdash(1-d1)*PitchRateErrorA(n1+1)+functiongxdash(2-d1)*PitchRateErrorA(n1+2);
    
    PitchError2=(meanTheta-(functiongx(1+d2)*Pitch(n2-1)+functionfx(d2)*Pitch(n2)+functionfx(1-d2)*Pitch(n2+1)+functiongx(2-d2)*Pitch(n2+2)));
    Pitch2=functiongx(1+d2)*Pitch(n2-1)+functionfx(d2)*Pitch(n2)+functionfx(1-d2)*Pitch(n2+1)+functiongx(2-d2)*Pitch(n2+2);
    PitchRateError2=functiongx(1+d2)*PitchRateErrorA(n2-1)+functionfx(d2)*PitchRateErrorA(n2)+functionfx(1-d2)*PitchRateErrorA(n2+1)+functiongx(2-d2)*PitchRateErrorA(n2+2);          
    PitchRateErrorRate2=functiongxdash(1+d2)*PitchRateErrorA(n2-1)+functionfxdash(d2)*PitchRateErrorA(n2)+functionfxdash(1-d2)*PitchRateErrorA(n2+1)+functiongxdash(2-d2)*PitchRateErrorA(n2+2);
    
    EKFstates(1)=EKFstates(1)+dt*(EKFstates(4)*EKFstates(2)*PitchRateError1+EKFstates(4)*PitchError1-EKFstates(1)+EKFstates(6)*Pitch2-EKFstates(6)*EKFstates(3)* PitchRateError2)/EKFstates(3);
    EKFstates(2)=EKFstates(2);
    EKFstates(3)=EKFstates(3);
    EKFstates(4)=EKFstates(4);
    EKFstates(5)=EKFstates(5);
    EKFstates(6)=EKFstates(6);
    EKFstates(7)=EKFstates(7);
    
        A=[1-(dt/EKFstates(3)) dt*EKFstates(4)*PitchRateError1/EKFstates(3) -dt*(EKFstates(4)*EKFstates(2)*PitchRateError1+EKFstates(4)*PitchError1-EKFstates(1)+EKFstates(6)*Pitch2-EKFstates(6)*EKFstates(3)* PitchRateError2)*(-EKFstates(6)* PitchRateError2)/(EKFstates(3)*EKFstates(3)) ...
            dt*(EKFstates(2)*PitchRateError1+PitchError1)/EKFstates(3) dt*(EKFstates(4)*EKFstates(2)*PitchRateErrorRate1 +EKFstates(4)*PitchRateError1)/EKFstates(3) dt*(Pitch2-EKFstates(3)* PitchRateError2)/EKFstates(3) dt*(EKFstates(6)*-PitchRateError2+EKFstates(6)*EKFstates(3)* PitchRateErrorRate2)/EKFstates(3) ;
        0 1 0 0 0 0 0;
        0 0 1 0 0 0 0;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 1];
    
        
    P=A*P*A' + Q1;
    
    K=P*H'/(H*P*H'+R);
    
    EKFstates=EKFstates + K*(p(i)-EKFstates(1));
    
    P=(eye(7)-K*H)*P; 
    
    PilotParameters.Lead(i)=EKFstates(2);
    PilotParameters.Lag(i)=EKFstates(3);
    PilotParameters.K1(i)=EKFstates(4);
%     if(EKFstates(5)<0)
%         EKFstates(5)=0;
%     end
%     EKFstates(5)=round(EKFstates(5));
    PilotParameters.tau1(i)=EKFstates(5);
    PilotParameters.K2(i)=EKFstates(6);
    PilotParameters.tau2(i)=EKFstates(7);
    PilotParameters.estCommand(i)=EKFstates(1);
    
    pvar(i)=P(1,1);
    Leadvar(i)=P(2,2);
    Lagvar(i)=P(3,3);
    K1var(i)=P(4,4);
    tau1var(i)=P(5,5);   
    K2var(i)=P(6,6);
    tau2var(i)=P(7,7);
    
end

PilotParameters.CalcCommand=PilotParameters.estCommand;
for i=42:1:lengthLim-20
    delayIndex1=i-PilotParameters.tau1(i);
    n1=fix(delayIndex1);
    d1=abs(delayIndex1-n1);
    
    delayIndex2=i-PilotParameters.tau2(i);
    n2=fix(delayIndex2);
    d2=abs(delayIndex2-n2);
    
    Pitch2=functiongx(1+d2)*Pitch(n2-1)+functionfx(d2)*Pitch(n2)+functionfx(1-d2)*Pitch(n2+1)+functiongx(2-d2)*Pitch(n2+2);
    PitchRateError2=functiongx(1+d2)*PitchRateErrorA(n2-1)+functionfx(d2)*PitchRateErrorA(n2)+functionfx(1-d2)*PitchRateErrorA(n2+1)+functiongx(2-d2)*PitchRateErrorA(n2+2);
    
    PitchError1=(meanTheta-(functiongx(1+d1)*Pitch(n1-1)+functionfx(d1)*Pitch(n1)+functionfx(1-d1)*Pitch(n1+1)+functiongx(2-d1)*Pitch(n1+2)));
    PitchRateError1=functiongx(1+d1)*PitchRateErrorA(n1-1)+functionfx(d1)*PitchRateErrorA(n1)+functionfx(1-d1)*PitchRateErrorA(n1+1)+functiongx(2-d1)*PitchRateErrorA(n1+2);
    PilotParameters.CalcCommand(i)=PilotParameters.CalcCommand(i-1)+dt*(PilotParameters.K1(i)*PilotParameters.Lead(i)*PitchRateError1+PilotParameters.K1(i)*PitchError1-PilotParameters.CalcCommand(i-1) ...
        +PilotParameters.K2(i)*Pitch2-PilotParameters.K2(i)*PilotParameters.Lag(i)* PitchRateError2)/PilotParameters.Lag(i);
end

EstimatedData=[PilotParameters.Lead'  PilotParameters.Lag' PilotParameters.K2' PilotParameters.tau1' PilotParameters.K2' PilotParameters.tau2'];
EstimatedDatavar=[Leadvar' Lagvar' K1var' tau1var' K2var' tau2var'];
colors = distinguishable_colors(15);
dataName={'Lead','Lag','K1','tau1','K2','tau2'};

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

