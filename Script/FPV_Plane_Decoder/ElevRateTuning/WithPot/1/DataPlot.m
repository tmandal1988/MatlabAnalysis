clear all
close all
clc

load Data.mat
load LiearFit.mat

for i=1:length(Actuator.Elev)
    if(Actuator.Elev(i)==0)
        Ecomm(i)=0;
    else
        Ecomm(i)=((Actuator.Elev(i)/10000000.0)-0.0014727)/0.000016529;
    end
    
    Elevatorms(i)=(Flight_Data.Elevator(i)*0.00057675+0.91439)*0.001;
    Elevatordeg(i)=(Elevatorms(i)-0.001472)/0.000016529;
end
    
plot(Ecomm,'LineWidth',2);
hold on
plot(Elevatordeg,'r','LineWidth',2);

figure()
plot(polyval(p,Flight_Data.ADC1),'r','LineWidth',2)
yfitTotal=polyval(p,Flight_Data.ADC1);
hold on
plot(Elevatordeg,'LineWidth',2)

figure()
plot(Ecomm,'LineWidth',2);
hold on
plot(yfitTotal,'r','LineWidth',2)