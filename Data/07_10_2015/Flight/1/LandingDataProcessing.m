clear all
clc
close all

load 'EKF.mat'
load 'Data.mat'

fid=fopen('LandingIndices.txt');

tline=fgetl(fid);
i=1;
while ischar(tline)
    tline=fgetl(fid);
    if (tline ~=-1)
        indices(i)=str2num(tline);
    end
    i=i+1;
    
end

fclose(fid);

ZData=cell(20,1);
PitchData=cell(20,1);
ElevatorDeg=cell(20,1);
Qdata=cell(20,1);
RollData=cell(20,1);
Rdata=cell(20,1);
colors = distinguishable_colors(15);
dataName={'First','Second','Third','Fourth','Fifth','Sixth','Seventh','Eight','Ninth','Tenth'};

i=1;
k=1;
NumLanding=length(indices)/2;
while i<length(indices)
    ZData{k}=-EKF.z(indices(i):indices(i+1));
    PitchData{k}=EKF.theta(indices(i):indices(i+1));
    PWMms=(Flight_Data.Elevator(indices(i):indices(i+1))*0.00057675+0.91439)*0.001;
    ElevatorDeg{k}=(PWMms-0.0014727)/0.000016617;
    Qdata{k}=Flight_Data.Q(indices(i):indices(i+1));
    Rdata{k}=Flight_Data.R(indices(i):indices(i+1));
    RollData{k}=EKF.phi(indices(i):indices(i+1));
    i=i+2;
    k=k+1;
end

for i=2:1:length(ElevatorDeg{3,1})
    if abs(ElevatorDeg{3,1}(i)-ElevatorDeg{3,1}(i-1))>7
        ElevatorDeg{3,1}(i)=ElevatorDeg{3,1}(i-1);
    end
    
end

for i=2:1:length(ElevatorDeg{5,1})
    if abs(ElevatorDeg{5,1}(i)-ElevatorDeg{5,1}(i-1))>7
        ElevatorDeg{5,1}(i)=ElevatorDeg{5,1}(i-1);
    end
    
end

figure()
hold on
grid on
for i=1:length(indices)/2
    h(i)=plot(ZData{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(indices)/2});
hold off

figure()
hold on
grid on
for i=1:length(indices)/2
    h1(i)=plot(PitchData{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h1,dataName{1:length(indices)/2});
hold off

figure()
hold on
grid on
for i=1:length(indices)/2
    h2(i)=plot(ElevatorDeg{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h2,dataName{1:length(indices)/2});
hold off

save ('LandingData.Mat','ZData','PitchData','ElevatorDeg','NumLanding','RollData','Qdata','Rdata')
