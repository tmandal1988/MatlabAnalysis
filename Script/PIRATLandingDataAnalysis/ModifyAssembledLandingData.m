clear all
close all
clc

load 'AssembledLandingData.Mat'

lengthLim=490;

for i=1:length(AssembledLandingDataElev)
    AssembledLandingDataElevMod{i}=AssembledLandingDataElev{i}(1:lengthLim);
    AssembledLandingDatathetaMod{i}=AssembledLandingDatatheta{i}(1:lengthLim)';
    AssembledLandingDataZMod{i}=AssembledLandingDataZ{i}(1:lengthLim)';
    AssembledLandingDataRollMod{i}=AssembledLandingDataRoll{i}(1:lengthLim)';
    AssembledLandingDataQMod{i}=AssembledLandingDataQ{i}(1:lengthLim)';
    AssembledLandingDataRMod{i}=AssembledLandingDataR{i}(1:lengthLim)';
end

ConcatElevMod=AssembledLandingDataElevMod{1};
ConcatZMod=AssembledLandingDataZMod{1};
ConcatThetaMod=AssembledLandingDatathetaMod{1};

for i=2:length(AssembledLandingDataElev)
    ConcatElevMod=horzcat(ConcatElevMod,AssembledLandingDataElevMod{i});
    ConcatZMod=horzcat(ConcatZMod,AssembledLandingDataZMod{i});
    ConcatThetaMod=horzcat(ConcatThetaMod,AssembledLandingDatathetaMod{i});
end

medianElev=median(ConcatElevMod);
meanElev=mean(ConcatElevMod);
varElev=var(ConcatElevMod);

medianTheta=median(ConcatThetaMod);
meanTheta=mean(ConcatThetaMod);
varTheta=var(ConcatThetaMod);

save ('AssembledLandingDataMod.Mat','AssembledLandingDatathetaMod','AssembledLandingDataZMod','AssembledLandingDataElevMod','lengthLim','medianTheta','meanTheta','varTheta','medianElev','meanElev','varElev','AssembledLandingDataRollMod','AssembledLandingDataQMod','AssembledLandingDataRMod')

colors = distinguishable_colors(15);
dataName={'1st','2nd','3rd','4th','5th','6th','7th','8tht','9th','10th','11th','12th','13th'};

figure()
hold on
grid on
for i=1:length(AssembledLandingDataZMod)
    h(i)=plot(AssembledLandingDataZMod{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZMod)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDatathetaMod)
    h(i)=plot(AssembledLandingDatathetaMod{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZMod)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDataElevMod)
    h(i)=plot(AssembledLandingDataElevMod{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZMod)});
hold off

save ('AssembledLandingData.Mat','AssembledLandingDatatheta','AssembledLandingDataZ','AssembledLandingDataElev')