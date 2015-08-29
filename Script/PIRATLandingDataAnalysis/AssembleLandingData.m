clear all
close all
clc


FileSelectOption={'Select File','Exit Menu'};
i=1;

while(1)
    MenuSelect=menu('Select Landing Data Mat File:','Select File','Exit Menu');
    if strcmp(FileSelectOption(MenuSelect),'Select File')
        [FileName,PathName] = uigetfile('C:\Users\Tanmay\Desktop\Work\Research\Matlab_Analysis\Flight_Data\FPV_Plane','Select data file');
        if FileName==0, return, end        
        load(fullfile(PathName,FileName))
        k=1;
         while k<=NumLanding
            AssembledLandingDataZ{i}=ZData{k};
            AssembledLandingDatatheta{i}=PitchData{k};
            AssembledLandingDataElev{i}=ElevatorDeg{k};
            AssembledLandingDataQ{i}=Qdata{k};
            AssembledLandingDataR{i}=Rdata{k};
            AssembledLandingDataRoll{i}=RollData{k};
            i=i+1;
            k=k+1;
        end
        k=1;
        
    end
    
    if strcmp(FileSelectOption(MenuSelect),'Exit Menu')
        break;
    end
end

dataName={'1st','2nd','3rd','4th','5th','6th','7th','8tht','9th','10th','11th','12th','13th'};
colors = distinguishable_colors(15);

figure()
hold on
grid on
for i=1:length(AssembledLandingDataZ)
    h(i)=plot(AssembledLandingDataZ{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDatatheta)
    h(i)=plot(AssembledLandingDatatheta{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDataElev)
    h(i)=plot(AssembledLandingDataElev{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDataZ)
    h(i)=plot(AssembledLandingDataRoll{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDataZ)
    h(i)=plot(AssembledLandingDataQ{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

figure()
hold on
grid on
for i=1:length(AssembledLandingDataZ)
    h(i)=plot(AssembledLandingDataR{i},'Color',colors(i,:),'LineWidth',2);
end

legend(h,dataName{1:length(AssembledLandingDataZ)});
hold off

save ('AssembledLandingData.Mat','AssembledLandingDatatheta','AssembledLandingDataZ','AssembledLandingDataElev','AssembledLandingDataRoll','AssembledLandingDataQ','AssembledLandingDataR')