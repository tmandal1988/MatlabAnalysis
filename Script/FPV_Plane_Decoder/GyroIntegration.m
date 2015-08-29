clc

biasQ=mean(Flight_Data.Q(1:8000));
biasP=mean(Flight_Data.P(1:8000));
biasR=mean(Flight_Data.R(1:8000));

dataSize=length(Flight_Data.P);

pitch=zeros(dataSize,1);
roll=zeros(dataSize,1);
yaw=zeros(dataSize,1);

for i=2:1:dataSize
    roll(i)=roll(i-1)+ ((Flight_Data.P(i)-biasP) + sind(roll(i-1))*tand(pitch(i-1))*(-Flight_Data.Q(i)+biasQ)+cosd(roll(i-1))*tand(pitch(i-1))*(Flight_Data.R(i-1)-biasR))*0.01;
    pitch(i)=pitch(i-1)+ (cosd(roll(i-1))*(-Flight_Data.Q(i)+biasQ) -sind(roll(i-1))*(Flight_Data.R(i)-biasR))*0.01;
    yaw(i)=yaw(i-1)+ (sind(roll(i-1))*secd(pitch(i-1))*(-Flight_Data.Q(i)+biasQ)+cosd(roll(i-1))*secd(pitch(i-1))*(Flight_Data.R(i)-biasR))*0.01;
    
end
