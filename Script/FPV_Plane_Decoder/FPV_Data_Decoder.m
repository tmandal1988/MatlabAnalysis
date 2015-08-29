clear all
close all
clc

[FileName,PathName] = uigetfile('/home/chaos/Desktop/Work/MatlabAnalysis/Data','Select data file');
if FileName==0, return, end

if(exist(fullfile(PathName,'Data.mat'),'file')==2)
    load(fullfile(PathName,'Data.mat'));
    fprintf('Decoded File Found \n');
    clearvars -except Flight_Data Raw_Data GPS Actuator
        
else
    fid = fopen( fullfile(PathName,FileName),'r' );
    Raw_Data=fread(fid);
    fclose(fid);

    HEADER_SIZE=3;
    PACKET_SIZE=140;%Including the Header
    Packet_Length=length(Raw_Data);

    header_index=find((Raw_Data==170) + circshift((Raw_Data==171),[-1,0]) + circshift((Raw_Data==187),[-2,0])==3);

    n_packet=length(header_index);
    n_bad_packet=sum(diff(header_index)~=PACKET_SIZE);
    per_bad=100*(n_bad_packet/n_packet);
    fprintf('Percentage of Bad Packets %d \n', per_bad);
    bad_packet_index=(diff(header_index)~=PACKET_SIZE);

    k=0;

    %Initialize Flight Data
    Flight_Data.NB_Counter=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Ax=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Ay=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Az=zeros(1,(n_packet-n_bad_packet)-1);

    Flight_Data.P=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Q=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.R=zeros(1,(n_packet-n_bad_packet)-1);

    Flight_Data.Delta_Time=zeros(1,(n_packet-n_bad_packet)-1);

    Flight_Data.Roll_Fil=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Pitch_Fil=zeros(1,(n_packet-n_bad_packet)-1);

    Flight_Data.Throttle=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Elevator=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Aeliron=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.Rudder=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.biasGx=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.biasGy=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.biasGz=zeros(1,(n_packet-n_bad_packet)-1);
    
    Flight_Data.modemChar=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.PilotDelayedCommand=zeros(1,(n_packet-n_bad_packet)-1);


    GPS.x=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.y=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.z=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.vx=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.vy=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.vz=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.sol_sat=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.week=zeros(1,(n_packet-n_bad_packet)-1);
    GPS.ms=zeros(1,(n_packet-n_bad_packet)-1);
    
   

    Flight_Data.alpha=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC0=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC1=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC2=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC3=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC4=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC5=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC6=zeros(1,(n_packet-n_bad_packet)-1);
    Flight_Data.ADC7=zeros(1,(n_packet-n_bad_packet)-1);

    
    Flight_Data.controlSwitch=zeros(1,(n_packet-n_bad_packet)-1);

    Actuator.Elev=zeros(1,(n_packet-n_bad_packet)-1);
    GPScontrolSwitchActive=zeros(3,(n_packet-n_bad_packet)-1);


    code_time=tic;
    header_index_report = floor(length(header_index)/10);

for i=1:length(header_index)-1
        if bad_packet_index(i)==0;
         k=k+1;
         start=header_index(i)+HEADER_SIZE;
         Flight_Data.R(k)=bytes2engr(Raw_Data(start+0:start+1),'short_nb')*0.02;
         Flight_Data.Q(k)=bytes2engr(Raw_Data(start+2:start+3),'short_nb')*0.02;%some random value
         Flight_Data.Ax(k)=bytes2engr(Raw_Data(start+4:start+5),'short_nb')*0.00025;
         Flight_Data.Ay(k)=bytes2engr(Raw_Data(start+6:start+7),'short_nb')*0.00025;
         Flight_Data.Az(k)=bytes2engr(Raw_Data(start+8:start+9),'short_nb')*0.00025;
         Flight_Data.P(k)=bytes2engr(Raw_Data(start+10:start+11),'short_nb')*0.02;
        
         Flight_Data.Roll_Fil(k)=bytes2engr(Raw_Data(start+20:start+21),'short_nb')/250;
         Flight_Data.Pitch_Fil(k)=bytes2engr(Raw_Data(start+22:start+23),'short_nb')/250;
        
         Flight_Data.Throttle(k)=bytes2engr(Raw_Data(start+12:start+13),'ushort_nb');
         Flight_Data.Elevator(k)=bytes2engr(Raw_Data(start+14:start+15),'ushort_nb');
         Flight_Data.Aeliron(k)=bytes2engr(Raw_Data(start+16:start+17),'ushort_nb');
         Flight_Data.Rudder(k)=bytes2engr(Raw_Data(start+18:start+19),'ushort_nb');
        
         GPS.x(k)=bytes2engr(Raw_Data(start+31:-1:start+24),'double');
         GPS.y(k)=bytes2engr(Raw_Data(start+39:-1:start+32),'double');
         GPS.z(k)=bytes2engr(Raw_Data(start+47:-1:start+40),'double');
        
         GPS.vx(k)=bytes2engr(Raw_Data(start+55:-1:start+48),'double');
         GPS.vy(k)=bytes2engr(Raw_Data(start+63:-1:start+56),'double');
         GPS.vz(k)=bytes2engr(Raw_Data(start+71:-1:start+64),'double');
         
         GPS.week(k)=bytes2engr(Raw_Data(start+93:start+94),'ushort');
         GPS.ms(k)=bytes2engr(Raw_Data(start+95:start+98),'ulong');
        
         Actuator.Elev(k)=bytes2engr(Raw_Data(start+88:start+89),'ushort_nb'); 
         
         Flight_Data.modemChar(k)=Raw_Data(start+99);
         Flight_Data.PilotDelayedCommand(k)=bytes2engr(Raw_Data(start+100:start+103),'float');
         
        Flight_Data.biasGx(k)=double(typecast(uint8(Raw_Data(start+90)),'int8'))/1000;
        Flight_Data.biasGy(k)=double(typecast(uint8(Raw_Data(start+91)),'int8'))/1000;
        Flight_Data.biasGz(k)=double(typecast(uint8(Raw_Data(start+92)),'int8'))/1000;
        

        
        
        GPS.sol_sat(k)=Raw_Data(start+72);
        
        Flight_Data.alpha(k)=Raw_Data(start+73)/75;
        
        Flight_Data.ADC0(k)=Raw_Data(start+73)/75;
        Flight_Data.ADC1(k)=Raw_Data(start+74)/75;
        Flight_Data.ADC2(k)=Raw_Data(start+75)/75;
        Flight_Data.ADC3(k)=Raw_Data(start+76)/75;
        
        Flight_Data.ADC4(k)=Raw_Data(start+77)/75;
        Flight_Data.ADC5(k)=Raw_Data(start+78)/75;
        Flight_Data.ADC6(k)=Raw_Data(start+79)/75;
        Flight_Data.ADC7(k)=Raw_Data(start+80)/75;
        
        
        Flight_Data.controlSwitch(k)=Raw_Data(start+81);      
               
        Flight_Data.Delta_Time(k)=Raw_Data(start+134)/10000;
        Flight_Data.NB_Counter(k)=Raw_Data(start+135); 
        Flight_Data.GCSDelay(k)=single(typecast(uint8([Raw_Data(start+105) Raw_Data(start+106)]),'int16'))/10;
        Flight_Data.GCSLimit(k)=single(typecast(uint8([Raw_Data(start+107) Raw_Data(start+108)]),'int16'))/100;
       % Flight_Data.GCSTrim=Raw_Data(start+109)+ Raw_Data(start+110);
        Flight_Data.GCSTrim(k)=single(typecast(uint8([Raw_Data(start+109) Raw_Data(start+110)]),'int16'))/100;
        end
        
    if mod(i,header_index_report) ==0
        fprintf('\tProcess %2.0f%% Completed @%4.0f sec\n', 10*i/header_index_report, toc(code_time));
    end
end
    

 
 save(fullfile(PathName,'Data.mat'),'Flight_Data','Raw_Data','GPS','Actuator');
  
end

 [EKF,localGPS]=PIRATEKFv1(Flight_Data,GPS);
 
 Data2plotStr={'IMU','EKF-Position','EKF-Attitude','NB-Counter and Delta Time','GPS-ms','Pilot-Channel','Alpha','NB-Elevator Command','Gyro-Integration','Toggle Control Switch','Close-all','Exit'};
 
 exit=0;
 toggleCS=0;
 

 
 for i=1:1:length(EKF.x)
     if Flight_Data.controlSwitch==100
         GPScontrolSwitchActive(1,i)=EKF.x(i);
         GPScontrolSwitchActive(2,i)=EKF.y(i);
         GPScontrolSwitchActive(3,i)=EKF.z(i);
     end
 end

    
 
while(exit==0)
    Data2plot=menu('Select Data to plot:','IMU','EKF-Position','EKF-Attitude','NB-Counter and Delta Time','GPS-ms','Pilot-Channel','Alpha','NB-Elevator Command','Gyro-Integration','Toggle Control Switch','Close-all','Exit'); 
  
    if strcmp(Data2plotStr(Data2plot),'Toggle Control Switch')
        if toggleCS==0
            toggleCS=1;
        else
            toggleCS=0;
        end
    
    end  
 
    
    if strcmp(Data2plotStr(Data2plot),'Close-all')
        close all
    end
    
    if strcmp(Data2plotStr(Data2plot),'Exit')
        exit=1;
    end
    
    if strcmp(Data2plotStr(Data2plot),'IMU')
        close all
        h=figure();
        subplot(2,3,1)
        plot(Flight_Data.Ax,'LineWidth',2)
        m=max(Flight_Data.Ax);
        n=min(Flight_Data.Ax);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
        xlabel('Data Points')
        ylabel('Ax-g')

        subplot(2,3,2)
        plot(Flight_Data.Ay,'LineWidth',2)
        m=max(Flight_Data.Ay);
        n=min(Flight_Data.Ay);        
        grid on
        if toggleCS==1
           hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
        xlabel('Data Points')
        ylabel('Ay-g')

        subplot(2,3,3)
        plot(Flight_Data.Az,'LineWidth',2)
        grid on
        m=max(Flight_Data.Az);
        n=min(Flight_Data.Az);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100)-1,'m','LineWidth',2);
            ylim([n-6 m+6]);
        end

        xlabel('Data Points')
        ylabel('Az-g')

        subplot(2,3,4)
        plot(Flight_Data.P,'LineWidth',2)
        grid on
        m=max(Flight_Data.P);
        n=min(Flight_Data.P);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
    
        xlabel('Data Points')
        ylabel('P-deg/s')

        subplot(2,3,5)
        plot(Flight_Data.Q,'LineWidth',2)
        grid on
        m=max(Flight_Data.Q);
        n=min(Flight_Data.Q);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
     
        xlabel('Data Points')
        ylabel('Q-deg/s')


        subplot(2,3,6)
        plot(Flight_Data.R,'LineWidth',2)
        grid on
        m=max(Flight_Data.R);
        n=min(Flight_Data.R);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
     
        xlabel('Data Points')
        ylabel('R-deg/s')
    end

    if strcmp(Data2plotStr(Data2plot),'EKF-Position')
        close all
        figure()
        plot3(EKF.x,EKF.y,-EKF.z,'LineWidth',2);
        xlabel('x(m)')
        ylabel('y(m)')
        zlabel('z(m)')
        grid on
        if toggleCS==1
            hold on
            for i=1:1:length(EKF.x)
                plot3(GPScontrolSwitchActive(1,i),GPScontrolSwitchActive(2,i),GPScontrolSwitchActive(3,i),'r','*')
            end
        end
     
    end

    if strcmp(Data2plotStr(Data2plot),'EKF-Attitude')
        close all
        figure()
        subplot(3,1,1)
        plot(EKF.phi,'LineWidth',2)
        m=max(EKF.phi);
        n=min(EKF.phi);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
      
        ylabel('Roll (deg)');
        xlabel('Data Points');

        subplot(3,1,2)
        plot(EKF.theta,'LineWidth',2)
        grid on
        m=max(EKF.theta);
        n=min(EKF.theta);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
       
        ylabel('theta (deg)');
        xlabel('Data Points');

        subplot(3,1,3)
        plot(EKF.psi,'LineWidth',2)
        grid on
        m=max(EKF.psi);
        n=min(EKF.psi);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
      
        ylabel('Yaw (deg)');
        xlabel('Data Points');
    end

    if strcmp(Data2plotStr(Data2plot),'Gyro-Integration')
        close all
        figure()
        subplot(1,2,1)
        plot(Flight_Data.Roll_Fil,'r','LineWidth',2)
        grid on
        m=max(Flight_Data.Roll_Fil);
        n=min(Flight_Data.Roll_Fil);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
       
        xlabel('Data Points')
        ylabel('Roll-deg')


        subplot(1,2,2)
        plot(Flight_Data.Pitch_Fil,'LineWidth',2)
        grid on
        m=max(Flight_Data.Pitch_Fil);
        n=min(Flight_Data.Pitch_Fil);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
  
        xlabel('Data Points')
        ylabel('Pitch-deg')
    end

    if strcmp(Data2plotStr(Data2plot),'NB-Counter and Delta Time')
        close all
        figure()
        subplot(2,1,1)
        plot(Flight_Data.NB_Counter,'r','LineWidth',2)
        grid on
        m=max(Flight_Data.NB_Counter);
        n=min(Flight_Data.NB_Counter);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
     
        xlabel('Data Points')
        ylabel('Unsigned 8-bit integer counter')

        subplot(2,1,2)
        plot(Flight_Data.Delta_Time,'LineWidth',2)
        grid on
        m=max(Flight_Data.Delta_Time);
        n=min(Flight_Data.Delta_Time);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+0.02)/100),'m','LineWidth',2);
            ylim([n-0.03 m+0.03]);
        end
      
        xlabel('Data Points')
        ylabel('Time Difference between adjacent data point-s')
    end

    if strcmp(Data2plotStr(Data2plot),'Pilot-Channel')
        close all
        figure()
        subplot(2,2,1)
        plot(Flight_Data.Throttle,'LineWidth',2)
        grid on
        m=max(Flight_Data.Throttle);
        n=min(Flight_Data.Throttle);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-100 m+100]);
        end
       
        xlabel('Data Points')
        ylabel('Thorttle-11-bit unsigned integer')

        subplot(2,2,2)
        plot(Flight_Data.Elevator,'LineWidth',2)
        grid on
        m=max(Flight_Data.Elevator);
        n=min(Flight_Data.Elevator);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-100 m+100]);
        end
      
        xlabel('Data Points')
        ylabel('Elevator-11-bit unsigned integer')

        subplot(2,2,3)
        plot(Flight_Data.Aeliron,'LineWidth',2)
        grid on
        m=max(Flight_Data.Aeliron);
        n=min(Flight_Data.Aeliron);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-100 m+100]);
        end
      
        xlabel('Data Points')
        ylabel('Aeliron-11-bit unsigned integer')

        subplot(2,2,4)
        plot(Flight_Data.Rudder,'LineWidth',2)
        grid on
        m=max(Flight_Data.Rudder);
        n=min(Flight_Data.Rudder);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-100 m+100]);
        end
     
        xlabel('Data Points')
        ylabel('Rudder-11-bit unsigned integer')
    end

    if strcmp(Data2plotStr(Data2plot),'NB-Elevator Command')
        close all
        figure()
        plot(Actuator.Elev,'LineWidth',2);
        xlabel('Data Points')
        ylabel('PWM (ms) in 16-bit number')
        grid on
        m=max(Actuator.Elev);
        n=min(Actuator.Elev);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-100 m+100]);
        end
        
    end

    if strcmp(Data2plotStr(Data2plot),'GPS-ms')
        close all
        figure()
        plot(GPS.ms,'LineWidth',2)
        xlabel('Data Points')
        ylabel('GPS ms')
        grid on
        m=max(GPS.ms);
        n=min(GPS.ms);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
     
    end
    
    if strcmp(Data2plotStr(Data2plot),'Alpha')
        close all
        plot(Flight_Data.alpha,'LineWidth',2)
        grid on
        m=max(Flight_Data.alpha);
        n=min(Flight_Data.alpha);
        grid on
        if toggleCS==1
            hold on
            plot(Flight_Data.controlSwitch*((m+3)/100),'m','LineWidth',2);
            ylim([n-6 m+6]);
        end
      
        xlabel('Data Points')
        ylabel('Volts')
    end    
    
        
end

close all
%  clearvars -except Flight_Data Raw_Data GPS Actuator EKF






