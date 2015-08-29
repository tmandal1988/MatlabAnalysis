function ReadAndModifyDisplay(serialObject,handles)

persistent RawDataBuf;
persistent g;
persistent cdeg2rad;
persistent Rot;
persistent atJacksonMill;
persistent ALPHAQ;
persistent ALPHA;
persistent Q;
persistent R;
persistent H;
persistent P;
persistent EKFstates;
persistent preCounter;
persistent preMS;

if isempty(RawDataBuf)
    RawDataBuf=zeros(1000,1);
end

dt=0.2;
persistent writePointer;
persistent readPointer;
persistent dataPointerW;
persistent dataPointerR;

if isempty(writePointer)
    writePointer=0;
    datPointerR=0;
end

if isempty(readPointer)
    readPointer=0;
    dataPointerW=0;
end

if isempty(g)
    g=9.8;
end

if isempty(cdeg2rad)
    cdeg2rad=0.0175;
end

if isempty(atJacksonMill)
    atJacksonMill=1;
end

if isempty(Rot)
    if(atJacksonMill==1)
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
    
    ALPHAQ=1;
    Q=diag([0,0,0,2.3453e-6,3.2589e-6,2.8023e-5,3.6204e-7,3.6204e-7,6.3775e-7])*ALPHAQ;
    
    ALPHA=2;
    R=diag([ALPHA*.1023,ALPHA*.5060,ALPHA*.5120,ALPHA*3.89e-4,ALPHA*1.113e-4,ALPHA*9.84e-4]);
    
    P=eye(9);

    H=zeros(6,9);
    H(1,1)=1;H(2,2)=1;H(3,3)=1;H(4,4)=1;H(5,5)=1;H(6,6)=1;
    
    EKFstates=zeros(9,1);
    
    preCounter=0;
    preMS=0;

end


n=serialObject.BytesAvailable;
if(n~=0)
    [RecBuf, count]=fread(serialObject,n,'uint8');
    
    %count
%   
    for i=1:count
        writePointer=mod(dataPointerW,1000);
        RawDataBuf(writePointer+1)=RecBuf(i);
        dataPointerW=dataPointerW+1;
    end
 
%     
%     if((writePointer+count-1)>1000)          
%         RawDataBuf(writePointer:1000)=RecBuf(1:1000-writePointer+1);
%         
%         RawDataBuf(1:(writePointer+count-1-1000))=RecBuf(1000-writePointer+2:end);
%         writePointer=(writePointer+count-1000);
%     else
%         RawDataBuf(writePointer:writePointer+count-1)=RecBuf;
%         writePointer=mod(writePointer+count,1000);
%     end
    
    
    
    flushinput(serialObject);
    
   
  
end



%dataPointerW-readPointer

if(dataPointerW-readPointer >= 9)

    %uint8(RawDataBuf(mod(readPointer,1000)+1)),uint8(RawDataBuf(mod((readPointer+1),1000)+1)),uint8(RawDataBuf(mod((readPointer+2),1000)+1))
    if(uint8(RawDataBuf(mod(readPointer,1000)+1))==170 && uint8(RawDataBuf(mod((readPointer+1),1000)+1))==171 && uint8(RawDataBuf(mod((readPointer+2),1000)+1))==187)
%        Flight_Data.R=single(typecast(uint8(RawDataBuf(mod((readPointer+4),1000):-1:mod((readPointer+3),1000))),'int16'))*0.02*cdeg2rad;
%        Flight_Data.Q=single(typecast(uint8(RawDataBuf(mod((readPointer+6),1000):-1:mod((readPointer+5),1000))),'int16'))*0.02*cdeg2rad;
%        Flight_Data.Ax=single(typecast(uint8(RawDataBuf(mod((readPointer+8),1000):-1:mod((readPointer+7),1000))),'int16'))*0.00025*g;
%        Flight_Data.Ay=single(typecast(uint8(RawDataBuf(mod((readPointer+10),1000):-1:mod((readPointer+9),1000))),'int16'))*0.00025*g;
%        Flight_Data.Az=single(typecast(uint8(RawDataBuf(mod((readPointer+12),1000):-1:mod((readPointer+11),1000))),'int16'))*0.00025*g;
%        Flight_Data.P=single(typecast(uint8(RawDataBuf(mod((readPointer+14),1000):-1:mod((readPointer+13),1000))),'int16'))*0.02*cdeg2rad;
%        
%        
% 
%        Flight_Data.NBcounter=RawDataBuf(mod((readPointer+139),1000));
%        Flight_Data.DataCounter=RawDataBuf(mod((readPointer+138),1000));
       
       GCSdata(1)=RawDataBuf(mod((readPointer+3),1000)+1);
       GCSdata(2)=RawDataBuf(mod((readPointer+4),1000)+1);
       GCSdata(3)=RawDataBuf(mod((readPointer+5),1000)+1);
       GCSdata(4)=RawDataBuf(mod((readPointer+6),1000)+1);
       GCSdata(5)=RawDataBuf(mod((readPointer+7),1000)+1);
       GCSdata(6)=RawDataBuf(mod((readPointer+8),1000)+1);
       
      %handles.GCSdata(2)
%         display('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
             %fprintf('%d,%d,%d,%d,%d,%d\n',uint8(GCSdata(1)),uint8(GCSdata(2)),uint8(GCSdata(3)),uint8(GCSdata(4)),uint8(GCSdata(5)),uint8(GCSdata(6)));
%             fprintf('%d,%d,%d,%d,%d,%d\n',uint8(handles.GCSdata(1)),uint8(handles.GCSdata(2)),uint8(handles.GCSdata(3)),uint8(handles.GCSdata(4)),uint8(handles.GCSdata(5)),uint8(handles.GCSdata(6)));
%         display('*******************************')
%         fprintf('%d,%d,%d,%d,%d,%d\n',int8(handles.GCSdata(1)),int8(handles.GCSdata(2)),int8(handles.GCSdata(3)),int8(handles.GCSdata(4)),int8(handles.GCSdata(5)),int8(handles.GCSdata(6)));
%         display('-------------------------')
       
       if(handles.GCSdata(1)==uint8(GCSdata(1)) && handles.GCSdata(2)==uint8(GCSdata(2)) && handles.GCSdata(3)==uint8(GCSdata(3)) && handles.GCSdata(4)==uint8(GCSdata(4)) && handles.GCSdata(5)==uint8(GCSdata(5)) && handles.GCSdata(6)==uint8(GCSdata(6)))
      
            DelayStr=num2str(single(typecast(uint8([GCSdata(1) GCSdata(2)]),'int16'))/10);
            LimitStr=num2str(single(typecast(uint8([GCSdata(3) GCSdata(4)]),'int16'))/100);
            TrimStr=num2str(single(typecast(uint8([GCSdata(5) GCSdata(6)]),'int16'))/100);
            
            loopbackdisplay=strcat('Delay(ms)=',DelayStr,',','Rate Limit(%)=',LimitStr,',','Trim (deg)=',TrimStr);
            
            set(handles.LoopBackDisplay,'String',loopbackdisplay);
       end
       
       


       


       
%        GPS.x=typecast(uint8(RawDataBuf(mod((readPointer+34),1000):-1:mod((readPointer+27),1000))),'double');
%        GPS.y=typecast(uint8(RawDataBuf(mod((readPointer+42),1000):-1:mod((readPointer+35),1000))),'double');
%        GPS.z=typecast(uint8(RawDataBuf(mod((readPointer+50),1000):-1:mod((readPointer+43),1000))),'double');
%         
%        GPS.vx=typecast(uint8(RawDataBuf(mod((readPointer+58),1000):-1:mod((readPointer+51),1000))),'double');
%        GPS.vy=typecast(uint8(RawDataBuf(mod((readPointer+66),1000):-1:mod((readPointer+59),1000))),'double');
%        GPS.vz=typecast(uint8(RawDataBuf(mod((readPointer+74),1000):-1:mod((readPointer+67),1000))),'double');
%          
%        GPS.week=typecast(uint8(RawDataBuf(mod((readPointer+97),1000):-1:mod((readPointer+96),1000))),'uint16');
%        GPS.ms=typecast(uint8(RawDataBuf(mod((readPointer+98),1000):mod((readPointer+101),1000))),'uint32');
%        
%        
%         localGPSP=[1 0 0 0;0 -1 0 0;0 0 -1 0]*Rot*[GPS.x;GPS.y;GPS.z;1]+[-157.2;-108.2;222];
%         localGPSV=[1 0 0 0;0 -1 0 0;0 0 -1 0]*Rot*[GPS.vx;GPS.vy;GPS.vz;0];
%         
%         localGPS=[localGPSP' localGPSV'];
%         
%        if(Flight_Data.NBcounter-preCounter==1 || Flight_Data.NBcounter-preCounter==-255)
%             
%         
%         s_phi=sin(EKFstates(7));
%         c_phi=cos(EKFstates(7));
% 
%         s_theta=sin(EKFstates(8));
%         c_theta=cos(EKFstates(8));
%         sec_theta=sec(EKFstates(8));
%         t_theta=tan(EKFstates(8));
% 
%         s_psi=sin(EKFstates(9));
%         c_psi=cos(EKFstates(9));
%     
%         EKFstates(1)=EKFstates(1)+dt*EKFstates(4);
%         EKFstates(2)=EKFstates(2)+dt*EKFstates(5);
%         EKFstates(3)=EKFstates(3)+dt*EKFstates(6);
%             
%         EKFstates(4)=EKFstates(4) + dt*(Flight_Data.Ax*c_theta*c_psi + Flight_Data.Ay*(-c_phi*s_psi + s_phi*s_theta*c_psi) + Flight_Data.Az*(s_phi*s_psi + c_phi*s_theta*c_psi));
%         EKFstates(5)=EKFstates(5) + dt*(Flight_Data.Ax*c_theta*s_psi + Flight_Data.Ay*(c_phi*c_psi + s_phi*s_theta*s_psi) + Flight_Data.Az*(-s_phi*c_psi + c_phi*s_theta*s_psi));
%         EKFstates(6)=EKFstates(6) + dt*(-Flight_Data.Ax*s_theta + (Flight_Data.Ay*s_phi + Flight_Data.Az*c_phi)*c_theta)+g*dt;
%     
%         EKFstates(7)=EKFstates(7) + dt*(Flight_Data.P+(Flight_Data.Q*s_phi + Flight_Data.R*c_phi)*t_theta);
%         EKFstates(8)=EKFstates(8) + dt*(Flight_Data.Q*c_phi - Flight_Data.R*s_phi);
%         EKFstates(9)=EKFstates(9) + dt*(Flight_Data.Q*s_phi + Flight_Data.R*c_phi)*sec_theta;
%     
%         A=[1 0 0 dt 0 0 0 0 0;
%         0 1 0 0 dt 0 0 0 0;
%         0 0 1 0 0 dt 0 0 0;
%         0 0 0 1 0 0 dt*(Flight_Data.Ay*(s_phi*s_psi + c_phi*s_theta*c_psi) + Flight_Data.Az*(c_phi*s_psi - s_phi*s_theta*c_psi)) dt*(Flight_Data.Ax*(-s_theta*c_psi) + Flight_Data.Ay*(c_theta*c_psi*s_phi) + Flight_Data.Az*(c_phi*c_theta*c_psi)) dt*(Flight_Data.Ax*(-c_theta*s_psi) - Flight_Data.Ay*(c_phi*c_psi+s_phi*s_theta*s_psi)+ Flight_Data.Az*(s_phi*c_psi-c_phi*s_theta*s_psi));
%         0 0 0 0 1 0 dt*(Flight_Data.Ay*(-s_phi*c_psi + c_phi*s_theta*s_psi) - Flight_Data.Az*(c_phi*c_psi + s_phi*s_theta*s_psi)) dt*(Flight_Data.Ax*(-s_theta*s_psi) + Flight_Data.Ay*(c_theta*s_phi*s_psi) + Flight_Data.Az*(c_phi*c_theta*s_psi)) dt*(Flight_Data.Ax*(c_theta*c_psi) + Flight_Data.Ay*(-c_phi*s_psi + s_phi*s_theta*c_psi) + Flight_Data.Az*(s_phi*s_psi + c_phi*s_theta*c_psi));
%         0 0 0 0 0 1 dt*(Flight_Data.Ay*(c_phi*c_theta) - Flight_Data.Az*(s_phi*c_theta))                                          -dt*(Flight_Data.Ax*(c_theta) + Flight_Data.Ay*(s_theta*s_phi) + Flight_Data.Az*(c_phi*s_theta))                                                                                         0                                                                                         ;
%         0 0 0 0 0 0 1+dt*t_theta*(c_phi*Flight_Data.Q- s_phi*Flight_Data.R)                                        dt*sec_theta*sec_theta*(s_phi*Flight_Data.Q + c_phi*Flight_Data.R)                                                                                                               0                                                                                        ;
%         0 0 0 0 0 0 -dt*(s_phi*Flight_Data.Q + c_phi*Flight_Data.R)                                                 1                                                                                                                                                                                                           0                                                                                        ;
%         0 0 0 0 0 0 dt*sec_theta*(c_phi*Flight_Data.Q - s_phi*Flight_Data.R)                                         dt*sec_theta*t_theta*(s_phi*Flight_Data.Q + c_phi*Flight_Data.R)                                                                                                                 1                                                                                        ];                
%     
%     
%     
%         P=A*P*A' + Q;
%         end
%     
%     preCounter=Flight_Data.NBcounter;
%     
%     if(GPS.ms-preMS==20)    
% 
%         K=P*H'/(H*P*H'+R);
%     
%         EKFstates=EKFstates + K*([localGPS(1,1);localGPS(1,2);localGPS(1,3);localGPS(1,4);localGPS(1,5);localGPS(1,6)]-EKFstates(1:6));     
%     
%     
%     
%         P=(eye(9)-K*H)*P; 
%     end
%     
%     preMS=GPS.ms;
%     
%         
%     if(EKFstates(7)>2*pi)
%         EKFstates(7)=EKFstates(7)-2*pi;
%     elseif(EKFstates(7)<-2*pi)
%         EKFstates(7)=EKFstates(7)+2*pi;
%     end
%     
%     if(EKFstates(8)>pi/2)
%         EKFstates(8)=EKFstates(8)-pi/2;
%     elseif(EKFstates(8)<-pi/2)
%         EKFstates(8)=EKFstates(8)+pi/2;
%     end
%     
%     if(EKFstates(9)>2*pi)
%         EKFstates(9)=EKFstates(9)-2*pi;
%     elseif(EKFstates(9)<0)
%         EKFstates(9)=EKFstates(9)+2*pi;
%     end
%        
%       ModifyDisplay(EKFstates(8),EKFstates(7),0,0);
%        -atan2(Flight_Data.Ax,1);
%        
       readPointer=readPointer+10;
     else
        readPointer=readPointer+1;
     end
%      
% end
%       
    end
  
        
end