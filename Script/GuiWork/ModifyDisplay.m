%modify MasterAH

function ModifyMasterAH(theta,phi,TAS,alt)

global MasterAH

%MasterAH speed
	if(TAS<1)
		set(MasterAH.speed,'String',['  ' num2str(round(TAS))])
	else
	   set(MasterAH.speed,'String',{num2str(round(TAS))})
    end
    
 %MasterAH Altitude
    if(alt<1)
        set(MasterAH.altitude,'String',['  ' num2str(round(alt))])
    else
        set(MasterAH.altitude,'String',[num2str(round(alt))])
    end
    
    %matrix to rotate MasterAH according to the roll
    Mrot=[cos(phi)	-sin(phi);
	   sin(phi)	cos(phi)];
   

   
   %MasterAH angle with arrow
   Pos=Mrot*[MasterAH.refarrow];
   set(MasterAH.arrow,'Xdata',Pos(1,:))
   set(MasterAH.arrow,'Ydata',Pos(2,:))
   clear Pos Mrot
   
   %MasterAH rotated and sky
   %La land et le ciel
if(phi<pi/2 & phi>-pi/2)==1
   set(MasterAH.land,'Xdata',MasterAH.width/2*[1 1 -1 -1])
   set(MasterAH.land,'Ydata',[MasterAH.width/2*tan(phi)-(theta*180/pi)/41.5*MasterAH.height/2 -MasterAH.height/2 -MasterAH.height/2 -MasterAH.width/2*tan(phi)-(theta*180/pi)/41.5*MasterAH.height/2])
elseif(phi==-pi/2)
   set(MasterAH.land,'Xdata',MasterAH.width/2*[0 -1 -1 0])
   set(MasterAH.land,'Ydata',MasterAH.height/2*[-1 -1 1 1])
elseif(phi==pi/2)
   set(MasterAH.land,'Xdata',MasterAH.width/2*[0 1 1 0])
   set(MasterAH.land,'Ydata',MasterAH.height/2*[1 1 -1 -1])
else	
	set(MasterAH.land,'Xdata',MasterAH.width/2*[1 1 -1 -1])
	set(MasterAH.land,'Ydata',[MasterAH.width/2*tan(phi)-(theta*180/pi)/41.5*MasterAH.height/2 MasterAH.height/2 MasterAH.height/2 -MasterAH.width/2*tan(phi)-(theta*180/pi)/41.5*MasterAH.height/2])
end
   

    
end