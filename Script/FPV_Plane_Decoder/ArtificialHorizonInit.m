close all
clc 

global MasterAH

figure(1)
hold on
set(gca,'Xtick',[])
set(gca,'Ytick',[])
set(gca,'Position',[0.05 0 0.90 1])



MasterAH.height=1;
MasterAH.width=1;

r1=3*MasterAH.width/10;
r2=r1+MasterAH.width/30;
r3=r1+MasterAH.width/20;

MasterAH.noPlate=r1*180/(pi*25);

axis([-MasterAH.width/2 MasterAH.width/2 -MasterAH.height/2 MasterAH.height/2])

MasterAH.sky=patch([-1;-1;1;1]*MasterAH.width/2,[-1;1;1;-1]*MasterAH.width/2,[0.3 0.6 1]);

MasterAH.land=patch([-1;-1;1;1]*MasterAH.width/2,[0;-1;-1;0]*MasterAH.width/2,[0.8 0.6 0.4]);

%setting up pitch display
smallBar=(1/2)*MasterAH.width/10;
averageBar=MasterAH.width/10;
largeBar=2*MasterAH.width/10;

RefLandX=[smallBar averageBar smallBar largeBar averageBar averageBar]/2;
RefLandX=[-RefLandX;RefLandX];
RefLandX=[RefLandX RefLandX];

RefLandY=[2.5:2.5:10 15 20]*MasterAH.noPlate*pi/180;
RefLandY=[RefLandY;RefLandY];
RefLandY=[RefLandY -RefLandY];
MasterAH.PitchBar=line(RefLandX,RefLandY);
set(MasterAH.PitchBar,'Color','w','LineWidth',2)

TextX=[1.1 1.1 -1.45 -1.45]*largeBar/2;
TextY=[1 -1 1 -1]*10*pi/180*MasterAH.noPlate;
TextT={'10';'10';'10';'10'};
MasterAH.PitchBarText=text(TextX,TextY,TextT);
set(MasterAH.PitchBarText,'Color','w')

clear smallBar averageBar largeBar RefLandX RefLandY TextX TextY TextT

%Setting up heading display
pinkX=[r1*cos(pi/6) r1*cos(pi/4) r1*cos(pi/3) r1*cos(7*pi/18) r1*cos(4*pi/9);
		 r3*cos(pi/6) r2*cos(pi/4) r3*cos(pi/3) r2*cos(7*pi/18) r2*cos(4*pi/9)];
pinkY=[r1*sin(pi/6) r1*sin(pi/4) r1*sin(pi/3) r1*sin(7*pi/18) r1*sin(4*pi/9);
		 r3*sin(pi/6) r2*sin(pi/4) r3*sin(pi/3) r2*sin(7*pi/18) r2*sin(4*pi/9)];
     
pinkX=[pinkX [0;0] -pinkX];
pinkY=[pinkY [r1;r3] pinkY];

MasterAH.rose=line(pinkX,pinkY);
set(MasterAH.rose,'Color','w','LineWidth',2)

largeA=(1/3)*(MasterAH.width/10);
MasterAH.arrow=patch([0 1 -1]*largeA/2,r1-largeA*sin(60*pi/180)*[0 1 1],'w');
set(MasterAH.arrow,'EdgeColor','w')
MasterAH.refarrow=[get(MasterAH.arrow,'Xdata')';get(MasterAH.arrow,'Ydata')'];
clear pinkX pinkY r1 r2 r3 largeA

%Box for airspeed
MasterAH.speedbox=patch([0;MasterAH.width/10;MasterAH.width/10;0]-MasterAH.width/2,[1;1;-1;-1]*MasterAH.width/40,'k');
MasterAH.speed=text(-MasterAH.width/2+5*MasterAH.width/1000,-25*MasterAH.width/10000,'000');
set(MasterAH.speed,'color','w','FontSize',18)

%Box for Altitude
MasterAH.altitudebox=patch(-[0;1.6*MasterAH.width/10;1.6*MasterAH.width/10;0]+MasterAH.width/2,[1;1;-1;-1]*MasterAH.width/40,'k');
MasterAH.altitude=text(-1.6*MasterAH.width/10+MasterAH.width/2+5*MasterAH.width/1000,-25*MasterAH.width/10000,'00000');
set(MasterAH.altitude,'Color','w','Fontsize',18)


%Aircraft Wing
MasterAH.rightwing=line([MasterAH.width/10 MasterAH.width/10 2*MasterAH.width/10],[-0.5*MasterAH.height/10 0 0]);
MasterAH.leftwing=line([-MasterAH.width/10 -MasterAH.width/10 -2*MasterAH.width/10],[-0.5*MasterAH.height/10 0 0]);
MasterAH.cockpit=line([-0.1*MasterAH.width/10 0.1*MasterAH.width/10],[0 0]);

set(MasterAH.rightwing,'LineWidth',4,'Color','k')
set(MasterAH.leftwing,'LineWidth',4,'Color','k')
set(MasterAH.cockpit,'LineWidth',4,'Color','k')





