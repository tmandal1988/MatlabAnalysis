% Name GPSLocation
atJacksonMill=1;

if atJacksonMill==1;
    AP=[8.2077e+05  -4.8886e+06    4.0005e+06];
else
     AP=[8.5636e+05  -4.8430e+06   4.0480e+06];
end
    Dis=6369948.151050708;
    %Lat=39.0933006;
    %Lon=-80.47024365;
    Lat=asin(AP(3)/Dis)*(180/pi);
    Lon=180/pi*asin(AP(2)/sqrt(AP(1)^2+AP(2)^2));
    R=inv(x2t([AP';1;0;(90-Lat)*pi/180;Lon*pi/180;0],'rpy'));
    I=eye(9);

