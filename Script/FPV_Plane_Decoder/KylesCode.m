phi = 0;
theta = 0;
psi = 0;
dt = .01;
p = Flight_Data.P*pi/180;
r = Flight_Data.Q*pi/180;
q = Flight_Data.R*pi/180;
p_offset = mean(p(1:4000));
q_offset = mean(q(1:4000));
r_offset = mean(r(1:4000));
for ii = 2:100000;
    phi(ii) = phi(ii-1)+dt*(p(ii)-p_offset+(q(ii)-q_offset)*sin(phi(ii-1))*tan(theta(ii-1))+(r(ii)-r_offset)*cos(phi(ii-1))*tan(theta(ii-1)));
	theta(ii) = theta(ii-1)+dt*((q(ii)-q_offset)*cos(phi(ii-1))-(r(ii)-r_offset)*sin(phi(ii-1)));
	psi(ii) = psi(ii-1)+dt*((q(ii)-q_offset)*sin(phi(ii-1))+(r(ii)-r_offset)*cos(phi(ii-1)))*(1/cos(theta(ii-1)));
end
figure(1)
plot(phi*180/pi)
figure(2)
plot(theta*180/pi)
figure(3)
plot(psi*180/pi)