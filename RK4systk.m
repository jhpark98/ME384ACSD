% Description of code:
% Matlab function for simulation of the system.
% Implementation of a Runge-Kutta 4th Order method for System of
% differential equations.
% The input is f: function of system of differential equations, tspan:
% start time and end time of simulation, y0: Initial conditions, h:
% steplength.
% The output is tv: the time vector, yv: matrix of the solution to the
% system of differential equations.

function [tv,yv]=RK4systk(f,tspan,y0,h)
    n=(tspan(2)-tspan(1))/h;
    tv=(tspan(1)+h*(0:n))';
    p=length(y0);
    yv=zeros(n+1,p);
    yv(1,:)=y0;
    for ii=1:n
        k1=feval(f,tv(ii),yv(ii,:));
        k2=feval(f,tv(ii)+h/2,yv(ii,:)+h/2*k1);
        k3=feval(f,tv(ii)+h/2,yv(ii,:)+h/2*k2);
        k4=feval(f,tv(ii)+h,yv(ii,:)+h*k3);
        yv(ii+1,:)=yv(ii,:)+h/6*(k1+2*k2+2*k3+k4);
    end
end