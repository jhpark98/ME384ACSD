% Project name: Reaction Wheel Stabilized Stick
% Bachelor thesis in mechatronics at KTH, spring 2019
% Bill Lavebratt, Pontus Gräsberg
% Description of code:
% Matlab program for simulation of the system. Needs the function RK4systk.m
% It predicts how the system will behave when implementing a State space controller

%% Set environment
clear all; close all; clc;

%% Define parameters

m=1.75; % 1.734; % Mass of the system, from cad model
R=0.14; % Distance from interface between stick and ground to the center of mass, from cad model
I= 350/100^2; % Moment of inertia of the system, at the interface between stick and ground, from Cad model
Ihh=10.196/100^2; % Moment of inertia of the reaction wheel, from Cad model
Im=110/(1000*100^2); % Moment of inertia of the rotor in motor, from datasheet of motor
Ih=Ihh+Im; % Combined moment of inertia of the reaction wheel and motor
g=9.82; % Gravitational acceleration
k2fi=5.84/100; % Torque constant of motor, from datasheet of motor
Ra=4.2; % Terminal resistance, from datasheet of motor State space model
A=[0 1 0;
m*g*R/I 0 k2fi^2/(I*Ra);
0 0 -k2fi^2/(Ih*Ra)];
B=[0; -k2fi/(I*Ra); k2fi/(Ih*Ra)];
C=[1 0 0];

% State space model when imlpementing the integrator
Any=[A zeros(size(A,1),1);-C 0];
Bny=[B;0];
Cny=[C 0];
ref=0; % Set point
D=[zeros(size(A,1),1);1];
Lny = place(Any, Bny, [-2,0,-1+0.5i -1-0.5i]); % Gives the feedback gain constants in L as a function of the poles of the system.
Amax=[A zeros(size(A,1),1); zeros(1,4)]; % Is used when desired voltage is larger than the maximum allowed.
Umax=24; % Maximum voltage

%% Solve for the states in the SS
% System of differential equations of the system with controller and anti windup, accounts for Umax.
f=@(tvec,yvec) ((((Any-Bny*Lny)*yvec' +ref*D)*(abs(- yvec(end,:)*Lny')<=Umax))+ ...
    (Amax*yvec'+Bny*Umax*abs(-yvec(end,:)*Lny')/ (-yvec(end,:)*Lny'))*(abs(-yvec(end,:)*Lny')>Umax)')';
y0=[0.125 0 0 0]; % Initial conditions of the states [Agle, Angular velocity, Angular velocity of recation wheel, Integraion of the error]
tspan=[0 10]; % Simulation spans over this time.
h=0.001; % Step length in ODE solver.
[tv,yv]=RK4systk(f,tspan,y0,h); % Solution of differential equation with implementation of RK4, See code "RK4systk.m "


%% Figure for states
figure(1)
subplot(2,2,1)
plot(tv,yv(:,1))
grid on, title('Angle of pendulum \theta'), xlabel('t [s]'),
ylabel('rad');

subplot(2,2,2)
plot(tv,yv(:,2))
grid on, title('Angular velocity of pendulum'), xlabel('t [s]'),
ylabel('rad/s');

subplot(2,2,4)
plot(tv,yv(:,4))
grid on, title('Integral(ref-angle)'), xlabel('t [s]');

subplot(2,2,3)
plot(tv,yv(:,3))
grid on, title('Angular velocity of reaction wheel'), xlabel('t [s]'),
ylabel('rad/s');

% The voltage that was sent to the motor
U=-yv*Lny';
for ii=1:length(U)
    if abs(U(ii))>Umax
        U(ii)=Umax*U(ii)/abs(U(ii));
    end
end

%% Figure for voltage and torque
M=@(w,Ua) Ua*k2fi/Ra-(k2fi)^2/Ra*w; % Equation of the motor
Mmotor=M(yv(:,3),U); % The torque the motor gave

figure(2)
subplot(2,1,1)
plot(tv,U)
grid on, title('Voltage'), xlabel('t [s]'), ylabel('V');

subplot(2,1,2)
plot(tv,Mmotor)
grid on, title('Torque from motor'), xlabel('t [s]'), ylabel('Nm');




