function [feas vOpt uOpt]= car_batch(N, v0,p_sampled,dp,vgrid,Fgrid,E_grid,profile)
%clear all;

%% Define parameters
%load TrainDataNew
g = 9.8; % gravitational acceleration in m/s^2
M = 2000; % mass of car in kg
A = 200; % rolling resistance coeffcient in N
B = 5.5; % drivetrain losses in Ns/m
C = 0.39; % aerodynamic drag coeffcient in N(s^2)/(m^2)
R=0.4; %radius of wheel in m
gr=6;  %gear ratio
n=4;   %number of motors

%% function arguments
vMin= 3; %m
vMax= 30; %m
%%
% Define state matrix(velocity)
v= sdpvar(1,N+1);
% Define decision variables
u = sdpvar(1,N);

% Define objective function and constraints
uMin = -880; % N
uMax = 880; % N

% Define anonymous function
computeVNext = @(v,u,p) v+dp/(v*M)*(-A-B*v-C*v^2-M*g*slope(p,profile)+u);
objective=0;
constraints=[];


efficiency = @(f,v) interp2(Fgrid,vgrid,E_grid,f,v);

%% run optimization
for i=1:N
    %objective= objective + ((u(i)+abs(u(i)))/2+(abs(u(i))-u(i)))*dp/efficiency(u(i),v(i));
   %objective= objective + ((u(i)+abs(u(i)))/2+(abs(u(i))-u(i)))*dp;
    objective= objective + u(i)*dp;
    
    constraints = [constraints vMin<=v(i)<=vMax uMin<=u(i)<=uMax v(i+1)==computeVNext(v(i),u(i),p_sampled(i))];
    sl(i)=slope(p_sampled(i),profile);
end
constraints = [constraints v(1)==v0];

% Set options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','fmincon','usex0',0,'cachesolvers', 1);

% Solve
sol = optimize(constraints, objective,options);

feas=sol.problem;
vOpt=value(v);
uOpt=value(u);
end


