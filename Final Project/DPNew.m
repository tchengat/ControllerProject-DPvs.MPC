%% DP on Car Probelm 
clc; clear; close all; 
tic 
% Define parameters
load train_data_midterm
g = 9.8; % gravitational acceleration in m/s^2
M = 2000; % mass of car in kg
A = 200; % rolling resistance coeffcient in N
B = 5.5; % drivetrain losses in Ns/m
C = 0.39; % aerodynamic drag coeffcient in N(s^2)/(m^2) 
umax = 880; % max input force in N 
umin = -880; %minimum input force (think of this as 'max' breaking, thus negative acceleration and subsequently force) in N; 
minspeed = 3; %minimum velocity of car in m/s (~10 km/h); 
maxspeed = 30; %maximum velocity of car in m/s (~100 km/h); 
R=0.4; %radius of wheel in m
gr=6;  %gear ratio  
n=4;   %number of motors
delta_F= 1*gr/R*n; % grid distance force
delta_v= 10*R/gr*2*(pi/60); % grid distance velocity
Fgrid= delta_F:delta_F:88*delta_F;
vgrid= delta_v:delta_v:1000*delta_v;
E_grid=load('Motor_data.mat'); 
E_grid = E_grid.Motor_data;
j = 2; 
for i=1:size(E_grid,2)-1
    if i == 1; 
    NewEGrid(:,i) = E_grid(:,i); 
    end
    if rem(i,10) == 0
    Store = E_grid(:,i); 
    NewEGrid(:,j)=Store; 
    j = j+1; 
    end 
end    

E_grid = NewEGrid; 
%E_grid = [flipud(E_grid);E_grid(88,:);E_grid];
%Fgrid = [fliplr(Fgrid),0,-Fgrid];

%% Grid states and inputs

vpoints = 40;  
v_sampled = linspace(minspeed,maxspeed,vpoints); % grid from minspeed to maxspeed
upoints = 30; 
u_sampled = linspace(umin,umax,upoints); %grid from minimum to max force

%% Grid position
track_length = profile(end,1); % in meters
dp = 10; % sampling in space
p_sampled = 0:dp:track_length; %sampled train position
N_p = length(p_sampled);

%% Create Arrays to save cost and inputs
Jsave = cell(N_p,1); 
USave = cell(N_p,1); 
Jtogo = cell(N_p+1,1); 
Uopt = cell(N_p,1); 


%% Define anononymous functions 

computeVNext = @(v,u,p) v+dp/(v*M)*(-A-B*v-C*v^2-M*g*slope(p,profile)+u); 
efficiency = @(u,v) interpn(Fgrid,vgrid,E_grid,u,v); 
Jstage = @(u,v) (((u+abs(u))/5)+(abs(u)-u))*dp/efficiency(u,v);  
%Jstage = @(u,v) abs(u)*dp/efficiency(u,v);
%Jstage = @(u,v) u*dp/efficiency(abs(u),v); 
Jtogo{end} = @(x) x*0;  


%% Dynamic Programming 

for pIndex = N_p:-1:1
    J = nan(vpoints,1); 
    UGR = nan(vpoints,1); 
    for i = 1:vpoints 
            vt = v_sampled(i) ;     
            Jbest = inf; 
            Ubest = nan;
        for j=u_sampled 
            ut = j; 
            vnext = computeVNext(vt,ut,pIndex); 
            if vnext<minspeed || vnext>maxspeed
                continue; 
            end 
                 
            Jt = Jstage(ut,vt)+Jtogo{pIndex+1}(vnext); 
            
            if isnan(Jt)
                continue; 
            end 
            if Jt < Jbest 
                Jbest = Jt; 
                Ubest = ut; 
            end 
        end 
        J(i) = Jbest; 
        UGR(i) = Ubest; 
    end
    Jsave{pIndex} = J; 
    USave{pIndex} = UGR; 
    Jtogo{pIndex} = @(v) interpn(v_sampled,J,v,'linear'); 
    UOpt{pIndex} = @(v) interpn(v_sampled,UGR,v,'linear'); 
end

save('NegSlope20BrakingSmallMotorAshtoBerkEff.mat')

%% 
load('NegSlope20BrakingSmallMotorAshtoBerk.mat')
V0=10;

VSim = zeros(1,N_p+1); 
VSim(:,1) = V0; 
USim = zeros(1,N_p); 
JSim = zeros(1,N_p); 
sl = zeros(1,N_p); 

for t = 1:N_p-1
    USim(:,t) = [UOpt{t}(VSim(:,t))]; 
    if any(isnan(USim(:,t)))
        disp('infeasible')
    break; 
    end 
    VSim(:,t+1) = computeVNext(VSim(:,t),USim(:,t),p_sampled(t)); 
    JSim(t) = Jstage(USim(:,t),VSim(:,t)); 
    sl(t)=slope(p_sampled(t),profile);
end 

figure()
JSum = sum(JSim)
subplot(3,1,1)
plot(p_sampled,VSim(1:end-1))
xlabel('Distance (m)')
ylabel('Speed (m/s)')
title('Velocity Trace')
subplot(3,1,2)
plot(p_sampled,USim); 
xlabel('Distance (m)')
ylabel('Drive Force (kN)')
title('Optimal Input') 
subplot(3,1,3)
plot(p_sampled,sl)
xlabel('Distance (m)')
ylabel('Slope Change')
title('Road Grade') 
toc;


