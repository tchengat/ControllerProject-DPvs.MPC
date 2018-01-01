function [p,slope,lat,lon,el] = gpxtoPandSlope(filename)
% This function converts the raw path data into usable position and slope. 
% It returns p- distance from initial point and slope- sin(theta) at p.

file=gpxread(filename);         % read the gps file 
points=numel(file.Latitude);    % extract number of discrete points by measuring latitude points

lat=file.Latitude;              % read latitudes
lon=file.Longitude;             % read longitudes
el=file.Elevation;              % read elevations
p=zeros(points,1);              % initialize position 

noise_slope=zeros(points,1);    % initialize raw slope array
for i=1:points-1
   [a,b]=distance(lat(i+1),lon(i+1),lat(i),lon(i)); % Assigns arclen in degrees and azimuthal to a and b to  
   deltaP=a*111120;                                 % Converts arclen into meters
   deltaH=el(i+1)-el(i);                            % Slope 
   if deltaP>1                                      % To avoid small deltaP blowing up the slope
        noise_slope(i)= deltaH/sqrt(deltaH^2+deltaP^2); % sin(theta)
   else
        noise_slope(i)=noise_slope(i-1); % Assign slope to the region with small deltaP
   end
        p(i+1)=p(i)+deltaP;             % Increment position
end
slope=medfilt1(noise_slope,3);          % Smoothen slope values

