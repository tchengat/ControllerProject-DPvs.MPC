T=gpxread('path.gpx');
states = geoshape(shaperead('usastatehi', 'UseGeoCoords', true));
latlim = [min(T.Latitude) max(T.Latitude)];
lonlim = [min(T.Longitude) max(T.Longitude)];
ocean = [0.7 0.8 1]; land = [0.9 0.9 0.8];
%% Overlay the GPS points on the map.
webmap('World Street Map', 'WrapAround', false)
colors = {'red'};
wmline(T, 'Color', colors)
wmlimits(latlim, lonlim)
%% Overlay the GPS points on the map.
figure
ax = usamap(latlim-[0.01,-.01], lonlim-[.01,-.01]);
setm(ax, 'FFaceColor', ocean)
geoshow(states,'FaceColor',land)
geoshow(T.Latitude,T.Longitude,'DisplayType','Point','Marker','.',...
    'MarkerSize',4,'MarkerEdgeColor',[0 0 1])
title('Trip Data')
xlabel('Bay Area')%%
