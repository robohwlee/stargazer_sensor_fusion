%% Car Object
car.coord = [0; 0];
car.Radius = 1;
car.shape = [sin(linspace(0,2*pi,100)); cos(linspace(0,2*pi,100))]*car.Radius;
car.color = [0.2 0.2 0.8; 0.8 0.8 0.1];

%% Rays
allRays.num = 500;
allRays.radius = 10;
allRays.ab = [2 2];
t = linspace(0,2*pi*(1-1/allRays.num),allRays.num);
x11 = zeros(allRays.num,3);
y11 = zeros(allRays.num,3);
x11(:,1) = sin(t)*allRays.radius*allRays.ab(1)+car.coord(1);
y11(:,1) = cos(t)*allRays.radius*allRays.ab(2)+car.coord(2);
x11(:,2) = sin(t)*car.Radius+car.coord(1);
y11(:,2) = cos(t)*car.Radius+car.coord(2);
x11(:,3) = NaN;
y11(:,3) = NaN;
x12 = reshape(x11',allRays.num*3,1);
y12 = reshape(y11',allRays.num*3,1);
allRays.plot = plot(x12,y12,'r','ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);