fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');
camera=sim.getObject('/arucoMarker/Vision_sensor');
sim.startSimulation();
while true
[imagevrep,resolution]=sim.getVisionSensorImg(camera,1);
resolution=cell2mat(resolution);
imagevrep=reshape(imagevrep,resolution(1),resolution(2),[]);
center=getaruco3(imagevrep,3)
[ny,nx]=size(imagevrep);
imshow(imagevrep);
hold on
C= round([nx ny]/2);
plot(C(1),C(2),'*b');
plot(center(1),center(2),'*r');
double(C(1)-center(1))
double(C(2)-center(2))
sim.setObjectPosition(targetObj,targetObj,{0.0001*double(C(1)-center(1)) 0.0001*double(C(2)-center(2)) 0});
pause(0.1);
end
sim.stopSimulation();