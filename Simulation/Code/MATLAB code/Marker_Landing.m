fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');
%Handles
targetObj=sim.getObject('/target');
d=sim.getObject('/Quadcopter/base');
heli=sim.getObject('/Quadcopter');
camera=sim.getObject('/Quadcopter/Vision_sensor');
sim.setObjectOrientation(targetObj,sim.handle_world,{0 0 0});
sim.setObjectPosition(heli,sim.handle_world,{0 0 1});
sim.setObjectPosition(targetObj,sim.handle_world,{0 0 1});
kp=2;
kd=10;
laste=0;

vparam=-2;
prevy=0;
prevRollE=0;
prevx=0;
prevPitchE=0;
prevEuler=0;
tarO=[0 0 1];
pos=sim.getObjectPosition(d,sim.handle_world);
targetPos=1;
client.setStepping(true);
sim.startSimulation();
r=0.5;
while sim.getSimulationTime()==0 || sim.getSimulationState()==sim.simulation_stopped
    pause(0.01);
    client.step();
end

while true

    t=sim.getSimulationTime();

    [imagevrep,resolution]=sim.getVisionSensorImg(camera,1);
    resolution=cell2mat(resolution);
    imagevrep=reshape(imagevrep,resolution(1),resolution(2),[]);
    center=getaruco3(imagevrep,3)
    [ny,nx]=size(imagevrep);
    C= round([nx ny]/2);
    dif1=double(C(1)-center(1));
    dif2=double(C(2)-center(2));
    sim.setObjectPosition(targetObj,targetObj,{-0.0001*dif1 0.0001*dif2 0});
    %Altitude Control
    
    if dif1==dif2 && dif2==0 && targetPos>=0.13 
        targetPos=targetPos-0.03;
    end
    p=sim.getObjectPosition(d,sim.handle_world);
    pos=p(3);
    pos=cell2mat(pos);
    vel=sim.getVelocity(heli);
    l=cell2mat(vel(1,3));
    e=(targetPos-pos);
    angvel=7.02+kp*e+kd*(e-laste)+vparam*l;
    laste=e;
    object=sim.getObjectPosition(d,sim.handle_world);
    
    tar=[0.003*dif1 -0.003*dif2 0];
    %tar=[cell2mat(object(1))-tarO(1) cell2mat(object(1))-tarO(2) cell2mat(object(1))-tarO(3)];

    %tar=sim.getObjectPosition(d,targetObj)
    m=sim.getObjectOrientation(d,sim.handle_world);
    
    %roll
    roll=0.03*(tar(2)+6.5*(tar(2)-prevy));
    prevy=tar(2);
    rollE=roll-cell2mat(m(1));
    rollCof=2*rollE+7*(rollE-prevRollE)+0.1*cell2mat(vel(2));
    prevRollE=rollE;
    cell2mat(m(1));

    %pitch
    pitch=-0.03*(tar(1)+6.5*(tar(1)-prevx));
    prevx=tar(1);
    pitchE=pitch-cell2mat(m(2));
    pitchCof=2*pitchE+7*(pitchE-prevPitchE)-0.1*cell2mat(vel(1));
    prevPitchE=pitchE;

    %yaw
    rotCorr=cell2mat(m(3))*0.1+2*(cell2mat(m(3))-prevEuler);
    prevEuler=cell2mat(m(3));

    angvel1=angvel-pitchCof+rollCof+rotCorr;
    angvel2=angvel+pitchCof+rollCof-rotCorr;
    angvel3=angvel+pitchCof-rollCof+rotCorr;
    angvel4=angvel-pitchCof-rollCof-rotCorr;
    
    sim.callScriptFunction('setVel@/Quadcopter',sim.scripttype_childscript,angvel1,angvel2,angvel3,angvel4);
    
    client.step();  % triggers next simulation step
end

sim.stopSimulation();

