fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');
%Handles
targetObj=sim.getObject('/target');
d=sim.getObject('/Quadcopter/base');
heli=sim.getObject('/Quadcopter');
sim.setObjectOrientation(targetObj,sim.handle_world,{0 0 0});

kp=2;
kd=10;
laste=0;

vparam=-2;
prevy=0;
prevRollE=0;
prevx=0;
prevPitchE=0;
prevEuler=0;

pos=sim.getObjectPosition(d,sim.handle_world);

client.setStepping(true);
sim.startSimulation();
r=0.5;
while sim.getSimulationTime()==0 || sim.getSimulationState()==sim.simulation_stopped
    pause(0.01);
    client.step();
end

while true

    t=sim.getSimulationTime();
    sim.setObjectPosition(targetObj,sim.handle_world,{r*cos(0.1*t) r*sin(0.1*t)   1});
    
    
    %Altitude Control
    targetPos=sim.getObjectPosition(targetObj,sim.handle_world);
    targetPos=cell2mat(targetPos(3));
    p=sim.getObjectPosition(d,sim.handle_world);
    pos=p(3);
    pos=cell2mat(pos);
    vel=sim.getVelocity(heli);
    l=cell2mat(vel(1,3));
    e=(targetPos-pos);
    angvel=7.02+kp*e+kd*(e-laste)+vparam*l;
    laste=e;
    
    tar=sim.getObjectPosition(d,targetObj);
    m=sim.getObjectOrientation(d,targetObj);
    
    %roll
    roll=0.03*(cell2mat(tar(2))+6.5*(cell2mat(tar(2))-prevy));
    prevy=cell2mat(tar(2));
    rollE=roll-cell2mat(m(1));
    rollCof=2*rollE+7*(rollE-prevRollE)+0.1*cell2mat(vel(2));
    prevRollE=rollE;
    cell2mat(m(1));

    %pitch
    pitch=-0.03*(cell2mat(tar(1))+6.5*(cell2mat(tar(1))-prevx));
    prevx=cell2mat(tar(1));
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
