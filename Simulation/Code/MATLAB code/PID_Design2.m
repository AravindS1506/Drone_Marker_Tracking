fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');

%Handles
targetObj=sim.getObject('/target');
sim.setObjectPosition(targetObj,sim.handle_world,{0 0 1});
d=sim.getObject('/Quadcopter/base');
heli=sim.getObject('/Quadcopter');

kp=2;
kd=10;
laste=0;
vparam=-2;
prevEuler=0;
    cumul=0;
    lastE=0;
    pAlphaE=0;
    pBetaE=0;
    psp2=0;
    psp1=0;
pos=sim.getObjectPosition(d,sim.handle_world);

    path=sim.getObject('/Path');
    velocity=0.1;
    posAlongPath=0;
    previousSimulationTime=0;


client.setStepping(true);
sim.startSimulation();
pause(1)
while sim.getSimulationTime()==0 || sim.getSimulationState()==sim.simulation_stopped
    pause(0.01);
    client.step();
end
      %[pathPositions,pathQuaternions,pathLengths,totalLength]=sim.callScriptFunction('pathval@/target',sim.scripttype_childscript)
while true

    %t=sim.getSimulationTime();
    %posAlongPath=posAlongPath+velocity*(t-previousSimulationTime);
    %posAlongPath=mod(posAlongPath,totalLength);
    %pospath=sim.getPathInterpolatedConfig(pathPositions,pathLengths,posAlongPath);
    %quat=sim.getPathInterpolatedConfig(pathQuaternions,pathLengths,posAlongPath,{},{2,2,2,2});
    %sim.setObjectPosition(targetObj,path,pospath);
    %sim.setObjectQuaternion(targetObj,path,quat);
    %previousSimulationTime=t;

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
    
    sp=sim.getObjectPosition(targetObj,d);
    m=sim.getObjectMatrix(d,sim.handle_world);
    vx={1,0,0};
    vx=sim.multiplyVector(m,vx);
    vy={0,1,0};
    vy=sim.multiplyVector(m,vy);
    alphaE=(cell2mat(vy(3)))
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE);
    betaE=cell2mat(vx(3));
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE);
    pAlphaE=alphaE;
    pBetaE=betaE;
    alphaCorr=alphaCorr+cell2mat(sp(2))*0.005+1*(cell2mat(sp(2))-psp2);
    betaCorr=betaCorr-cell2mat(sp(1))*0.005-1*(cell2mat(sp(1))-psp1);
    psp2=cell2mat(sp(2));
    psp1=cell2mat(sp(1));
    
    
    euler=sim.getObjectOrientation(d,targetObj);
    rotCorr=cell2mat(euler(3))*0.1+2*(cell2mat(euler(3))-prevEuler);
    prevEuler=cell2mat(euler(3));
    angvel1=angvel*(1-alphaCorr+betaCorr-rotCorr);
    angvel2=angvel*(1-alphaCorr-betaCorr+rotCorr);
    angvel3=angvel*(1+alphaCorr-betaCorr-rotCorr);
    angvel4=angvel*(1+alphaCorr+betaCorr+rotCorr);
    
    sim.callScriptFunction('setVel@/Quadcopter',sim.scripttype_childscript,angvel1,angvel2,angvel3,angvel4);
    
    client.step();  % triggers next simulation step
end

sim.stopSimulation();
