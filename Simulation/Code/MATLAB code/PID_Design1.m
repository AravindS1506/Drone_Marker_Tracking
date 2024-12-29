fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');

%Handles
propellerRespondable0=sim.getObject('/Quadcopter/propeller[0]/respondable');
propellerRespondable1=sim.getObject('/Quadcopter/propeller[1]/respondable');
propellerRespondable2=sim.getObject('/Quadcopter/propeller[2]/respondable');
propellerRespondable3=sim.getObject('/Quadcopter/propeller[3]/respondable');
propeller0=sim.getObjectParent(propellerRespondable0);
propeller1=sim.getObjectParent(propellerRespondable1);
propeller2=sim.getObjectParent(propellerRespondable2);
propeller3=sim.getObjectParent(propellerRespondable3);
joint0=sim.getObject('/Quadcopter/propeller[0]/joint');
joint1=sim.getObject('/Quadcopter/propeller[1]/joint');
joint2=sim.getObject('/Quadcopter/propeller[2]/joint');
joint3=sim.getObject('/Quadcopter/propeller[3]/joint');
targetObj=sim.getObject('/target');
sim.setObjectPosition(targetObj,sim.handle_world,{0 0 1});
d=sim.getObject('/Quadcopter/base');
heli=sim.getObject('/Quadcopter');

kp=2;
kd=1;
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

client.setStepping(true);
sim.startSimulation();

while true

    %Altitude Control
    targetPos=sim.getObjectPosition(targetObj,sim.handle_world);
    targetPos=cell2mat(targetPos(3));
    p=sim.getObjectPosition(d,sim.handle_world);
    pos=p(3);
    pos=cell2mat(pos);
    vel=sim.getVelocity(heli);
    l=cell2mat(vel(1,3));
    e=(targetPos-pos);
    angvel=7.015+kp*e+kd*(e-laste)+vparam*l;
    laste=e;
    
    sp=sim.getObjectPosition(targetObj,d);
    m=sim.getObjectMatrix(d,sim.handle_world);
    vx={1,0,0};
    vx=sim.multiplyVector(m,vx);
    vy={0,1,0};
    vy=sim.multiplyVector(m,vy);
    alphaE=(cell2mat(vy(3))-cell2mat(m(12)));
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE);
    betaE=(cell2mat(vx(3))-cell2mat(m(12)));
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

    thrust1=angvel1*angvel1*0.0257;
    thrust2=angvel2*angvel2*0.0257;
    thrust3=angvel3*angvel3*0.0257;
    thrust4=angvel4*angvel4*0.0257;
    force1=[0 0 thrust1];
    force2=[0 0 thrust2];
    force3=[0 0 thrust3];
    force4=[0 0 thrust4];
    m1=sim.getObjectMatrix(propeller0,sim.handle_world);
    m2=sim.getObjectMatrix(propeller1,sim.handle_world);
    m3=sim.getObjectMatrix(propeller2,sim.handle_world);
    m4=sim.getObjectMatrix(propeller3,sim.handle_world);

    m1(4)={0};m1(8)={0};m1(12)={0}; 
    m2(4)={0};m2(8)={0};m2(12)={0};
    m3(4)={0};m3(8)={0};m3(12)={0};
    m4(4)={0};m4(8)={0};m4(12)={0};
    force1=sim.multiplyVector(m1,force1);
    force2=sim.multiplyVector(m2,force2);
    force3=sim.multiplyVector(m3,force3);
    force4=sim.multiplyVector(m4,force4);
    torque1=[0 0 0.02*(angvel1)];
    torque2=[0 0 -0.02*(angvel2)];
    torque3=[0 0 0.02*(angvel3)];
    torque4=[0 0 -0.02*(angvel4)];
    torque1=sim.multiplyVector(m1,torque1);
    torque2=sim.multiplyVector(m2,torque2);
    torque3=sim.multiplyVector(m3,torque3);
    torque4=sim.multiplyVector(m4,torque4);
    sim.addForceAndTorque(propellerRespondable0,force1,torque1);
    sim.addForceAndTorque(propellerRespondable1,force2,torque2);
    sim.addForceAndTorque(propellerRespondable2,force3,torque3);
    sim.addForceAndTorque(propellerRespondable3,force4,torque4);
    
    client.step();  % triggers next simulation step
end

sim.stopSimulation();
