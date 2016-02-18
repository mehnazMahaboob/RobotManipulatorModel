clear all
close all
clc

L(1) = Link([0 1 0 pi/2 0]);
L(2) = Link([0 0 1 0 0]);
L(3) = Link([0 0 1 0 0]);
L;
mehnaz = SerialLink(L, 'name', 'Elbow Manipulator');
mehnaz;
mehnaz.plotopt = {'workspace' [-5,5,-5,5,-5,5]};


t = [0:0.05:2]';
%time for each movement

RobotReadyPos = transl(0.4, 0.2, 0.75)*trotx(pi);
MoveToReadyPosMove = mehnaz.ikine6s(RobotReadyPos, 'ru');
%robot ready position

PosA1 = transl(0.4, 0.2, 0.5)*trotx(pi);
MoveToPosA1 = mehnaz.ikine6s(PosA1, 'ru');

ReadyPosA1 = transl(-0.4, 0.5, 0.5)*trotx(pi);
MoveToReadyPosA1 = mehnaz.ikine6s(ReadyPosA1, 'ru');

PosB1 = transl(0.350, 0.2, 0.5)*trotx(pi);
MoveToReadyPosB1 = mehnaz.ikine6s(PosB1, 'ru');

ReadyPosB1 = transl(0.375, 0.2, 0.5)*trotx(pi);
MoveToPosB1 = mehnaz.ikine6s(ReadyPosB1, 'ru');

PosC1 = transl(0.375, 0.25, 0.5)*trotx(pi);
MoveToPosC1 = mehnaz.ikine6s(PosC1, 'ru');

ReadyPosC1 = transl(0.4, 0.25, 0.5)*trotx(pi);
MoveToReadyPosC1 = mehnaz.ikine6s(ReadyPosC1, 'ru');

PosD1 = transl(-0.375, 0.5, 0.25)*trotx(pi);
MoveToPosD1 = mehnaz.ikine6s(PosD1, 'ru');

ReadyPosD1 = transl(-0.4, 0.5, 0.25)*trotx(pi);
MoveToReadyPosD1 = mehnaz.ikine6s(ReadyPosD1, 'ru');

PosE1 = transl(-0.325, 0.5, 0.25)*trotx(pi);
MoveToPosE1 = mehnaz.ikine6s(PosE1, 'ru');

ReadyPosE1 = transl(-0.35, 0.5, 0.25)*trotx(pi);
MoveToReadyPosE1 = mehnaz.ikine6s(ReadyPosE1, 'ru');


PosA2 = transl(0.4, 0.2, 0.5)*trotx(pi);
MoveToPosA2 = mehnaz.ikine6s(PosA2, 'ru');

ReadyPosA2 = transl(-0.4, 0.5, 0.5)*trotx(pi);
MoveToReadyPosA2 = mehnaz.ikine6s(ReadyPosA2, 'ru');


PosB2 = transl(0.350, 0.2, 0.5)*trotx(pi);
MoveToPosB2 = mehnaz.ikine6s(PosB2, 'ru');

ReadyPosB2 = transl(0.375, 0.2, 0.5)*trotx(pi);
MoveToReadyPosB2 = mehnaz.ikine6s(ReadyPosB2, 'ru');

PosC2 = transl(0.375, 0.25, 0.5)*trotx(pi);
MoveToPosC2 = mehnaz.ikine6s(PosC2, 'ru');

ReadyPosC2 = transl(0.4, 0.25, 0.5)*trotx(pi);
MoveToReadyPosC2 = mehnaz.ikine6s(ReadyPosC2, 'ru');

PosD2 = transl(-0.375, 0.5, 0.25)*trotx(pi);
MoveToPosD2 = mehnaz.ikine6s(PosD2, 'ru');

ReadyPosD2 = transl(-0.4, 0.5, 0.25)*trotx(pi);
MoveToReadyPosD2 = mehnaz.ikine6s(ReadyPosD2, 'ru');

PosE2 = transl(-0.325, 0.5, 0.25)*trotx(pi);
MoveToPosE2 = mehnaz.ikine6s(PosE2, 'ru');

ReadyPosE2 = transl(-0.35, 0.5, 0.25)*trotx(pi);
MoveToReadyPosE2 = mehnaz.ikine6s(ReadyPosE2, 'ru');

RobotThroughPos = transl(-0.325, 0.5, 0.25)*trotx(pi);
MoveToRobotThroughPos = mehnaz.ikine6s(RobotThroughPos, 'ru');
%these are the inverse kinematics
%'ru' is forcing the elbot to point up

AStep1 = jtraj(MoveToRobotReadyPos, MoveToReadyPosA1, t);
mehnaz.plot(AStep1)
%approach object

AStep2 = jtraj(MoveToReadyPosA1, MoveToPosA1, t);
mehnaz.plot(AStep2)
%approach object

AStep3 = jtraj(MoveToPosA1, MoveToReadyPosA1, t);
mehnaz.plot(AStep3)
%approach object

AStep4 = jtraj(MoveToReadyPosA1, MoveToRobotThroughPos, t);
mehnaz.plot(AStep4)
%approach object

AStep5 = jtraj(MoveToRobotThroughPos, MoveToReadyPosA2, t);
mehnaz.plot(AStep5)
%approach object

AStep6 = jtraj(MoveToReadyPosA2, MoveToPosA2, t);
mehnaz.plot(AStep6)
%approach object

AStep7 = jtraj(MoveToPosA2, MoveToReadyPosA2, t);
mehnaz.plot(AStep7)
%approach object

AStep8 = jtraj(MoveToReadyPosA2, MoveToRobotThroughPos, t);
mehnaz.plot(AStep8)
%approach object

BStep1 = jtraj(MoveToRobotThroughPos, MoveToReadyPosB1, t);
mehnaz.plot(BStep1)
%approach object

BStep2 = jtraj(MoveToReadyPosB1, MoveToPosB1, t);
mehnaz.plot(BStep2)
%approach object

BStep3 = jtraj(MoveToPosB1, MoveToReadyPosB1, t);
mehnaz.plot(BStep3)
%approach object

BStep4 = jtraj(MoveToReadyPosB1, MoveToRobotThroughPos, t);
mehnaz.plot(BStep4)
%approach object

BStep5 = jtraj(MoveToRobotThroughPos, MoveToReadyPosB2, t);
mehnaz.plot(BStep5)
%approach object

BStep6 = jtraj(MoveToReadyPosB2, MoveToPosB2, t);
mehnaz.plot(BStep6)
%approach object

BStep7 = jtraj(MoveToPosB2, MoveToReadyPosB2, t);
mehnaz.plot(BStep7)
%approach object

BStep8 = jtraj(MoveToReadyPosB2, MoveToRobotThroughPos, t);
mehnaz.plot(BStep8)
%approach object

CStep1 = jtraj(MoveToRobotThroughPos, MoveToReadyPosC1, t);
mehnaz.plot(CStep1)
%approach object

CStep2 = jtraj(MoveToReadyPosC1, MoveToPosC1, t);
mehnaz.plot(CStep2)
%approach object

CStep3 = jtraj(MoveToPosC1, MoveToReadyPosC1, t);
mehnaz.plot(CStep3)
%approach object

CStep4 = jtraj(MoveToReadyPosC1, MoveToRobotThroughPos, t);
mehnaz.plot(CStep4)
%approach object

CStep5 = jtraj(MoveToRobotThroughPos, MoveToReadyPosC2, t);
mehnaz.plot(CStep5)
%approach object

CStep6 = jtraj(MoveToReadyPosC2, MoveToPosC2, t);
mehnaz.plot(CStep6)
%approach object

CStep7 = jtraj(MoveToPosC2, MoveToReadyPosC2, t);
mehnaz.plot(CStep7)
%approach object

CStep8 = jtraj(MoveToReadyPosC2, MoveToRobotThroughPos, t);
mehnaz.plot(CStep8)
%approach object

DStep1 = jtraj(MoveToRobotThroughPos, MoveToReadyPosD1, t);
mehnaz.plot(DStep1)
%approach object

DStep2 = jtraj(MoveToReadyPosD1, MoveToPosD1, t);
mehnaz.plot(DStep2)
%approach object

DStep3 = jtraj(MoveToPosD1, MoveToReadyPosD1, t);
mehnaz.plot(DStep3)
%approach object

DStep4 = jtraj(MoveToReadyPosD1, MoveToRobotThroughPos, t);
mehnaz.plot(DStep4)
%approach object

DStep5 = jtraj(MoveToRobotThroughPos, MoveToReadyPosD2, t);
mehnaz.plot(DStep5)
%approach object

DStep6 = jtraj(MoveToReadyPosD2, MoveToPosD2, t);
mehnaz.plot(DStep6)
%approach object

DStep7 = jtraj(MoveToPosD2, MoveToReadyPosD2, t);
mehnaz.plot(DStep7)
%approach object

DStep8 = jtraj(MoveToReadyPosD2, MoveToRobotThroughPos, t);
mehnaz.plot(DStep8)
%approach object

EStep1 = jtraj(MoveToRobotThroughPos, MoveToReadyPosE1, t);
mehnaz.plot(EStep1)
%approach object

EStep2 = jtraj(MoveToReadyPosE1, MoveToPosE1, t);
mehnaz.plot(EStep2)
%approach object

EStep3 = jtraj(MoveToPosE1, MoveToReadyPosE1, t);
mehnaz.plot(EStep3)
%approach object

EStep4 = jtraj(MoveToReadyPosE1, MoveToRobotThroughPos, t);
mehnaz.plot(EStep4)
%approach object

EStep5 = jtraj(MoveToRobotThroughPos, MoveToReadyPosE2, t);
mehnaz.plot(EStep5)
%approach object

EStep6 = jtraj(MoveToReadyPosE2, MoveToPosE2, t);
mehnaz.plot(EStep6)
%approach object

EStep7 = jtraj(MoveToPosE2, MoveToReadyPosE2, t);
mehnaz.plot(EStep7)
%approach object

EStep8 = jtraj(MoveToReadyPosE2, MoveToRobotThroughPos, t);
mehnaz.plot(EStep8)
%approach object