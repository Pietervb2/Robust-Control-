%% SISO

data = importdata("Assignment_Data_SC42145_2022.mat");

FWT = data.FWT;
A=data.A;
B=data.B;
C=data.C;
D=data.D;
%print(data)
sys1 = ss(A,B,C,D);

step(-FWT(1,1));
bode(-FWT(1,1));
margin(-FWT(1,1)); 
pzplot(-FWT(1,1));
stepinfo(-FWT(1,1));

%%
% PID controller
Plant = -FWT(1,1);
Kp = 400;
Ki = 50.2;
Kd = 0;
Tf = 0.06;
C = pid(Kp,Ki,Kd,Tf);

Polecancel = minreal(Plant*C); % To speed up the calculations
fbsys = feedback(Polecancel,1);
bode(fbsys);
step(fbsys);
stepinfo(fbsys);

%%
Plant = -FWT(1,3);

Polecancel = minreal(Plant*C); % To speed up the calculations
fbsys = feedback(Polecancel,1);
bode(fbsys)
% step(fbsys);
stepinfo(fbsys);

