data = importdata("Assignment_Data_SC42145_2022.mat")
FWT=data.FWT;

%% 1.1 
bode(-FWT(1,1))
step(-FWT(1,1))
margin(-FWT(1,1))
pzmap(-FWT(1,1));
A =zero(-FWT(1,1));
stepinfo(-FWT(1,1))

%% 1.3
s = tf('s');
Plant = -FWT(1,1);

% PID weights
Kp = 0;
Ki = 0.26;
Kd = 0;
Tf=0;
K = pid(Kp, Ki, Kd);

PoleCancel = minreal(Plant*K);
margin(PoleCancel)
sys_cl = feedback(PoleCancel,1);

% Step response and other system specificiations
[y,t] = step(sys_cl);
step(sys_cl)
stepinfo(sys_cl)
rlocus(PoleCancel)
