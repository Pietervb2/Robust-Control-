data = importdata("Assignment_Data_SC42145_2022.mat");
FWT=data.FWT;

%% 2.1 
eye(2);
G1 = FWT(1:2,1:2);
s = tf('s');
G = G1.C*inv((s*eye(5)-G1.A))*G1.B;
omega = [0, 0.6*pi];
    for i =1:length(omega)
        Gf=freqresp(G,omega(i));
        RGAw(:,:,i)=Gf.*inv(Gf).';
    end
RGA = frd(RGAw,omega)

%% 2.5 
G=tf(FWT(1:2,1:2));

wB1=0.3*2*pi; % desired closed-loop bandwidth
wB2=0.2; % desired closed-loop bandwidth
A=1/10000; % desired disturbance attenuation inside bandwidth
M=3; % desired bound on hinfnorm(S)
Wp11=tf([1/M wB1],[1 wB1*A]);
Wp=[Wp11 0; 0 wB2]; %

% Control weights
Wu22 =(5e-3*s^2+7e-4*s+5e-5)/(s^2+14e-4*s+1e-6);
Wu=[0.01 0; 0 Wu22]; % Control weight
Wt=[]; % Empty weight

Plant=augw(G,Wp,Wu,[]); % build plant
P=minreal(Plant) % to check number of states

%% 2.7

[K2,CL2,GAM2,INFO2] = hinfsyn(P,2,2);

%Nyquist plot
LoopT=G*K2+eye(2);
PolesL=pole(minreal(LoopT));
nyqDet=LoopT(1,1)*LoopT(2,2)-LoopT(1,2)*LoopT(2,1);
figure()
nyquist(nyqDet)

%% 2.8 

% Calculate S and T
Sens=(1/Wp)*CL2(1:2,1:2);
Compl=eye(2)-Sens;

% Step in reference
figure()
step(Compl(1:2,1))
stepinfo(Compl(1:2,1))


% Plot influence of step in disturbance
D_rej=Sens*FWT(1:2,3);
figure()
step(D_rej)
stepinfo(D_rej)

