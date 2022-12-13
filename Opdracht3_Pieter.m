% clear all

data = importdata("Assignment_Data_SC42145_2022.mat");
WindData=data.WindData;
FWT=data.FWT;
G=[FWT(1,1) FWT(1,2)];
%G=tf(G);
Gd = FWT(1,3);
s = tf('s');

wB1=0.3*2*pi; % desired closed-loop bandwidth
A=1/10000; % desired disturbance attenuation inside bandwidth
M=3 ; % desired bound on hinfnorm(S)
% Wp11=tf([1/sqrt(M) (2*wB1)/sqrt(M) wB1^2],[1 2*wB1*sqrt(A) A*wB1*wB1]);
Wp11=(((s/sqrt(M))+wB1)^2)/((s+(wB1*sqrt(A)))^2);
Wp=[Wp11]; %
%Sensitivity weight
%tau11 = 0.1;

% Values for lead lag Wu11 
tau11= 2;
k1 = 1500;
a1 = 0.5;

% Values for lead lag Wu22
tau22 = 10;
k2 = 0.0001;
a2=0.01;

a1_low = 30;
tau11_low = 0.01;
Wu11 = a1*((k1*tau11*s+1)/(tau11*s+1)); %lead lag filter 
Wu11_low = a1_low*(1/(tau11_low*s +1));
Wu22 = a2*(1/k2)*((tau22*k2*s+1)/(tau22*s+1));  %lead lag filter

Wu=[Wu11 0; 0 Wu22]; % Control weight
Wt=[]; % Empty weight
% figure(1)
% bode(1/Wu11,1/Wu22)
% legend({'Wu11=beta','Wu22=tau'})

%%
tau11= 0.1;
k1 = 2500;
a1 = 0.1;

Wu11 = a1*((k1*tau11*s+1)/(tau11*s+1)); %lead lag filter 
figure()
bode(1/Wu11);
title("1/Wu11")

hold on
tau11 = 0.1;
k1 = 1500;
Wu11 = a1*((k1*tau11*s+1)/(tau11*s+1)); %lead lag filter 
bode(1/Wu11);

hold on
tau11 = 0.1;
k1 = 500;
Wu11 = a1*((k1*tau11*s+1)/(tau11*s+1)); %lead lag filter 
bode(1/Wu11);

legend({'2500','1500','500'});


% figure()
% bode(1/Wu22)
% title("1/Wu22")


%%
close all

P11=Wp*Gd;
P12=Wp*-G;
P21=[0;0];
P22=Wu;
P31=Gd;
P32=-G;

P=[P11 P12; P21 P22;P31 P32];
%P=tf(P);

[K,CL,GAM,INFO] = hinfsyn(P,1,2);

Sens=(1/Wp);
% bode(Sens)

KS1=(1/(Wu11))*CL(2);
KS2=(1/(Wu11_low))*CL(3);

t = WindData(:,1);
input = WindData(:,2);

% figure(2)
% lsim(Sens*CL(1),input,t)
% legend('controlled')

figure(3)
lsim(KS1,input,t)
legend('beta')

figure(4)
lsim(KS2,input,t)
legend('torque')

t1 = linspace(0,599,6000);
u1 = sin(2*pi*t1/1000);

% figure(4)
% 
% subplot(3,1,2)
% lsim(K(1),u1,t1);
% legend('Kbeta')
% subplot(3,1,3)
% lsim(K(2),u1,t1);
% legend('Ktau')
% subplot(3,1,1)
% lsim(Sens,u1,t1);
% legend('S')

%%
tau = [10 20 40 60 100 200 300];
for i = tau
    tst = 15/(i*s + 1);
    hold on 
    bode(1/tst);
end
legend({'10','20','40','60','100','200','300'})

%%
Ts = 1/100000;
x = WindData(:,2);
y = fft(x);
fs = 1/Ts;
f = (0:length(y)-1)*fs/length(y);
plot(f,log(abs(y)))

