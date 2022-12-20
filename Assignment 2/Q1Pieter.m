data = importdata("Assignment_Data_SC42145_2022.mat");
FWT=data.FWT;
G = FWT(1:2,1:2);
%% Assignment 2.3 first part
s = tf('s');

Wi1=((s/(16*pi))+0.3)/((s/(64*pi))+1);
Wi2 = Wi1;
Wi=[Wi1 0;0 Wi2];

Wo1=((0.05*s)+0.02)/((0.01*s)+1);
Wo2= Wo1;
Wo=[Wo1 0;0 Wo2];

freq = logspace(-5,5,100);

arrayWi = [];
arrayWo = [];
for i = linspace(1,100,100)
    arrayWi(i) = funcWi(freq(i));
    arrayWo(i) = funcWo(freq(i));
end

figure()
loglog(freq,arrayWi)
title('Wi')

figure()
loglog(freq,arrayWo)
title('Wo')

%% Assignment 2.3 second part
H1 = ultidyn("H1",2);
H2 = ultidyn("H2",2);

Gp = (eye(2)+H1*Wi)*G*(eye(2)+H2*Wo);
sigma(Gp)

%% Assignment 2.4
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

G=tf(FWT(1:2,1:2));

%get controller from first part
Plant=augw(G,Wp,Wu,[]); 
P1=minreal(Plant); 
[K,CL,GAM,INFO] = hinfsyn(P1,2,2); %K is controller

P=[zeros(2) zeros(2) zeros(2) Wi;Wo*G zeros(2) zeros(2) Wo*G;Wp*G Wp Wp Wp*G;zeros(2) zeros(2) zeros(2) Wu; -G -1*eye(2) -1*eye(2) -G];
% Pmin=minreal(P);
% Pl=ss(Pmin);
% Pss=minreal(Pl);

%% 2.5 NS
N=lft(P,K);
Nmin=minreal(N);

K25,

Nnominal = N(5:6,5:6)
det = N(5,5))*(N(6,6)) - N(5,6)*N(6,5)
figure()
nyquist(det)


%% 2.5 NP
omega=logspace(-3,3,300);
Nf=frd(N,omega);
blk_NP=[ 2 4]; % Full complex uncertainty block
[mubnds,muinfo]=mussv(Nf(5:8,5:6),blk_NP,'c');
muNP=mubnds(:,1);
[muNPinf, muNPw]=norm(muNP,inf); 

Delta = [H1 zeros(2); zeros(2) H2];
% M = lft(Delta,N);
% Mmin = minreal(M);
%% 2.5 RS 
omega=logspace(-3,3,61);
Nf=frd(N,omega);
blk_RS = [1 0; 1 0; 1 0; 1 0] ; % structured uncertainty
[mubnds,muinfo]=mussv(Nf(1:4,1:4),blk_RS,'c');
muRS=mubnds(:,1);
[muRSinf, muRSw] = norm(muRS,inf)

%% 2.5 RP 
omega=logspace(-3,3,61);
Nf=frd(N,omega);
blk_RP = [1 0; 1 0; 1 0; 1 0; 2 4]; % structured uncertainty and ∆P
[mubnds,muinfo]=mussv(Nf,blk_RP,'c');
muRP=mubnds(:,1);
[muRPinf, muRPw] = norm(muRP,inf)

%%
function wi = funcWi(x)
wi = ((x/(16*pi))+0.3)/((x/(64*pi))+1);
end 

function wo = funcWo(x)
wo = ((0.05*x)+0.02)/((0.01*x)+1);
end

