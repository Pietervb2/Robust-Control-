data = importdata("Assignment_Data_SC42145_2022.mat");
FWT = data.FWT;

% FWT = data.FWT(1:2,1:2);
A=FWT(1:2,1:2).A;
B=FWT(1:2,1:2).B;
C=FWT(1:2,1:2).C;
D=FWT(1:2,1:2).D;

s = tf('s');
G = C*inv((s*eye(5)-A))*B;

omega = [0, 0.6*pi];
    for i =1:length(omega)
        Gf=freqresp(G,omega(i));
        RGAw(:,:,i)=Gf.*inv(Gf).';
        RGAno(i)=sum(sum(abs(RGAw(:,:,i)-eye(2))));
        
    end
 RGA = frd(RGAw,omega)
%% MIMO exercise 4
wB1=0.3; % desired closed-loop bandwidth

% Wp
A_Wp = 10^-4;
M=3 ; % desired bound on hinfnorm(S)
Wp11 = (s/M+wB1)/(s+wB1*A_Wp);
Wp=[Wp11 0; 0 0.2]; %

% Wu
Wu22 = ((s^2)*5e-3+s*7e-4+5e-5) / (s^2+s*14*10^(-4)+10^(-6));
Wu=[0.01 0; 0 Wu22]; % Control weight
    
% Wt
Wt = []; % Empty weight 


[K,CL,GAM,INFO]=mixsyn(minreal(G),Wp,Wu,Wt);



%% doesn't work yet 
% w = [1 2]
% u = [1 2]
% systemnames ='G Wp Wu Wt'; % Define systems
% inputvar =[w(2); u(2)]; % Input generalized plant
% input_to_G = [u];
% input_to_Wu= [u];
% input_to_Wt= [G_real];
% input_to_Wp= [w+G_real];
% outputvar= [Wp; Wt; Wu; G_real+w]; % Output generalized plant
% sysoutname=P;
% T = sysic
