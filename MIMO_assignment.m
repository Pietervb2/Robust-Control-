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
RGA = frd(RGAw,omega);
%% MIMO exercise 5
G = tf(FWT(1:2,1:2));

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

%% MIMO exercise 6
freq = logspace(-2,2);

function S = sens(s)
S = ((s^2)*5e-3+s*7e-4+5e-5) / (s^2+s*14*10^(-4)+10^(-6));
end

S_array = [50];

plot()



