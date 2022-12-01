FWT = data.FWT(1:2,1:2);
A=FWT.A;
B=FWT.B;
C=FWT.C;
D=FWT.D;

s = tf('s');
G = C*inv((s*eye(5)-A))*B;

omega = [0, 0.6*pi];
    for i =1:length(omega)
        Gf=freqresp(G,omega(i));
        RGAw(:,:,i)=Gf.*inv(Gf).';
        RGAno(i)=sum(sum(abs(RGAw(:,:,i)-eye(2))));
        
    end
 RGA = frd(RGAw,omega)
%%
% pzplot(FWT);
bode(FWT)
[Gm,Pm,Wcg,Wcp] = margin(FWT);