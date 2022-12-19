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
H = ultidyn("H",2);
Delta = Wo*H*Wi;
[Udelta,Sdelta,Vdelta] = svd(Delta.A);
figure()
sigma(Delta)

%% Assignment 2.4

%%
function wi = funcWi(x)
wi = ((x/(16*pi))+0.3)/((x/(64*pi))+1);
end 

function wo = funcWo(x)
wo = ((0.05*x)+0.02)/((0.01*x)+1);
end

