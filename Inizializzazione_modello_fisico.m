%% PARAMETRI FISICI
mw=0.2; %Massa Ruota [Kg]
mb=0.4; %Massa corpo [Kg]
l=0.106; %Distanza tra vertice e centro di massa [m]
g=9.81; %Non credo serva specificare [m*s^(-1)]

%Momenti di Inerzia
Ib=0.57e-3; %Momento di inerzia della faccia [Kg*m^2]
Iw=3.34e-3; %Momento di inerzia della ruota [Kg*m^2]

% Attriti [Kg*m^2*s^(-1)]
Cb = 1.02e-3; %Attrito Corpo
Cw = 0.05e-3; %Attrito Ruota

% Stato iniziale
x0= [0 0 0];

%Vettore dei parametri del mio modello
m=mw+mb; %Massa totale [Kg]
alpha=l*m*g;
c=[alpha; Ib; Iw; Cb; Cw];