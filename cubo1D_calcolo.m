%Parametri fisici
%Calcolo inerzie
%Dinamica non lineare
% Linearizzazione
%Calcolo velocità ruota per jump-up
%LQR
%% PARAMETRI FISICI

g = 9.81;              % gravità [m/s^2]

% Masse
mb = 0.419;            % massa corpo [kg]
mw = 0.204;            % massa ruota [kg]

% Geometria
l  = 0.085;            % distanza ruota - pivot [m]
lb = 0.075;            % distanza baricentro corpo - pivot [m]

% Inerzie
Ib = 3.34e-3;          % inerzia corpo [kg m^2]
Iw = 0.57e-3;          % inerzia ruota [kg m^2]

% Attriti (opzionali)
Cb = 1.02e-3;
Cw = 0.05e-3;

% Motore
Km = 25.1e-3;          % Nm/A
%% DINAMICA NON LINEARE

syms theta_b dtheta_b theta_w dtheta_w Tm real

den = Ib + mw*l^2;

ddtheta_b = ((mb*lb + mw*l)*g*sin(theta_b) - Tm - Cb*dtheta_b + Cw*dtheta_w) / den;

ddtheta_w = ((Ib + Iw + mw*l^2)*(Tm - Cw*dtheta_w) - Iw*((mb*lb + mw*l)*g*sin(theta_b) + Cb*dtheta_b)) / (Iw*den);
%% JUMP-UP: velocità minima ruota

omega_w_sq = (2 - sqrt(2)) * ((Iw + Ib + mw*l^2) / Iw^2) * (mb*lb + mw*l) * g;

omega_w = sqrt(omega_w_sq);   % [rad/s]
rpm_w   = omega_w * 60 / (2*pi);

fprintf('Velocità ruota richiesta:\n');
fprintf('omega_w = %.1f rad/s\n', omega_w);
fprintf('rpm     = %.0f rpm\n', rpm_w);
%% MODELLO LINEARIZZATO

A = [ 0 1 0; (mb*lb+mw*l)*g/den   -Cb/den   Cw/den; -(mb*lb+mw*l)*g/den   Cb/den   -Cw*(Ib+Iw+mw*l^2)/(Iw*den)];

B = [0; -Km/den; Km*(Ib+Iw+mw*l^2)/(Iw*den)];
%% LQR

Q = diag([100 1 0.1]);
R = 0.01;

K = lqr(A,B,Q,R);
%% Angolo massimo recupoerabile
Iw = 0.57e-3;
omega_w_max = 500;    % rad/s

mb = 0.419;
mw = 0.204;
lb = 0.075;
l  = 0.085;
g  = 9.81;

arg = 1 - (Iw * omega_w_max^2) / (2 * (mb*lb + mw*l) * g);

arg = max(min(arg,1),-1);   % clamp numerico altrimenti acos() va fuori accetta solo valori tra -1 e 1

theta_max = acos(arg);
