
clc;
clear;

% Predifined
% symbols and plotting

syms s t;

t = 0:0.01:50;
unit_step = ones(size(t));


%% Self Driving Car Controls

% Safety: <10% overshoot - given unit step response
%   
% Comfort: Steering input <4°
%   

%   R(s) +   E(s)               U(s)       X(s)
%   -----> ○ ----> Gc ----> Ga ----> Gp --┬-->
%        - ↑                              |
%          └------------------------------┘

% x(t) = lateral position (lanes)
% e(t) = lateral position error (lanes)
% r(t) = desired lateral position (lanes)
% u(t) = steering angle (degrees)


%% Playing with PID controllers
% This section can be ignored

% KD = 0
% KP = 3.95
% KI = .12
% 
% if KI == 0
% 
%     Gc_num = [KD KP];
%     Gc_den = [1];
% 
% else
% 
%     Gc_num = [KD KP KI];
%     Gc_den = [1 0];
% 
% end
% 
% Gc = tf(Gc_num,Gc_den)


%% Givens

K = 4.106; % Gain
Gc = K  % P Controller

Ga_num = [10];
Ga_den = [1 10];

Ga = tf(Ga_num,Ga_den)

Gp_num = [.1];
Gp_den = [1 1 0];

Gp = tf(Gp_num,Gp_den)


%% Creating the Tranfer Function 
%% R(s) -> X(s)

%   R(s) +   E(s)               U(s)       X(s)
%   -----> ○ ----> Gc ----> Ga ----> Gp --┬-->
%        - ↑                              |
%          └------------------------------┘

G = Gc * Ga * Gp
TF = feedback(G, 1)

% Root Locus Plot
figure(1)
rlocus(Ga * Gp) % Stable when K < 110

% Step Response Plot
figure(2)
y = lsim(TF, unit_step, t);

plot(t, y)
title('Step Response of X(t)')
xlabel('Time (s)')
ylabel('Position (Lane)')
grid on


%% Finding optimal gain for overshoot
% Max overshoot vs Gain plot

K_vector = 1:.1:10;   % Gain
Mp = zeros(size(K_vector));

for i = 1:length(K_vector)

    Gc_i = K_vector(i);
    G = Gc_i * Ga * Gp;
    TF_i = feedback(G, 1);

    y = lsim(TF_i, unit_step, t);

    Mp(i) = max(y) - 1;

end

figure(3)
plot(K_vector, Mp*100)
title('Max overshoot vs Gain')
xlabel('Gc (Gain)')
ylabel('Overshoot (%)')
grid on

% Found K = 6.26 as fastest rise with <10% overshoot


%% Creating our steering TF
%% R(s) -> U(s)

%   R(s) +   E(s)               U(s)
%   -----> ○ ----> Gc ----> Ga --┬-->
%        - ↑                     |
%          |      X(s)           |
%          └----------------Gp---┘


G_steering = Gc * Ga
TF_steering = feedback(G_steering, Gp)

y_steering = lsim(TF_steering, unit_step, t);

% Steering Step Response Plot
figure (4)
plot(t, y_steering)
title('Step Response of U(t)')
xlabel('Time (s)')
ylabel('Steering Angle (degrees)')
grid on


%% Finding optimal gain for steering angle
% Max turning angle vs Gain plot

K_vector = 1:.1:10;   % Gain
Max_turning_input = zeros(size(K_vector));

for i = 1:length(K_vector)

    Gc_i = K_vector(i);
    G_steering_i = Gc_i * Ga;
    TF_steering_i = feedback(G_steering_i, Gp);

    y_steering = lsim(TF_steering_i, unit_step, t);

    Max_turning_input(i) = max(y_steering);

end

figure(5)
plot(K_vector, Max_turning_input)
title('Max turning angle vs Gain')
xlabel('Gc (Gain)')
ylabel('Angle (°)')
grid on


% Found K = 4.106 as fastest rise with <4° turning angle
% K = 4.106 approaches the limit of 4°, but does not reach 4°
% If using 3 significant figures, round down to K = 4.10


% Our dominating constraint is our 4° turning angle,
% which gives us a maximum gain of K = 4.106.


%% Finding values

Gain = K
% 4.106

Max_Overshoot_Percent = stepinfo(TF).Overshoot
% 2.93%
Rise_Time = stepinfo(TF).RiseTime
% 3.57s

Max_Steering_Angle = stepinfo(TF_steering).Peak
% 4°


%% Bode Plot

figure(6)
bode(TF)

Bandwidth = bandwidth(TF)
% .602



