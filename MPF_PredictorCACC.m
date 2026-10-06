%% ========================================================================
%  Predictor-based CACC simulation of a heterogeneous platoon
%  (leader L + 9 followers), constant-time-headway (CTH) spacing policy.
%
%  Overview
%  - Every vehicle has a first-order actuation lag (time constant tau_i)
%    and a common input (actuation) delay D = d*ts.
%  - Vehicle i receives data from its predecessors over V2V links that have
%    vehicle-dependent communication delays D_c_j = d_c_j*ts.
%  - Each follower uses a predictor (Artstein-type, finite-horizon
%    integral) to compensate the actuation delay D, plus an integral-like
%    term (sigma) that compensates the mismatch caused by communication
%    delay.
%  - Followers 1-2 use all available predecessors; followers 3-9 use
%    the three preceding vehicles (11-dimensional state).
%  - Time stepping: explicit Euler with step ts. All signals are stored as
%    sample-index arrays, so "x(:, i-d)" means "the value d samples ago".
%
%  Notation
%    s_i  : spacing error / spacing state of vehicle i
%    v_i  : speed of vehicle i              (v_L: leader)
%    a_i  : acceleration of vehicle i       (a_L: leader)
%    u_i  : control input of vehicle i      (u_L = U_0: leader input)
%    x_i  : state vector of follower i (see "Initializing" section)
%    Q_i  : predictor state of follower i
%  ========================================================================
close all
clear all

%% ------------------------------------------------------------------------
% Global simulation settings
% -------------------------------------------------------------------------
tf=100;            % total simulation horizon [s]
ts=0.01;           % sampling / Euler integration step [s]
%lamda=0.6;
t=0:ts:tf;         % time vector
d=70;              % actuation (input) delay in samples
D=d*ts;            % actuation delay in seconds (= 0.7 s), same for all vehicles

%%
% Defining Vehicles and control parameters


%Communicatio value


% t_t=0:ts:tf-D-max_d_c*ts;


%Vehicle L
% d_0=20;
% D_0=d_0*ts;

% ---- Leader (vehicle 0 / L) ----
d_c_0=3;           % communication delay of the leader's transmitted data [samples]
D_c_0=d_c_0*ts;    % same delay in seconds
tau_0=0.3;         % leader actuation time constant [s]

%Vehicle 1
d_c_1=9;           % communication delay of vehicle 1 [samples]
D_c_1=d_c_1*ts;    % [s]

d_r_1=0;           % (unused) receiving delay [samples]
D_r_1=d_r_1*ts;
%t_t_1=0:ts:tf-D;

tau_1=0.3;         % actuation time constant of vehicle 1 [s]
h_1=0.4;           % time headway of vehicle 1 [s]
% lamda_1=lamda;
% h_1=1;


% p1=-1.5^2/h_1;
% p_1=-2.5/(h_1-d_c_1*ts);
a_1=5;             % controller tuning parameters (gains a, b, c) of vehicle 1
b_1=10;
c_1=2;

% a_1=lamda_1^3*h_1^4;
% b_1=-a_1+3*(lamda_1*h_1)^2;
% c_1=(1/tau_1)-3*(lamda_1*h_1);

%Vehicle 2
d_c_2=12;          % communication delay [samples]
D_c_2=d_c_2*ts;

d_r_2=0;
D_r_2=d_r_2*ts;
%t_t_2=0:ts:tf-D;

tau_2=0.25;        % actuation time constant [s]
h_2=0.4;           % time headway [s]
% lamda_2=lamda;
% h_1=1;


% p2=-1.5^2/h_2;
% p_2=-2.5/(h_2-d_c_2*ts);
a_2=5;             % controller gains
b_2=10;
c_2=2;
% a_2=lamda_2^3*h_2^4;
% b_2=-a_2+3*(lamda_2*h_2)^2;
% c_2=(1/tau_2)-3*(lamda_2*h_2);

%Vehicle 3
d_c_3=14;          % communication delay [samples]
D_c_3=d_c_3*ts;

d_r_3=0;
D_r_3=d_r_3*ts;
%t_t_3=0:ts:tf-D;

tau_3=0.25;        % actuation time constant [s]
h_3=0.5;           % time headway [s]
% lamda_3=lamda;
% h_1=1;

% p3=-1.5^2/h_3;
% p_3=-2.5/(h_3-d_c_3*ts);
a_3=5;             % controller gains
b_3=10;
c_3=2;
% a_3=lamda_3^3*h_3^4;
% b_3=-a_3+3*(lamda_3*h_3)^2;
% c_3=(1/tau_3)-3*(lamda_3*h_3);

%Vehicle 4
d_c_4=9;           % communication delay [samples]
D_c_4=d_c_4*ts;

d_r_4=0;
D_r_4=d_r_4*ts;
%t_t_4=0:ts:tf-D;

tau_4=0.2;         % actuation time constant [s]
h_4=0.5;           % time headway [s]
% lamda_4=lamda;
% h_1=1;


% p4=-1.5^2/h_4;
% p_4=-2.5/(h_4-d_c_4*ts);
a_4=5;             % controller gains
b_4=10;
c_4=2;
% a_4=lamda_4^3*h_4^4;
% b_4=-a_1+3*(lamda_4*h_4)^2;
% c_4=(1/tau_4)-3*(lamda_4*h_4);

%Vehicle 5
d_c_5=18;          % communication delay [samples]
D_c_5=d_c_5*ts;

d_r_5=0;
D_r_5=d_r_5*ts;
%t_t_5=0:ts:tf-D;

tau_5=0.25;        % actuation time constant [s]
h_5=0.3;           % time headway [s]
% lamda_5=lamda;
% h_1=1;


% p5=-1.5^2/h_5;
% p_5=-2.5/(h_5-d_c_5*ts);
a_5=5;             % controller gains
b_5=10;
c_5=2;
% a_5=lamda_5^3*h_5^4;
% b_5=-a_5+3*(lamda_5*h_5)^2;
% c_5=(1/tau_5)-3*(lamda_5*h_5);

%Vehicle 6
d_c_6=10;          % communication delay [samples]
D_c_6=d_c_6*ts;

d_r_6=0;
D_r_6=d_r_6*ts;
%t_t_6=0:ts:tf-D;

tau_6=0.3;         % actuation time constant [s]
h_6=0.25;          % time headway [s]
% lamda_6=lamda;
% h_1=1;

% p6=-1.5^2/h_6;
% p_6=-2.5/(h_6-d_c_6*ts);
a_6=5;             % controller gains
b_6=10;
c_6=2;
% a_6=lamda_6^3*h_6^4;
% b_6=-a_6+3*(lamda_6*h_6)^2;
% c_6=(1/tau_6)-3*(lamda_6*h_6);

%Vehicle 7
d_c_7=12;          % communication delay [samples]
D_c_7=d_c_7*ts;

d_r_7=0;
D_r_7=d_r_7*ts;
%t_t_7=0:ts:tf-D;

tau_7=0.3;         % actuation time constant [s]
h_7=0.25;          % time headway [s]
% lamda_7=lamda;
% h_1=1;


% p7=-1.5^2/h_7;
% p_7=-2.5/(h_7-d_c_7*ts);
a_7=5;             % controller gains
b_7=10;
c_7=2;
% a_7=lamda_7^3*h_7^4;
% b_7=-a_7+3*(lamda_7*h_7)^2;
% c_7=(1/tau_7)-3*(lamda_7*h_7);

%Vehicle 8
d_c_8=14;          % communication delay [samples]
D_c_8=d_c_8*ts;

d_r_8=0;
D_r_8=d_r_8*ts;
%t_t_8=0:ts:tf-D;

tau_8=0.25;        % actuation time constant [s]
h_8=0.5;           % time headway [s]
% lamda_8=lamda;
% h_1=1;


% p8=-1.5^2/h_8;
% p_8=-2.5/(h_8-d_c_8*ts);
a_8=5;             % controller gains
b_8=10;
c_8=2;
% a_8=lamda_8^3*h_8^4;
% b_8=-a_8+3*(lamda_8*h_8)^2;
% c_8=(1/tau_8)-3*(lamda_8*h_8);

%Vehicle 9
d_c_9=6;           % communication delay [samples]
D_c_9=d_c_9*ts;

d_r_9=0;
D_r_9=d_r_9*ts;
%t_t_9=0:ts:tf-D;

tau_9=0.3;         % actuation time constant [s]
h_9=0.3;           % time headway [s]
% lamda_9=lamda;
% h_1=1;


% p9=-1.5^2/h_9;
% p_9=-2.5/(h_9-d_c_9*ts);
a_9=5;             % controller gains
b_9=10;
c_9=2;
% a_9=lamda_9^3*h_9^4;
% b_9=-a_9+3*(lamda_9*h_9)^2;
% c_9=(1/tau_9)-3*(lamda_9*h_9);


% Largest communication delay in the platoon. It defines the length of the
% initial history window that must be pre-filled (d + max_d_c samples) and
% the number of samples discarded at the start when plotting.
A=[d_c_0,d_c_1,d_c_2,d_c_3,d_c_4,d_c_5,d_c_6,d_c_7,d_c_8,d_c_9];
max_d_c=max(A);
D_c=max_d_c*ts;
t_t=0:ts:tf-D-D_c;   % time vector used for plotting (history window removed)






% t_t_1=0:ts:tf-max(D_r_1,D)-D_c;
% t_t_2=0:ts:tf-max(D_r_2,D)-D_c;
% t_t_3=0:ts:tf-max(D_r_3,D)-D_c;
% t_t_4=0:ts:tf-max(D_r_4,D)-D_c;
% t_t_5=0:ts:tf-max(D_r_5,D)-D_c;
% t_t_6=0:ts:tf-max(D_r_6,D)-D_c;
% t_t_7=0:ts:tf-max(D_r_7,D)-D_c;
% t_t_8=0:ts:tf-max(D_r_8,D)-D_c;
% t_t_9=0:ts:tf-max(D_r_8,D)-D_c;



%%
%initializing 
% State vector layouts (columns = time samples):
%   x_1 (5 states) : [s1; v1; vL; a1; aL]
%   x_2 (8 states) : [s2; s1; v2; v1; vL; a2; a1; aL]
%   x_3 (11 states): [s3; s2; s1; v3; v2; v1; vL; a3; a2; a1; aL]
%   x_i, i=4..9 (11 states): [s_i; s_{i-1}; s_{i-2}; v_i; v_{i-1}; v_{i-2}; v_{i-3};
%                              a_i; a_{i-1}; a_{i-2}; a_{i-3}]
% Initial conditions are constant over the whole history window
% (the first d+1+max_d_c samples), which provides the "past" values needed by the
% delayed terms in the predictor.


%vehicle1
x_1=zeros(5,length(t)+1);
for w= 1:d+1+max_d_c
x_1(:,w)= [5.5;15;14;0;0];   % initial spacing, speeds (v1=15, vL=14), zero accelerations
end


%vehicle 2
x_2=zeros(8,length(t)+1);
for w= 1:d+1+max_d_c
x_2(:,w)= [6;5.5;15;15;14;0;0;0];
end
% x_2(:,d+d_c+1)= [15;15;15;0;0];
%vehicle 3
x_3=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_3(:,w)= [7.5;6;5.5;15;15;15;14;0;0;0;0];
end
% x_3(:,d+d_c+1)= [15;15;15;0;0];
%vehicle 4
x_4=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_4(:,w)= [7.5;7.5;6;15;15;15;15;0;0;0;0];
end
%vehicle 5
x_5=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_5(:,w)=[4.5;7.5;7.5;15;15;15;15;0;0;0;0];
end
%vehicle 6
x_6=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_6(:,w)= [3.75;4.5;7.5;15;15;15;15;0;0;0;0];
end
%vehicle 7
x_7=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_7(:,w)= [3.75;3.75;4.5;15;15;15;15;0;0;0;0];
end
%vehicle 8
x_8=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_8(:,w)= [7.5;3.75;3.75;15;15;15;15;0;0;0;0];
end
%vehicle 9
x_9=zeros(11,length(t)+1);
for w= 1:d+1+max_d_c
x_9(:,w)= [4.5;7.5;3.75;15;15;15;15;0;0;0;0];
end


% Control input histories (U_0 = leader input, U_i = input of follower i)
U_0=zeros(1,length(t)+1);
U_1=zeros(1,length(t)+1);
U_2=zeros(1,length(t)+1);
U_3=zeros(1,length(t)+1);
U_4=zeros(1,length(t)+1);
U_5=zeros(1,length(t)+1);
U_6=zeros(1,length(t)+1);
U_7=zeros(1,length(t)+1);
U_8=zeros(1,length(t)+1);
U_9=zeros(1,length(t)+1);


% Predictor state histories Q_i (same dimension as x_i)
Q_1=zeros(5,length(t)+1);
Q_2=zeros(8,length(t)+1);
Q_3=zeros(11,length(t)+1);
Q_4=zeros(11,length(t)+1);
Q_5=zeros(11,length(t)+1);
Q_6=zeros(11,length(t)+1);
Q_7=zeros(11,length(t)+1);
Q_8=zeros(11,length(t)+1);
Q_9=zeros(11,length(t)+1);


% sigma_i: auxiliary integral state that accounts for the spacing offset
% caused by the communication delay of the predecessor's speed
% (sigma_i' = v_pred(t-d_c) - v_pred(t)).
sigma_1 =zeros(1,length(t)+1);
%sigma_1(1,d+d_c_1+1)=0;
sigma_2 =zeros(1,length(t)+1);
%sigma_2(1,d+d_c_2+1)=0;
sigma_3 =zeros(1,length(t)+1);
%sigma_3(1,d+d_c_3+1)=0;
sigma_4 =zeros(1,length(t)+1);
%sigma_4(1,d+d_c_4+1)=0;
sigma_5 =zeros(1,length(t)+1);
%sigma_5(1,d+d_c_5+1)=0;
sigma_6 =zeros(1,length(t)+1);
%sigma_6(1,d+d_c_6+1)=0;
sigma_7 =zeros(1,length(t)+1);
%sigma_7(1,d+d_c_7+1)=0;
sigma_8 =zeros(1,length(t)+1);
%sigma_8(1,d+d_c_8+1)=0;
sigma_9 =zeros(1,length(t)+1);
%sigma_9(1,d+d_c_9+1)=0;
sigma_10 =zeros(1,length(t)+1);   % (unused)
%sigma_10(1,d+d_c_10+1)=0;


%%
%Model definition
% Continuous-time dynamics  x_i' = A_i x_i + sum_j B_j_i * u_j(t-D)
% - Spacing rows:      s_k' = v_{k-1} - v_k
% - Speed rows:        v_k' = a_k
% - Acceleration rows: a_k' = -a_k/tau_k + u_k/tau_k  (first-order actuator lag)
% B_j_i is the input matrix through which the input of vehicle j enters the
% model of vehicle i (it only touches the acceleration row of vehicle j).

%Vehicle 1
A_1=[0,-1,1,0,0;0,0,0,1,0;0,0,0,0,1;0,0,0,-1/tau_1,0;0,0,0,0,-1/tau_0];
B_1_1=[0;0;0;1/tau_1;0];     % own input u_1
B_0_1=[0;0;0;0;1/tau_0];     % leader input u_L
%Vehicle 2
A_2=[0,0,-1,1,0,0,0,0;0,0,0,-1,1,0,0,0;0,0,0,0,0,1,0,0;0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,1;0,0,0,0,0,-1/tau_2,0,0;0,0,0,0,0,0,-1/tau_1,0;0,0,0,0,0,0,0,-1/tau_0];
B_2_2=[0;0;0;0;0;1/tau_2;0;0];   % own input u_2
B_1_2=[0;0;0;0;0;0;1/tau_1;0];   % input of vehicle 1
B_0_2=[0;0;0;0;0;0;0;1/tau_0];   % leader input
%Vehicle 3
A_3=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_3,0,0,0;0,0,0,0,0,0,0,0,-1/tau_2,0,0;0,0,0,0,0,0,0,0,0,-1/tau_1,0;0,0,0,0,0,0,0,0,0,0,-1/tau_0];
B_3_3=[0;0;0;0;0;0;0;1/tau_3;0;0;0];
B_2_3=[0;0;0;0;0;0;0;0;1/tau_2;0;0];
B_1_3=[0;0;0;0;0;0;0;0;0;1/tau_1;0];
B_0_3=[0;0;0;0;0;0;0;0;0;0;1/tau_0];
%Vehicle 4  (from here on the model uses a window of the 3 preceding vehicles)
A_4=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_4,0,0,0;0,0,0,0,0,0,0,0,-1/tau_3,0,0;0,0,0,0,0,0,0,0,0,-1/tau_2,0;0,0,0,0,0,0,0,0,0,0,-1/tau_1];
B_4_4=[0;0;0;0;0;0;0;1/tau_4;0;0;0];
B_3_4=[0;0;0;0;0;0;0;0;1/tau_3;0;0];
B_2_4=[0;0;0;0;0;0;0;0;0;1/tau_2;0];
B_1_4=[0;0;0;0;0;0;0;0;0;0;1/tau_1];
%Vehicle 5
A_5=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_5,0,0,0;0,0,0,0,0,0,0,0,-1/tau_4,0,0;0,0,0,0,0,0,0,0,0,-1/tau_3,0;0,0,0,0,0,0,0,0,0,0,-1/tau_2];
B_5_5=[0;0;0;0;0;0;0;1/tau_5;0;0;0];
B_4_5=[0;0;0;0;0;0;0;0;1/tau_4;0;0];
B_3_5=[0;0;0;0;0;0;0;0;0;1/tau_3;0];
B_2_5=[0;0;0;0;0;0;0;0;0;0;1/tau_2];
%Vehicle 6
A_6=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_6,0,0,0;0,0,0,0,0,0,0,0,-1/tau_5,0,0;0,0,0,0,0,0,0,0,0,-1/tau_4,0;0,0,0,0,0,0,0,0,0,0,-1/tau_3];
B_6_6=[0;0;0;0;0;0;0;1/tau_6;0;0;0];
B_5_6=[0;0;0;0;0;0;0;0;1/tau_5;0;0];
B_4_6=[0;0;0;0;0;0;0;0;0;1/tau_4;0];
B_3_6=[0;0;0;0;0;0;0;0;0;0;1/tau_3];
%Vehicle 7
A_7=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_7,0,0,0;0,0,0,0,0,0,0,0,-1/tau_6,0,0;0,0,0,0,0,0,0,0,0,-1/tau_5,0;0,0,0,0,0,0,0,0,0,0,-1/tau_4];
B_7_7=[0;0;0;0;0;0;0;1/tau_7;0;0;0];
B_6_7=[0;0;0;0;0;0;0;0;1/tau_6;0;0];
B_5_7=[0;0;0;0;0;0;0;0;0;1/tau_5;0];
B_4_7=[0;0;0;0;0;0;0;0;0;0;1/tau_4];
%Vehicle 8
A_8=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_8,0,0,0;0,0,0,0,0,0,0,0,-1/tau_7,0,0;0,0,0,0,0,0,0,0,0,-1/tau_6,0;0,0,0,0,0,0,0,0,0,0,-1/tau_5];
B_8_8=[0;0;0;0;0;0;0;1/tau_8;0;0;0];
B_7_8=[0;0;0;0;0;0;0;0;1/tau_7;0;0];
B_6_8=[0;0;0;0;0;0;0;0;0;1/tau_6;0];
B_5_8=[0;0;0;0;0;0;0;0;0;0;1/tau_5];
%Vehicle 9
A_9=[0,0,0,-1,1,0,0,0,0,0,0;0,0,0,0,-1,1,0,0,0,0,0;0,0,0,0,0,-1,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,-1/tau_9,0,0,0;0,0,0,0,0,0,0,0,-1/tau_8,0,0;0,0,0,0,0,0,0,0,0,-1/tau_7,0;0,0,0,0,0,0,0,0,0,0,-1/tau_6];
B_9_9=[0;0;0;0;0;0;0;1/tau_9;0;0;0];
B_8_9=[0;0;0;0;0;0;0;0;1/tau_8;0;0];
B_7_9=[0;0;0;0;0;0;0;0;0;1/tau_7;0];
B_6_9=[0;0;0;0;0;0;0;0;0;0;1/tau_6];



%%
%Gains
% Feedback gain vectors K_i (column vectors, same length as x_i). The control
% law is  u_i = K_i' * Q_i + (n_i * tau_i*a_i/(h_i - d_c_{i-1}*ts)) * sigma_i,
% i.e. a state-feedback on the predicted state plus the sigma-term.
% Structure of each K_i:
%   - spacing entries   : scaled by tau_i*a_i/h_i (CTH spacing policy; the first
%                         entry uses h_i minus the predecessor's comm. delay)
%   - speed entries     : tau_i*(b_i) combined with headway-weighted a_i terms
%   - acceleration entries: tau_i*c_i
%   - the "own" entries (-n*tau*(a+b), -n*tau*c) are negative and n = number of
%     predecessors used (1, 2 or 3), so that the gains sum consistently.
K_1=[tau_1*a_1/(h_1-d_c_0*ts);-tau_1*(a_1+b_1);tau_1*b_1;-tau_1*c_1;tau_1*c_1];
K_2=[2*tau_2*a_2/(h_2-d_c_1*ts);tau_2*a_2/(h_2);-2*tau_2*(a_2+b_2);tau_2*b_2-tau_2*a_2*1*(h_1)/(h_2);tau_2*b_2;-2*tau_2*c_2;tau_2*c_2;tau_2*c_2];
K_3=[3*tau_3*a_3/(h_3-d_c_2*ts);2*tau_3*a_3/(h_3);tau_3*a_3/(h_3);-3*tau_3*(a_3+b_3);tau_3*b_3-tau_3*a_3*2*(h_2)/(h_3);tau_3*b_3-tau_3*a_3*1*(h_1)/(h_3);tau_3*b_3;-3*tau_3*c_3;tau_3*c_3;tau_3*c_3;tau_3*c_3];
K_4=[3*tau_4*a_4/(h_4-d_c_3*ts);2*tau_4*a_4/(h_4);tau_4*a_4/(h_4);-3*tau_4*(a_4+b_4);tau_4*b_4-tau_4*a_4*2*(h_3)/(h_4);tau_4*b_4-tau_4*a_4*1*(h_2)/(h_4);tau_4*b_4;-3*tau_4*c_4;tau_4*c_4;tau_4*c_4;tau_4*c_4];
K_5=[3*tau_5*a_5/(h_5-d_c_4*ts);2*tau_5*a_5/(h_5);tau_5*a_5/(h_5);-3*tau_5*(a_5+b_5);tau_5*b_5-tau_5*a_5*2*(h_4)/(h_5);tau_5*b_5-tau_5*a_5*1*(h_3)/(h_5);tau_5*b_5;-3*tau_5*c_5;tau_5*c_5;tau_5*c_5;tau_5*c_5];
K_6=[3*tau_6*a_6/(h_6-d_c_5*ts);2*tau_6*a_6/(h_6);tau_6*a_6/(h_6);-3*tau_6*(a_6+b_6);tau_6*b_6-tau_6*a_6*2*(h_5)/(h_6);tau_6*b_6-tau_6*a_6*1*(h_4)/(h_6);tau_6*b_6;-3*tau_6*c_6;tau_6*c_6;tau_6*c_6;tau_6*c_6];
K_7=[3*tau_7*a_7/(h_7-d_c_6*ts);2*tau_7*a_7/(h_7);tau_7*a_7/(h_7);-3*tau_7*(a_7+b_7);tau_7*b_7-tau_7*a_7*2*(h_6)/(h_7);tau_7*b_7-tau_7*a_7*1*(h_5)/(h_7);tau_7*b_7;-3*tau_7*c_7;tau_7*c_7;tau_7*c_7;tau_7*c_7];
K_8=[3*tau_8*a_8/(h_8-d_c_7*ts);2*tau_8*a_8/(h_8);tau_8*a_8/(h_8);-3*tau_8*(a_8+b_8);tau_8*b_8-tau_8*a_8*2*(h_7)/(h_8);tau_8*b_8-tau_8*a_8*1*(h_6)/(h_8);tau_8*b_8;-3*tau_8*c_8;tau_8*c_8;tau_8*c_8;tau_8*c_8];
K_9=[3*tau_9*a_9/(h_9-d_c_8*ts);2*tau_9*a_9/(h_9);tau_9*a_9/(h_9);-3*tau_9*(a_9+b_9);tau_9*b_9-tau_9*a_9*2*(h_8)/(h_9);tau_9*b_9-tau_9*a_9*1*(h_7)/(h_9);tau_9*b_9;-3*tau_9*c_9;tau_9*c_9;tau_9*c_9;tau_9*c_9];


% (Older gain variant, kept for reference: every headway is reduced by the
%  corresponding communication delay, h_i - d_c*ts.)
% K_1=[tau_1*a_1/(h_1-d_c_0*ts);-tau_1*(a_1+b_1);tau_1*b_1;-tau_1*c_1;tau_1*c_1];
% K_2=[2*tau_2*a_2/(h_2-d_c_1*ts);tau_2*a_2/(h_2-d_c_1*ts);-2*tau_2*(a_2+b_2);tau_2*b_2-tau_2*a_2*1*(h_1-d_c_0*ts)/(h_2-d_c_1*ts);tau_2*b_2;-2*tau_2*c_2;tau_2*c_2;tau_2*c_2];
% K_3=[3*tau_3*a_3/(h_3-d_c_2*ts);2*tau_3*a_3/(h_3-d_c_2*ts);tau_3*a_3/(h_3-d_c_2*ts);-3*tau_3*(a_3+b_3);tau_3*b_3-tau_3*a_3*2*(h_2-d_c_1*ts)/(h_3-d_c_2*ts);tau_3*b_3-tau_3*a_3*1*(h_1-d_c_0*ts)/(h_3-d_c_2*ts);tau_3*b_3;-3*tau_3*c_3;tau_3*c_3;tau_3*c_3;tau_3*c_3];
% K_4=[3*tau_4*a_4/(h_4-d_c_3*ts);2*tau_4*a_4/(h_4-d_c_3*ts);tau_4*a_4/(h_4-d_c_3*ts);-3*tau_4*(a_4+b_4);tau_4*b_4-tau_4*a_4*2*(h_3-d_c_2*ts)/(h_4-d_c_3*ts);tau_4*b_4-tau_4*a_4*1*(h_2-d_c_1*ts)/(h_4-d_c_3*ts);tau_4*b_4;-3*tau_4*c_4;tau_4*c_4;tau_4*c_4;tau_4*c_4];
% K_5=[3*tau_5*a_5/(h_5-d_c_4*ts);2*tau_5*a_5/(h_5-d_c_4*ts);tau_5*a_5/(h_5-d_c_4*ts);-3*tau_5*(a_5+b_5);tau_5*b_5-tau_5*a_5*2*(h_4-d_c_3*ts)/(h_5-d_c_4*ts);tau_5*b_5-tau_5*a_5*1*(h_3-d_c_2*ts)/(h_5-d_c_4*ts);tau_5*b_5;-3*tau_5*c_5;tau_5*c_5;tau_5*c_5;tau_5*c_5];
% K_6=[3*tau_6*a_6/(h_6-d_c_5*ts);2*tau_6*a_6/(h_6-d_c_5*ts);tau_6*a_6/(h_6-d_c_5*ts);-3*tau_6*(a_6+b_6);tau_6*b_6-tau_6*a_6*2*(h_5-d_c_4*ts)/(h_6-d_c_5*ts);tau_6*b_6-tau_6*a_6*1*(h_4-d_c_3*ts)/(h_6-d_c_5*ts);tau_6*b_6;-3*tau_6*c_6;tau_6*c_6;tau_6*c_6;tau_6*c_6];
% K_7=[3*tau_7*a_7/(h_7-d_c_6*ts);2*tau_7*a_7/(h_7-d_c_6*ts);tau_7*a_7/(h_7-d_c_6*ts);-3*tau_7*(a_7+b_7);tau_7*b_7-tau_7*a_7*2*(h_6-d_c_5*ts)/(h_7-d_c_6*ts);tau_7*b_7-tau_7*a_7*1*(h_5-d_c_4*ts)/(h_7-d_c_6*ts);tau_7*b_7;-3*tau_7*c_7;tau_7*c_7;tau_7*c_7;tau_7*c_7];
% K_8=[3*tau_8*a_8/(h_8-d_c_7*ts);2*tau_8*a_8/(h_8-d_c_7*ts);tau_8*a_8/(h_8-d_c_7*ts);-3*tau_8*(a_8+b_8);tau_8*b_8-tau_8*a_8*2*(h_7-d_c_6*ts)/(h_8-d_c_7*ts);tau_8*b_8-tau_8*a_8*1*(h_6-d_c_5*ts)/(h_8-d_c_7*ts);tau_8*b_8;-3*tau_8*c_8;tau_8*c_8;tau_8*c_8;tau_8*c_8];
% K_9=[3*tau_9*a_9/(h_9-d_c_8*ts);2*tau_9*a_9/(h_9-d_c_8*ts);tau_9*a_9/(h_9-d_c_8*ts);-3*tau_9*(a_9+b_9);tau_9*b_9-tau_9*a_9*2*(h_8-d_c_7*ts)/(h_9-d_c_8*ts);tau_9*b_9-tau_9*a_9*1*(h_7-d_c_6*ts)/(h_9-d_c_8*ts);tau_9*b_9;-3*tau_9*c_9;tau_9*c_9;tau_9*c_9;tau_9*c_9];


%%
%Leader's maneuver
% Piecewise-constant leader acceleration command (indices are samples, 1 sample = 0.01 s):
%   t = 25.0 - 27.5 s : u_L = -4 m/s^2 (hard braking)
%   t = 27.5 - 50.0 s : u_L =  0       (cruise)
%   t = 50.0 - 55.0 s : u_L = +2 m/s^2 (acceleration)
%   t >= 55.0 s       : u_L =  0
for i= d+max_d_c+1:length(t)

   if (i>=2500)
     U_0(1,i)=-4;  
   end
   if (i>=2750)
     U_0(1,i)=0;  
   end
    if (i>=5000)
     U_0(1,i)=2;  
    end
   if (i>=5500)
     U_0(1,i)=0;  
   end
end

%%
% ========================= Vehicle 1 (follows the leader) =========================
% Each follower's loop is run over the complete time horizon one vehicle at a time
% (vehicle 1 first, then 2, ...). This works because vehicle i only needs the
% (already computed) input histories of its predecessors.
for i= d+max_d_c+1:length(t)

%Predictor_based design 
% Predictor: Q(t) = e^{A D} x_delayed(t) + int_{t-D}^{t} e^{A(t-theta)} B u(theta) dtheta,
% which gives the state D seconds ahead and cancels the input delay D.

    int1=zeros(5,1);   % value of the convolution integral

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);                 % sample grid of the integration window [i-d, i]
    integrand_values_1_1=zeros(5, length(theta_vals_1_1));  % integrand evaluated on that grid
    z_1_1=length(theta_vals_1_1);                           % write index (filled backwards)
    %Vehicle 1 
    
    % Evaluate e^{A(i-j)ts}*B*u(j)*ts for every sample j in the window.
    % The leader's input is available with communication delay d_c_0; the vehicle's own
    % input U_1 is used without communication delay.
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_1*(i-j)*ts) * ( ( B_0_1*U_0(1,j-d_c_0) ) + (B_1_1*U_1(1,j)) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int1 = trapz(theta_vals_1_1, integrand_values_1_1,2);   % trapezoidal integration along time (dim 2)

    %Vehicle 1
    % Predicted state: own measurements are current, the leader's speed and acceleration
    % (components 3 and 5) arrive with delay d_c_0.
    Q_1(:,i)=expm(A_1*D)*[x_1(1,i);x_1(2,i);x_1(3,i-d_c_0);x_1(4,i);x_1(5,i-d_c_0)]+ int1;

    %sigma 
    % Initial value of sigma: minus the integral of the predecessor speed over the
    % last d_c_0 samples of the initial history (so that sigma starts consistent with
    % the delayed spacing).
    sigma_zero_1=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_0
   
     sigma_zero_1=sigma_zero_1+x_1(3,l);

    end
    sigma_1(1,d+max_d_c+1)=-sigma_zero_1*ts;
    % Euler update of sigma' = v_L(t-d_c) - v_L(t)
    sigma_1(1,i+1)= sigma_1(1,i)+ts*(x_1(3,i-d_c_0) - x_1(3,i));
   
    %control law
    %Vehicle 1
    % Predictor feedback + weighted sigma-term
    U_1(1,i)= K_1'*Q_1(:,i)+ 1*(tau_1*a_1/(h_1-d_c_0*ts))*(sigma_1(1,i));
 

      %%
   %States definitions  

    %Vehicle 1
    % Plant update (Euler): inputs enter with the actuation delay d.
    x_1(:,i+1)=x_1(:,i)+ts*(A_1*x_1(:,i)+B_1_1*U_1(1,i-d)+B_0_1*U_0(1,i-d));


end



%%
%The same steps for the other vehicles 
% Pattern for every follower: (1) predictor integral, (2) predicted state Q,
% (3) sigma update, (4) control law, (5) Euler plant update with delayed input.


% ========================= Vehicle 2 =========================
% Uses information of the leader (delayed by d_c_0) and of vehicle 1 (delayed by d_c_1).

for i= d+max_d_c+1:length(t)






   %Predictor_based design 

    int2=zeros(8,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(8, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 2 
    
    % Integrand: leader input (delay d_c_0), vehicle 1 input (delay d_c_1), own input (no delay)
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_2*(i-j)*ts) * ( ( B_0_2*U_0(1,j-d_c_0) ) + (B_1_2*U_1(1,j-d_c_1))  +  B_2_2*U_2(1,j)  )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int2 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 2
    % Own states (s2, v2, a2) are current; states received from vehicle 1 (s1, v1, a1)
    % are delayed by d_c_1; leader's (vL, aL) by d_c_0.
    Q_2(:,i)=expm(A_2*D)*[x_2(1,i);x_2(2,i-d_c_1);x_2(3,i);x_2(4,i-d_c_1);x_2(5,i-d_c_0);x_2(6,i);x_2(7,i-d_c_1);x_2(8,i-d_c_0)]+ int2;
   

    %sigma 
    % Uses the speed of vehicle 1 (state 4) and its delay d_c_1.
    sigma_zero_2=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_1
   
     sigma_zero_2=sigma_zero_2+x_2(4,l);

    end
    sigma_2(1,d+max_d_c+1)=-sigma_zero_2*ts;
    sigma_2(1,i+1)= sigma_2(1,i)+ts*(x_2(4,i-d_c_1) - x_2(4,i));
   
    %control law

    %Vehicle 2
    % Factor 2 = number of predecessors used by vehicle 2.
    U_2(1,i)= K_2'*Q_2(:,i)+ 2*(tau_2*a_2/(h_2-d_c_1*ts))*(sigma_2(1,i));
 

      %%
   %States definitions  

    %Vehicle 2
    x_2(:,i+1)=x_2(:,i)+ts*(A_2*x_2(:,i)+B_0_2*U_0(1,i-d)+B_1_2*U_1(1,i-d)+B_2_2*U_2(1,i-d));



end

% ========================= Vehicle 3 =========================
% Uses leader (d_c_0), vehicle 1 (d_c_1) and vehicle 2 (d_c_2).

for i= d+max_d_c+1:length(t)


 %Predictor_based design 

    int3=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 3 
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_3*(i-j)*ts) * ( ( B_0_3*U_0(1,j-d_c_0) ) + (B_1_3*U_1(1,j-d_c_1))  +  B_2_3*U_2(1,j-d_c_2) +  B_3_3*U_3(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int3 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 3
    % Each component is delayed by the communication delay of the vehicle that
    % transmits it (own: none, vehicle 2: d_c_2, vehicle 1: d_c_1, leader: d_c_0).
    Q_3(:,i)=expm(A_3*D)*[x_3(1,i);x_3(2,i-d_c_2);x_3(3,i-d_c_1);x_3(4,i);x_3(5,i-d_c_2);x_3(6,i-d_c_1);x_3(7,i-d_c_0);x_3(8,i);x_3(9,i-d_c_2);x_3(10,i-d_c_1);x_3(11,i-d_c_0)]+ int3;
   
    
    %sigma 
    % Based on the speed of vehicle 2 (state 5) and delay d_c_2.
    sigma_zero_3=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_2
   
     sigma_zero_3=sigma_zero_3+x_3(5,l);

    end
    sigma_3(1,d+max_d_c+1)=-sigma_zero_3*ts;
    sigma_3(1,i+1)= sigma_3(1,i)+ts*(x_3(5,i-d_c_2) - x_3(5,i));
   
    %control law

    %Vehicle 3
    % Factor 3 = number of predecessors used.
    U_3(1,i)= K_3'*Q_3(:,i)+ 3*(tau_3*a_3/(h_3-d_c_2*ts))*(sigma_3(1,i));
 

      %%
   %States definitions  

    %Vehicle 3
    x_3(:,i+1)=x_3(:,i)+ts*(A_3*x_3(:,i)+B_0_3*U_0(1,i-d)+B_1_3*U_1(1,i-d)+B_2_3*U_2(1,i-d)+B_3_3*U_3(1,i-d));



end

% ========================= Vehicle 4 =========================
% Uses vehicles 1, 2, 3 (delays d_c_1, d_c_2, d_c_3); the leader is no longer in its window.
for i= d+max_d_c+1:length(t)


 %Predictor_based design 

    int4=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 4
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_4*(i-j)*ts) * ( ( B_1_4*U_1(1,j-d_c_1) ) + (B_2_4*U_2(1,j-d_c_2))  +  B_3_4*U_3(1,j-d_c_3) +  B_4_4*U_4(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int4 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 4
    Q_4(:,i)=expm(A_4*D)*[x_4(1,i);x_4(2,i-d_c_3);x_4(3,i-d_c_2);x_4(4,i);x_4(5,i-d_c_3);x_4(6,i-d_c_2);x_4(7,i-d_c_1);x_4(8,i);x_4(9,i-d_c_3);x_4(10,i-d_c_2);x_4(11,i-d_c_1)]+ int4;
   
    %sigma 
    % Speed of vehicle 3 (state 5), delay d_c_3.
    sigma_zero_4=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_3
   
     sigma_zero_4=sigma_zero_4+x_4(5,l);

    end
    sigma_4(1,d+max_d_c+1)=-sigma_zero_4*ts;
    sigma_4(1,i+1)= sigma_4(1,i)+ts*(x_4(5,i-d_c_3) - x_4(5,i));


    %control law

    %Vehicle 4
    U_4(1,i)= K_4'*Q_4(:,i)+ 3*(tau_4*a_4/(h_4-d_c_3*ts))*(sigma_4(1,i));
 

      %%
   %States definitions  

    %Vehicle 4
    x_4(:,i+1)=x_4(:,i)+ts*(A_4*x_4(:,i)+B_1_4*U_1(1,i-d)+B_2_4*U_2(1,i-d)+B_3_4*U_3(1,i-d)+B_4_4*U_4(1,i-d));




end

% ========================= Vehicle 5 =========================
% Uses vehicles 2, 3, 4 (delays d_c_2, d_c_3, d_c_4).
for i= d+max_d_c+1:length(t)

 %Predictor_based design 

    int5=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 5
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_5*(i-j)*ts) * ( ( B_2_5*U_2(1,j-d_c_2) ) + (B_3_5*U_3(1,j-d_c_3))  +  B_4_5*U_4(1,j-d_c_4) +  B_5_5*U_5(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int5 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 5
    Q_5(:,i)=expm(A_5*D)*[x_5(1,i);x_5(2,i-d_c_4);x_5(3,i-d_c_3);x_5(4,i);x_5(5,i-d_c_4);x_5(6,i-d_c_3);x_5(7,i-d_c_2);x_5(8,i);x_5(9,i-d_c_4);x_5(10,i-d_c_3);x_5(11,i-d_c_2)]+ int5;
   
   %sigma 
   % Speed of vehicle 4 (state 5), delay d_c_4.
    sigma_zero_5=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_4
   
     sigma_zero_5=sigma_zero_5+x_5(5,l);

    end
    sigma_5(1,d+max_d_c+1)=-sigma_zero_5*ts;
    sigma_5(1,i+1)= sigma_5(1,i)+ts*(x_5(5,i-d_c_4) - x_5(5,i));
    %control law

    %Vehicle 5
    U_5(1,i)= K_5'*Q_5(:,i)+ 3*(tau_5*a_5/(h_5-d_c_4*ts))*(sigma_5(1,i));
 

      %%
   %States definitions  

    %Vehicle 5
    x_5(:,i+1)=x_5(:,i)+ts*(A_5*x_5(:,i)+B_2_5*U_2(1,i-d)+B_3_5*U_3(1,i-d)+B_4_5*U_4(1,i-d)+B_5_5*U_5(1,i-d));




end

% ========================= Vehicle 6 =========================
% Uses vehicles 3, 4, 5 (delays d_c_3, d_c_4, d_c_5).
for i= d+max_d_c+1:length(t)

 %Predictor_based design 

    int6=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 6
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_6*(i-j)*ts) * ( ( B_3_6*U_3(1,j-d_c_3) ) + (B_4_6*U_4(1,j-d_c_4))  +  B_5_6*U_5(1,j-d_c_5) +  B_6_6*U_6(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int6 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 6
    Q_6(:,i)=expm(A_6*D)*[x_6(1,i);x_6(2,i-d_c_5);x_6(3,i-d_c_4);x_6(4,i);x_6(5,i-d_c_5);x_6(6,i-d_c_4);x_6(7,i-d_c_3);x_6(8,i);x_6(9,i-d_c_5);x_6(10,i-d_c_4);x_6(11,i-d_c_3)]+ int6;
   
     %sigma 
     % Speed of vehicle 5 (state 5), delay d_c_5.
    sigma_zero_6=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_5
   
     sigma_zero_6=sigma_zero_6+x_6(5,l);

    end
    sigma_6(1,d+max_d_c+1)=-sigma_zero_6*ts;
    sigma_6(1,i+1)= sigma_6(1,i)+ts*(x_6(5,i-d_c_5) - x_6(5,i));
   
    %control law

    %Vehicle 6
    U_6(1,i)= K_6'*Q_6(:,i)+ 3*(tau_6*a_6/(h_6-d_c_5*ts))*(sigma_6(1,i));
 

      %%
   %States definitions  

    %Vehicle 6
    x_6(:,i+1)=x_6(:,i)+ts*(A_6*x_6(:,i)+B_3_6*U_3(1,i-d)+B_4_6*U_4(1,i-d)+B_5_6*U_5(1,i-d)+B_6_6*U_6(1,i-d));

end

% ========================= Vehicle 7 =========================
% Uses vehicles 4, 5, 6 (delays d_c_4, d_c_5, d_c_6).
for i= d+max_d_c+1:length(t)

    int7=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 7
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_7*(i-j)*ts) * ( ( B_4_7*U_4(1,j-d_c_4) ) + (B_5_7*U_5(1,j-d_c_5))  +  B_6_7*U_6(1,j-d_c_6) +  B_7_7*U_7(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int7 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 7
    Q_7(:,i)=expm(A_7*D)*[x_7(1,i);x_7(2,i-d_c_6);x_7(3,i-d_c_5);x_7(4,i);x_7(5,i-d_c_6);x_7(6,i-d_c_5);x_7(7,i-d_c_4);x_7(8,i);x_7(9,i-d_c_6);x_7(10,i-d_c_5);x_7(11,i-d_c_4)]+ int7;
   
    %sigma 
    % Speed of vehicle 6 (state 5), delay d_c_6.
    sigma_zero_7=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_6
   
     sigma_zero_7=sigma_zero_7+x_7(5,l);

    end
    sigma_7(1,d+max_d_c+1)=-sigma_zero_7*ts;
    sigma_7(1,i+1)= sigma_7(1,i)+ts*(x_7(5,i-d_c_6) - x_7(5,i));

    %control law

    %Vehicle 7
    U_7(1,i)= K_7'*Q_7(:,i)+ 3*(tau_7*a_7/(h_7-d_c_6*ts))*(sigma_7(1,i));
 

      %%
   %States definitions  

    %Vehicle 7
    x_7(:,i+1)=x_7(:,i)+ts*(A_7*x_7(:,i)+B_4_7*U_4(1,i-d)+B_5_7*U_5(1,i-d)+B_6_7*U_6(1,i-d)+B_7_7*U_7(1,i-d));

end

% ========================= Vehicle 8 =========================
% Uses vehicles 5, 6, 7 (delays d_c_5, d_c_6, d_c_7).

for i= d+max_d_c+1:length(t)

int8=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 8
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_8*(i-j)*ts) * ( ( B_5_8*U_5(1,j-d_c_5) ) + (B_6_8*U_6(1,j-d_c_6))  +  B_7_8*U_7(1,j-d_c_7) +  B_8_8*U_8(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int8 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 8
    Q_8(:,i)=expm(A_8*D)*[x_8(1,i);x_8(2,i-d_c_7);x_8(3,i-d_c_6);x_8(4,i);x_8(5,i-d_c_7);x_8(6,i-d_c_6);x_8(7,i-d_c_5);x_8(8,i);x_8(9,i-d_c_7);x_8(10,i-d_c_6);x_8(11,i-d_c_5)]+ int8;
   
    
    %sigma 
    % Speed of vehicle 7 (state 5), delay d_c_7.
    sigma_zero_8=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_7
   
     sigma_zero_8=sigma_zero_8+x_8(5,l);

    end
    sigma_8(1,d+max_d_c+1)=-sigma_zero_8*ts;
    sigma_8(1,i+1)= sigma_8(1,i)+ts*(x_8(5,i-d_c_7) - x_8(5,i));


    %control law

    %Vehicle 8
    U_8(1,i)= K_8'*Q_8(:,i)+ 3*(tau_8*a_8/(h_8-d_c_7*ts))*(sigma_8(1,i));
 

      %%
   %States definitions  

    %Vehicle 8
    x_8(:,i+1)=x_8(:,i)+ts*(A_8*x_8(:,i)+B_5_8*U_5(1,i-d)+B_6_8*U_6(1,i-d)+B_7_8*U_7(1,i-d)+B_8_8*U_8(1,i-d));


end

% ========================= Vehicle 9 (last vehicle) =========================
% Uses vehicles 6, 7, 8 (delays d_c_6, d_c_7, d_c_8).
for i= d+max_d_c+1:length(t)


int9=zeros(11,1);

%Predictor_based
    theta_vals_1_1=linspace(i - d, i, d+1);
    integrand_values_1_1=zeros(11, length(theta_vals_1_1));
    z_1_1=length(theta_vals_1_1);
    %Vehicle 9
    
    for j=i:-1:i-d
    integrand_values_1_1(:, z_1_1) = expm(A_9*(i-j)*ts) * ( ( B_6_9*U_6(1,j-d_c_6) ) + (B_7_9*U_7(1,j-d_c_7))  +  B_8_9*U_8(1,j-d_c_8) +  B_9_9*U_9(1,j) )*ts;
    z_1_1 = z_1_1 - 1;
    end
    int9 = trapz(theta_vals_1_1, integrand_values_1_1,2);

    %Vehicle 9
    Q_9(:,i)=expm(A_9*D)*[x_9(1,i);x_9(2,i-d_c_8);x_9(3,i-d_c_7);x_9(4,i);x_9(5,i-d_c_8);x_9(6,i-d_c_7);x_9(7,i-d_c_6);x_9(8,i);x_9(9,i-d_c_8);x_9(10,i-d_c_7);x_9(11,i-d_c_6)]+ int9;
   
    %sigma 
    % Speed of vehicle 8 (state 5), delay d_c_8.
    sigma_zero_9=0;
    for l=d+max_d_c+1:-1:d+max_d_c+1-d_c_8
   
     sigma_zero_9=sigma_zero_9+x_9(5,l);

    end
    sigma_9(1,d+max_d_c+1)=-sigma_zero_9*ts;
    sigma_9(1,i+1)= sigma_9(1,i)+ts*(x_9(5,i-d_c_8) - x_9(5,i));
   
    %control law

    %Vehicle 9
    U_9(1,i)= K_9'*Q_9(:,i)+ 3*(tau_9*a_9/(h_9-d_c_8*ts))*(sigma_9(1,i));
 

      %%
   %States definitions  

    %Vehicle 9
    x_9(:,i+1)=x_9(:,i)+ts*(A_9*x_9(:,i)+B_6_9*U_6(1,i-d)+B_7_9*U_7(1,i-d)+B_8_9*U_8(1,i-d)+B_9_9*U_9(1,i-d));




end




%%
%Simulation
% Post-processing: shift the stored signals left by d+max_d_c samples so that the
% artificial initial history window is dropped and plots start at t = 0.
% Only the signals that are plotted (own spacing, own speed, own acceleration, own
% input) are shifted. The leader's signals are taken from vehicle 1's state.

for w=1:1:length(t_t)

%vehicle 0   
x_1(3,w)=x_1(3,w+d+max_d_c);   % leader speed v_L
x_1(5,w)=x_1(5,w+d+max_d_c);   % leader acceleration a_L
U_0(1,w)=U_0(1,w+d+max_d_c);   % leader input u_L

%vehicle 1
x_1(1,w)=x_1(1,w+d+max_d_c);   % s_1
x_1(2,w)=x_1(2,w+d+max_d_c);   % v_1
x_1(4,w)=x_1(4,w+d+max_d_c);   % a_1
U_1(1,w)=U_1(1,w+d+max_d_c);   % u_1

%vehicle 2
x_2(1,w)=x_2(1,w+d+max_d_c);   % s_2
x_2(3,w)=x_2(3,w+d+max_d_c);   % v_2
x_2(6,w)=x_2(6,w+d+max_d_c);   % a_2
U_2(1,w)=U_2(1,w+d+max_d_c);

%vehicle 3
x_3(1,w)=x_3(1,w+d+max_d_c);   % s_3
x_3(4,w)=x_3(4,w+d+max_d_c);   % v_3
x_3(8,w)=x_3(8,w+d+max_d_c);   % a_3
U_3(1,w)=U_3(1,w+d+max_d_c);

%vehicle 4
x_4(1,w)=x_4(1,w+d+max_d_c);
x_4(4,w)=x_4(4,w+d+max_d_c);
x_4(8,w)=x_4(8,w+d+max_d_c);
U_4(1,w)=U_4(1,w+d+max_d_c);

%vehicle 5
x_5(1,w)=x_5(1,w+d+max_d_c);
x_5(4,w)=x_5(4,w+d+max_d_c);
x_5(8,w)=x_5(8,w+d+max_d_c);
U_5(1,w)=U_5(1,w+d+max_d_c);

%vehicle 6
x_6(1,w)=x_6(1,w+d+max_d_c);
x_6(4,w)=x_6(4,w+d+max_d_c);
x_6(8,w)=x_6(8,w+d+max_d_c);
U_6(1,w)=U_6(1,w+d+max_d_c);

%vehicle 7
x_7(1,w)=x_7(1,w+d+max_d_c);
x_7(4,w)=x_7(4,w+d+max_d_c);
x_7(8,w)=x_7(8,w+d+max_d_c);
U_7(1,w)=U_7(1,w+d+max_d_c);

%vehicle 8
x_8(1,w)=x_8(1,w+d+max_d_c);
x_8(4,w)=x_8(4,w+d+max_d_c);
x_8(8,w)=x_8(8,w+d+max_d_c);
U_8(1,w)=U_8(1,w+d+max_d_c);

% NOTE: there is no shift block for vehicle 9 (x_9, U_9), so its plotted curves
% are not time-aligned with the others (they still contain the initial history).

end



%%
%Spacing plots 
% Spacing of all nine followers (state 1 of each x_i), over the trimmed time vector t_t.

figure
plot(t_t,x_1(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,x_2(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_3(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_4(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_5(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_6(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_7(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_8(1,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_9(1,1:length(t_t)),'LineWidth',2)

legend('${\it} s_1(t)$','${\it} s_2(t)$','${\it} s_3(t)$','${\it} s_4(t)$','${\it} s_5(t)$','${\it} s_6(t)$','${\it} s_7(t)$','${\it} s_8(t)$','${\it} s_9(t)$','Interpreter','Latex','Location','northeast')
xlabel('${\it} t(s)$','Interpreter','Latex') 
ylabel('${\it} s_i(t)\nobreakspace(m)$','Interpreter','Latex') 


%%
%velocity plots 
% Speed index inside each x_i: leader vL = x_1(3), v1 = x_1(2), v2 = x_2(3),
% v_i = x_i(4) for i = 3..9.
figure
plot(t_t,x_1(3,1:length(t_t)),'LineWidth',2)   % leader
hold on
plot(t_t,x_1(2,1:length(t_t)),'LineWidth',2)   % vehicle 1
hold on
plot(t_t,x_2(3,1:length(t_t)),'LineWidth',2)   % vehicle 2
hold on 
plot(t_t,x_3(4,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,x_4(4,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_5(4,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_6(4,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_7(4,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,x_8(4,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_9(4,1:length(t_t)),'LineWidth',2)
legend('${\it} v_{\rm L}(t)$','${\it} v_1(t)$','${\it} v_2(t)$','${\it} v_3(t)$','${\it} v_4(t)$','${\it} v_5(t)$','${\it} v_6(t)$','${\it} v_7(t)$','${\it} v_8(t)$','${\it} v_9(t)$','Interpreter','Latex','Location','northeast')
xlabel('${\it} t(s)$','Interpreter','Latex') 
ylabel('${\it} v_i(t)\nobreakspace(\frac{m}{s})$','Interpreter','Latex')

%%
%acceleration plots 
% Acceleration index inside each x_i: aL = x_1(5), a1 = x_1(4), a2 = x_2(6),
% a_i = x_i(8) for i = 3..9.
figure
plot(t_t,x_1(5,1:length(t_t)),'LineWidth',2)   % leader
hold on
plot(t_t,x_1(4,1:length(t_t)),'LineWidth',2)   % vehicle 1
hold on
plot(t_t,x_2(6,1:length(t_t)),'LineWidth',2)   % vehicle 2
hold on 
plot(t_t,x_3(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_4(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_5(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_6(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_7(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_8(8,1:length(t_t)),'LineWidth',2)
hold on 
plot(t_t,x_9(8,1:length(t_t)),'LineWidth',2)
legend('${\it} a_{\rm L}(t)$','${\it} a_1(t)$','${\it} a_2(t)$','${\it} a_3(t)$','${\it} a_4(t)$','${\it} a_5(t)$','${\it} a_6(t)$','${\it} a_7(t)$','${\it} a_8(t)$','${\it} a_9(t)$','Interpreter','Latex','Location','southeast')
xlabel('${\it} t(s)$','Interpreter','Latex') 
ylabel('${\it} a_i(t)\nobreakspace(\frac{m}{s^2})$','Interpreter','Latex')


%%
%control input plots 
% Control inputs of the leader (U_0) and of all followers (U_1 ... U_9).
figure
plot(t_t,U_0(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_1(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_2(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_3(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_4(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_5(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_6(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_7(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_8(1,1:length(t_t)),'LineWidth',2)
hold on
plot(t_t,U_9(1,1:length(t_t)),'LineWidth',2)
legend('${\it} u_{\rm L}(t)$','${\it} u_1(t)$','${\it} u_2(t)$','${\it} u_3(t)$','${\it} u_4(t)$','${\it} u_5(t)$','${\it} u_6(t)$','${\it} u_7(t)$','${\it} u_8(t)$','${\it} u_9(t)$','Interpreter','Latex','Location','southeast')
xlabel('${\it} t(s)$','Interpreter','Latex') 
ylabel('${\it}  u_i(t)\nobreakspace (\frac{m}{s^2})$','Interpreter','Latex')