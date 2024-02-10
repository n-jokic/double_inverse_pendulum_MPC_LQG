%% MPC
addpath('C:\Users\Ivona\Desktop\CASADI\casadi-3.6.4-windows64-matlab2018b')
load('parameters.mat');

% stanja
x1 = casadi.SX.sym('x1');
x2 = casadi.SX.sym('x2');
x3 = casadi.SX.sym('x3');
x4 = casadi.SX.sym('x4');
x5 = casadi.SX.sym('x5');

% vektor stanja
states = [x1;x2;x3;x4;x5];

% upravljanje
u = casadi.SX.sym('u');
control = u;

state_transition = casadi.Function('xdot', ...
    {states, control}, ...
    {[ states(2);...
       (-J2*(Ka1*states(2) - Kf*states(5) + states(4)*(Lcm2*Le1*m2*states(4) + 2*J2*states(2)*cos(states(3)))*sin(states(3)))...
        + Lcm2*Le1*m2*cos(states(3))*(-Ka2*states(4) + (g*Lcm2*m2+J2*states(2)^2*cos(states(3)))*sin(states(3))))/...
        (-Lcm2^2*Le1^2*m2^2*cos(states(3))^2 + J2*(J0+J2*sin(states(3))^2));...
       states(4);
       (Ka2*states(4) - g*Lcm2*m2*sin(states(3))*(J0+J2*sin(states(3))^2)+...
        cos(states(3))*((-J0*J2*states(2)^2 + Lcm2^2*Le1^2*m2^2*states(4)^2)*sin(states(3))-...
        J2^2*states(2)^2*sin(states(3))^3 + Lcm2*Le1*m2*(Ka1*states(2) - Kf*states(5) + J2*states(2)*states(4)*sin(2*states(3)))))/...
        (Lcm2^2*Le1^2*m2^2*cos(states(3))^2 - J2*(J0 + J2*sin(states(3))^2));
        (-Kt*states(2) - R*states(5) + u)/Lb
    ] ...
    } ...
    , {'states', 'control'}, {'dot_states'});
%%
h = 0.05; %step size in seconds [s]
N = 5; % prediction horizon
Nc = 2;
iter_num = casadi.SX.sym('iter_num');
reference = casadi.SX.sym('reference', 2, 1);

n_controls = length(control);
state_transition; %nonlinear mapping (state_t, control_t)->state_t+1
n_states = length(states);
n_ref = 2; % number of reference signals

% weighing matrix (states)
Q = zeros(n_ref,n_ref); 
Q(1,1) = 0.1; Q(2,2) = 1;

% weighing matrix (controls)
R = 0.5;

ref_states = zeros(n_ref, n_states);
ref_states(1, 1) = 1; ref_states(2, 3) = 1; 

J = (ref_states*states-reference)'*Q*(ref_states*states-reference) ...
            + control'*R*control*(iter_num<=Nc);

% Continuous time dynamics
f = casadi.Function('f', {states, control, reference, ...
    iter_num}, ...
    {state_transition(states, control), J});
%%
% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = 1; % RK4 steps per interval
DT = h/N/M;
X0 = casadi.MX.sym('X0', n_states);
U = casadi.MX.sym('U', n_controls);
Iter  = casadi.MX.sym('Iter_num');
Ref = casadi.MX.sym('Ref', 2);
X = X0;
Q = 0;
for j=1:M
  
   sol = f(X, U, Ref, Iter);
   X = X + h*sol(1);
   Q = Q + h*sol(2);
    % g = [g;st_next-st_next_euler]; % compute constraints
end

F = casadi.Function('F', {X0, U, Ref,Iter}, ...
    {X, Q}, {'x0', 'u', 'reference','iter_num'}, ...
    {'xf', 'qf'});
%%
% Decision variables (controls)
U = casadi.MX.sym('U',n_controls,N); 

% parameters (which include the initial state and the reference state)
P = casadi.MX.sym('P',n_ref + n_states);

% A vector that represents the states over the optimization problem.
X = casadi.MX.sym('X',n_states,(N+1));

obj = 0; % Objective function
g = [];  % constraints vector


st  = X(:,1); % initial state
g = [g;st-P(n_ref+1:n_ref+n_states)]; % initial condition constraints
slip = casadi.MX(P(n_ref+n_states+1:end));
ref = casadi.MX(P(1:n_ref));

for k = 1:N
    st = casadi.MX(X(:,k));  con =  casadi.MX(U(:,k));    
    Xk =  casadi.MX(X(:,k+1));
   
    Fk = F('x0', st, 'u', con, ...
        'reference', ref, 'iter_num',k);
    Xk_end = Fk.xf;
    obj = Fk.qf;

    
    g = [g; Xk-Xk_end];

    if k~= N && k>Nc -1
        Uk = U(:,k+1);
        g = [g; Uk-U(:, Nc)];
    end
end
%%
% make the decision variable one column  vector
OPT_variables = [reshape(X,(N+1)*n_states,1);reshape(U,(N)*n_controls,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% solver setup:
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = casadi.nlpsol('solver', 'ipopt', nlp_prob, opts);

%%
args = struct;

args.lbg(1:n_states*(N+1)+n_controls*(N-Nc)) = 1e-20;  % Equality constraints
args.ubg(1:n_states*(N+1)+n_controls*(N-Nc)) = 1e-20;   % Equality constraints

for i = 1 : n_states

    args.lbx(i:n_states:n_states*(N+1),1) = -2; %state x lower bound
    args.ubx(i:n_states:n_states*(N+1),1) = 2; %state x upper bound
end

for i = 1 : n_controls
    args.lbx(n_states*(N+1)+i:n_controls:n_states*(N+1)+n_controls*(N),1) = -5; %v lower bound
    args.ubx(n_states*(N+1)+i:n_controls:n_states*(N+1)+n_controls*(N),1) = 5; %v upper bound
end
%%
reference_trajecotry = casadi.MX.sym('reference_trajecotry', 2);
state_measurement = casadi.MX.sym('state_measurement', n_states);
control_measurement = casadi.MX.sym('control_measurement', n_controls);

args.p = [reference_trajecotry; state_measurement];
args.x0  = [];
for i = 1 : N+1
    args.x0 = [args.x0; state_measurement];
end
for i = 1 : N
    args.x0 = [args.x0; control_measurement];
end


ubw_sym = casadi.MX(args.ubx);
lbw_sym = casadi.MX(args.lbx);

lbw_sym(1:n_states) = state_measurement;
ubw_sym(1:n_states) = state_measurement;

sol_sym = solver('x0', args.x0, 'lbx', lbw_sym, 'ubx', ubw_sym,...
            'lbg', args.lbg, 'ubg', args.ubg);


function_name = 'mpc';
mpc = casadi.Function(function_name,{state_measurement, control_measurement ...
    reference_trajecotry},{sol_sym.x(n_states*(N+1)+1:(N+1)*n_states+n_controls)});

%%
tic
u = mpc(ones(5, 1), 0, ones(2,1)*0.1);
toc