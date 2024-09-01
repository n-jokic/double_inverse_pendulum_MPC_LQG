classdef MPC_swing_up < handle
    properties
        % CASADI compatible state transition function
        state_transition
        
        % state dynamics model
        state_dynamics = @inverted_pendulum;
        
        % norm for control
        u_norm = 2;
        x_norm = 2;

        % Prediction setup:
        M_rk4_iter
        dt
        N

        %Number of states:
        n_controls = 1;
        n_states = 5;
        n_reference = 5;

        % Weighting matricies
        Qw = [(1/pi*180)^2, 0, 0, 0, 0; 
               0, (1/2)^2, 0, 0, 0;
               0, 0, (1/pi*180)^2, 0, 0;
               0, 0, 0, (1/2)^2, 0 
               0, 0, 0, 0, 1];
        Rw = 1/36;
        Ow = [1, 0, 0, 0, 0; 0, 0, 1, 0, 0];
        Qwobs = [(1/pi*180)^2, 0, 0, 0, 0; 
               0, (1/2)^2, 0, 0, 0;
               0, 0, (1/pi*180)^2, 0, 0;
               0, 0, 0, (1/2)^2, 0 
               0, 0, 0, 0, 1];
        gamma_control = 1;
        gamma_final = 10;

        % NLP setup
        max_iter = 6000;
        acceptable_tol = 5e-4;
        acceptable_obj_change_tol = 1e-5;
   
        % MPC solver 
        solver

        % upper/lower bounds
        args
        
        % control signal constraints
        u_max
        u_min

        % initial trajectory for MPC
        x0
        % initial state + reference state 
        p

        % MPC full outputs
        u_mpc
        x_mpc

        num_steps = -1;

        

    end
    
    methods
        % Constructor
        function obj = MPC_swing_up(u_limit, ...
                M_rk4_iter, dt, N)
            % Initialize properties and setup MPC parameters

            obj.u_max = u_limit(2);
            obj.u_min = u_limit(1);

            obj.M_rk4_iter = M_rk4_iter;
            obj.dt = dt;
            obj.N = N;
            
            u0 = randn(obj.N*obj.n_controls, 1)*2; 
            X0 = repmat(randn(1, obj.n_states)*3, 1,obj.N+1)';
            obj.x0  = [X0; u0];
            obj.p = zeros(obj.n_states + obj.n_reference, 1);

            obj.setupSystemDynamics();
            obj.setupMPCProblem();
        end
        
        % Method to set up the system dynamics
        function obj = setupSystemDynamics(obj)
           
            % states
            x1 = casadi.SX.sym('x1');
            x2 = casadi.SX.sym('x2');
            x3 = casadi.SX.sym('x3');
            x4 = casadi.SX.sym('x4');
            x5 = casadi.SX.sym('x5');

            % state vector
            states = [x1;x2;x3;x4;x5];

            % control
            control = casadi.SX.sym('u');

            obj.state_transition = ...
            casadi.Function...
            ('xdot', {states, control}, ...
            {obj.state_dynamics(states, control)} ...
        , {'states', 'control'}, {'dot_states'} ...
        );

        end

        function result = norm(obj, x, Q, l)

            if l == 1
                result = sum(Q.^0.5*abs(x));
            elseif l == 2 
                result = x'*Q*x;
            end

        end
        
        % Method to set up the MPC problem
        function obj=setupMPCProblem(obj)
            
            U = casadi.SX.sym('U',obj.n_controls,obj.N); 
            P = casadi.SX.sym('P',obj.n_states + obj.n_reference);
           

            X = casadi.SX.sym('X',obj.n_states,(obj.N+1));


            cost = 0; % Objective function
            con_vctr = [];  % constraints vector



            st  = X(:,1); % initial state
            con_vctr = [con_vctr;st-P(1:obj.n_states)]; % initial condition constraints
            current_end = length(con_vctr);
            con_vctr = [con_vctr; zeros(obj.n_states*obj.N, 1)];
            for k = 1:obj.N
                st = X(:,k);  con = U(:,k);
                cost = cost+ ...
                obj.norm(con, obj.Rw, obj.u_norm)*obj.gamma_control*obj.dt + ...
                obj.norm(st, ...
                obj.Qw, obj.x_norm)*obj.dt;

                st_next = X(:,k+1);
                
               
                st_next_RK4=rk4_integration( ...
                    st, con, @obj.state_transition, obj.dt, obj.M_rk4_iter);   
    
                con_vctr(current_end+1:current_end+obj.n_states, 1) = ...
                    st_next-st_next_RK4; % compute constraints % new

                current_end = current_end+obj.n_states;
            end
            

            % N*dt is intoduced to make scales invariant to horizon length
            cost = cost + ...
                obj.norm(obj.Ow*st-P(obj.n_states+1:end), ...
                obj.Qwobs, obj.x_norm)*obj.gamma_final*obj.N*obj.dt; % final state

            % make the decision variable one column  vector
            OPT_variables = [reshape(X,obj.n_states*(obj.N+1),1);
                reshape(U,obj.n_controls*obj.N,1)];

            nlp_prob = struct('f', cost, ...
                'x', OPT_variables, ...
                'g', con_vctr, 'p', P);

            opts = struct;
            opts.ipopt.max_iter = obj.max_iter;
            opts.ipopt.acceptable_tol =obj.acceptable_tol;
            opts.ipopt.acceptable_obj_change_tol = ...
            obj.acceptable_obj_change_tol;
            opts.ipopt.print_level =3;%0,3
            opts.print_time = 0;

            obj.solver = casadi.nlpsol('solver', 'ipopt', nlp_prob,opts);

            obj.args = struct;
            
            % equality constraints: 
            obj.args.lbg(1:obj.n_states*(obj.N+1), 1) = 1e-5;  
            obj.args.ubg(1:obj.n_states*(obj.N+1), 1) = 1e-5;
            
            % state constraints
            obj.args.lbx(1:obj.n_states*(obj.N+1), 1) = -inf;  
            obj.args.ubx(1:obj.n_states*(obj.N+1), 1) = inf;

            obj.args.lbx(3:obj.n_states:obj.n_states*(obj.N+1), 1) = -2*pi;  
            obj.args.ubx(3:obj.n_states:obj.n_states*(obj.N+1), 1) = 2*pi;
            
            obj.args.lbx(1:obj.n_states:obj.n_states*(obj.N+1), 1) = -pi;  
            obj.args.ubx(1:obj.n_states:obj.n_states*(obj.N+1), 1) = pi;

            


         
            % controll constraints 
            obj.args.lbx(obj.n_states*(obj.N+1)+1: ...
                obj.n_states*(obj.N+1)+obj.N*obj.n_controls,1) = obj.u_min; 
            obj.args.ubx(obj.n_states*(obj.N+1)+1: ...
                obj.n_states*(obj.N+1)+obj.N*obj.n_controls,1) = obj.u_max; 
        end

        function obj=set_reference(obj, r)
            obj.p(obj.n_states+1:end) = r; 
        end
        
        % Method to run MPC loop
        function u = runMPC(obj, ...
                current_state, reference)
           
           %initial state
           obj.p(1:obj.n_states)   = current_state;    
           obj.x0(1:obj.n_states)  = current_state;

           obj.p(obj.n_states+1:end) = reference;
           sol = obj.solver('x0', obj.x0, 'lbx', obj.args.lbx, 'ubx', ...
               obj.args.ubx, 'lbg', obj.args.lbg, 'ubg', obj.args.ubg, ...
               'p',obj.p);

           full_trajetory = sol.x;

           u_full = reshape(full( ...
               full_trajetory(obj.n_states*(obj.N+1)+1:end) ...
               )',obj.n_controls,obj.N)';

           x_full = reshape(full( ...
               full_trajetory(1:obj.n_states*(obj.N+1)) ...
               )',obj.n_states,obj.N+1)'; 
           

         
            
           u = u_full(1);
           obj.store_trajectory(x_full, u_full);
           
           % shifting by 1 time lag
           x_full = [x_full(2:end,:); x_full(end, :)];
           u_full = [u_full(2:end,:); u_full(end, :)];
            
           obj.x0 = [reshape(x_full, obj.n_states*(obj.N+1), 1);...
               reshape(u_full, obj.n_controls*obj.N, 1)];

           obj.num_steps = obj.num_steps+1;
        end

        function obj = store_trajectory(obj, x_full, u_full)
            obj.u_mpc = u_full;
            obj.x_mpc = x_full;
        end

        function plot_trajectories(obj)
            figure();
            hold all;
            t = 0:length(obj.u_mpc)-1;
            t = t*obj.dt + obj.num_steps*obj.dt;

            stairs(t, obj.u_mpc, 'k--');
           
            figure();
            hold all;
            t = 0:length(obj.x_mpc)-1;
            t = t*obj.dt + obj.num_steps*obj.dt;

            plot(t, obj.x_mpc(:, 1)/pi*180, 'k--');
            plot(t, obj.x_mpc(:, 3)/pi*180, 'k--');
       

        end
    end
end
