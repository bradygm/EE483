classdef vtolDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        Jc
        mc
        mr
        ml
        d
        u
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = vtolDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.zv0;...      % initial base angle
                        P.h0;...        % initial panel angle
                        P.theta0;...   % initial angular velocity of base
                        P.zvdot0;...
                        P.hdot0;...
                        P.thetadot0;...% initial angular velocity of panel
                        ];     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
%             alpha = 0.2;  % Uncertainty parameter
            alpha = 0;
            self.Jc = P.Jc * (1+2*alpha*rand-alpha);  % inertia of base
            self.mc = P.mc * (1+2*alpha*rand-alpha);  % inertia of panel
            self.mr = P.mr * (1+2*alpha*rand-alpha);    % spring coefficient
            self.ml = self.mr;
            self.g = P.g;
            self.d = P.d * (1+2*alpha*rand-alpha);    % Damping coefficient, Ns
            self.u = P.u * (1+2*alpha*rand-alpha); 
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
          
        end
        %----------------------------
        function self = propagateDynamics(self, u1,u2)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, u1,u2);
            k2 = self.derivatives(self.state + self.Ts/2*k1, u1,u2);
            k3 = self.derivatives(self.state + self.Ts/2*k2, u1,u2);
            k4 = self.derivatives(self.state + self.Ts*k3, u1,u2);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
        end
        %----------------------------
        function xdot = derivatives(self, state, u1,u2)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            % re-label states and inputs for readability
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);
            fl = u1;
            fr = u2;
            % The equations of motion.
            M = [...
                self.mc+2*self.mr, 0, 0;...
                0, self.mc+2*self.mr, 0;...
                0, 0, self.Jc+2*self.mr*self.d^2;...
                ];
            c = [...
                (-fr-fl)*sin(theta)-self.u*zdot;...
                (-self.mc-2*self.mr)*self.g+(fr+fl)*cos(theta);...
                (fr-fl)*self.d;...
                ];
            tmp = M\c;
            thetadotdot = tmp(3);
            hdotdot = tmp(2);
            zdotdot = tmp(1);
            % build xdot and return
            xdot = [zdot; hdot; thetadot; zdotdot; hdotdot; thetadotdot];
        end
        %----------------------------
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [theta, phi] with added Gaussian noise
            % 
            % re-label states for readability
            z = self.state(1);
            height = self.state(2);
            theta = self.state(3);
            % add Gaussian noise to outputs
            theta_m = theta + 0.001*randn;
            h_m = height + 0.001*randn;
            z_m = z + 0.001*randn;
            % return measured outputs
            y = [z_m; h_m; theta_m];
        end
        %----------------------------
        function x = states(self)
            %
            % Returns all current states as a list
            %
            x = self.state;
        end
    end
end


