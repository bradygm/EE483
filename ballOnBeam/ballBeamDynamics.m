classdef ballBeamDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m1
        m2
        ell
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = ballBeamDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = 0;  % Uncertainty parameter
            self.m1 = P.m1 * (1+2*alpha*rand-alpha);  % Mass of the pendulum, kg
            self.m2 = P.m2 * (1+2*alpha*rand-alpha);  % Mass of the cart, kg
            self.ell = P.ell * (1+2*alpha*rand-alpha);  % Length of the rod, m
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
          
        end
        %----------------------------
        function self = propagateDynamics(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, u);
            k2 = self.derivatives(self.state + self.Ts/2*k1, u);
            k3 = self.derivatives(self.state + self.Ts/2*k2, u);
            k4 = self.derivatives(self.state + self.Ts*k3, u);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
        end
        %----------------------------
        function xdot = derivatives(self, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            % re-label states and inputs for readability
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            F = u;
            % The equations of motion.
            M = [self.m1, 0;...
                 0,self.m2*self.ell^2/3.0 + self.m1*z^2];
            C = [self.m1*z*thetadot^2-self.m1*self.g*sin(theta);...
                 -2*self.m1*z*zdot*thetadot-self.m2*self.g*self.ell/2.0*cos(theta)-self.m1*self.g*z*cos(theta)+F*self.ell*cos(theta)];
            tmp = M\C;
%             tmp = inv(M)*C;
            zddot = tmp(1);
            thetaddot = tmp(2);
            % build xdot and return
            xdot = [zdot; thetadot; zddot; thetaddot];
        end
        %----------------------------
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            % re-label states for readability
            z = self.state(1);
            theta = self.state(2);
            % add Gaussian noise to outputs
            z_m = z + 0.0*randn;
            theta_m = theta + 0.00*randn;
            % return measured outputs
            y = [z_m; theta_m];
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


