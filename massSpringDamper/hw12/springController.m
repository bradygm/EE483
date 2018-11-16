classdef springController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        m
        b
        k
        g
        limit
        K
        ki
        beta
        Ts
        z_dot
        z_d1
        integrator
        error_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = springController(P)
            % plant parameters known to controller
            self.m = P.m;
%             self.b = P.b;
%             self.k = P.k;
            self.g = P.g;
            self.limit = P.f_max;
            % initialize object properties
            self.z_dot = P.zdot0;
            self.z_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r; %****!!!!!! Should i subtract???
            z = y(1);
            
            % compute equilibrium force
%             f_e = self.k*z;
            f_e = 0;
            
            % differentiate z
            self.differentiateZ(z);
            
            % integrate error
            self.integrateError(z_r-z);

            % Construct the state
            x = [z; self.z_dot];
            
            % Compute the state feedback controller
            z_tilde = -self.K*x - self.ki*self.integrator;

            
            zSat = self.saturate(z_tilde + 0);
%             self.integratorAntiWindup(zSat, z_tilde);
            force = zSat;
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z-self.z_d1) / self.Ts);
            self.z_d1 = z;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
         %----------------------------
        function self = integratorAntiWindup(self, u_sat, u_unsat)
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator + self.Ts/self.ki*(u_sat-u_unsat);
            end
        end
    end
end