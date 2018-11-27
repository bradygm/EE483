classdef springController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        limit
        K
        ki
        Ts
        z_d1
        integrator
        error_d1
        force_d1
        C
        B
        A
        x_hat
        L
    end
    %----------------------------
    methods
        %----------------------------
        function self = springController(P)
            self.limit = P.f_max;
            % initialize object properties
            self.z_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.Ts = P.Ts;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.force_d1 = 0.0;
            self.C = P.C;
            self.B = P.B;
            self.A = P.A;
            self.x_hat = [0.0; 0.0];
            self.L = P.L;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r; 
            
            % compute equilibrium force
            f_e = 0;
            
            % update the observer and extract z_hat
            self.updateObserver(y);
            z_hat = self.x_hat(1);
            
            % integrate error
            self.integrateError(z_r-z_hat);
            
            % Compute the state feedback controller
            z_tilde = -self.K*self.x_hat - self.ki*self.integrator;

            
            zSat = self.saturate(z_tilde + f_e);
            force = zSat;
            self.updateForce(force);
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*self.x_hat...
                    + self.B*(self.force_d1)...
                    + self.L*(y_m-self.C*self.x_hat));
            end
        end
        %----------------------------
        function self = updateForce(self, force)
            self.force_d1 = force;
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