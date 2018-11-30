classdef beamController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        x_hat
        F_d1
        m2
        g
        ell
        m1    
        limit
        Ts
        K
        ki
        L
        A
        B
        C
        integrator
        error_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = beamController(P)
            % Instantiates the SS_ctrl object
            self.x_hat = [P.z0; 0.0; 0.0; 0.0];
            self.F_d1 = 0.0;
            self.m2 = P.m2;
            self.g = P.g;
            self.ell = P.ell;
            self.m1 = P.m1;
            self.limit = P.F_max;
            self.Ts = P.Ts;
            self.K = P.K;
            self.ki = P.ki;
            self.L = P.L;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.A = P.A;
            self.B = P.B;
            self.C = P.C;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            
            % update the observer and extract z_hat
            self.updateObserver(y);
            z_hat = self.x_hat(1);
            
            % integrate error
            self.integrateError(z_r-z_hat);
            
            % Construct the state
            x = self.x_hat;
            x(1) = x(1)-self.ell/2;
            
            % Compute the state feedback controller
            F_tilda = -self.K*x - self.ki*self.integrator;

            
            Ftemp = F_tilda + (self.m2*self.g*self.ell+2*self.m1*self.g*(self.ell/2))/(2*self.ell);
            Fsat = self.saturate(Ftemp);
            self.updateForce(Fsat);
            F = Fsat;
            
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*(self.x_hat-[self.ell/2;0;0;0])...
                    + self.B*(self.F_d1-(self.m2*self.g*self.ell+2*self.m1*self.g*(self.ell/2))/(2*self.ell))...
                    + self.L*(y_m-self.C*self.x_hat));
            end
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
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
    end
end