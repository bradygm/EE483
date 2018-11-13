classdef beamController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        z_dot
        theta_dot
        z_d1
        theta_d1
        m2
        g
        ell
        m1    
        limit
        beta
        Ts
        K
        ki
        integrator
        error_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = beamController(P)
            % Instantiates the SS_ctrl object
            self.z_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = P.z0;
            self.theta_d1 = 0.0;
            self.m2 = P.m2;
            self.g = P.g;
            self.ell = P.ell;
            self.m1 = P.m1;
            self.limit = P.F_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.K = P.K;
            self.ki = P.ki;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            
            % differentiate z and theta
            self.differentiateZ(z);
            self.differentiateTheta(theta);
            
            % integrate error
            self.integrateError(z_r-z);
            
            % Construct the state
            x = [z-.25; theta; self.z_dot; self.theta_dot];
            
            % Compute the state feedback controller
            F_tilda = -self.K*x - self.ki*self.integrator;

            
            Ftemp = F_tilda + (self.m2*self.g*self.ell+2*self.m1*self.g*z)/(2*self.ell);
            F = self.saturate(Ftemp);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
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