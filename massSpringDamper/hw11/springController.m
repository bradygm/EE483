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
        kr
        beta
        Ts
        z_dot
        z_d1
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
            self.kr = P.kr;
            self.beta = P.beta;
            self.Ts = P.Ts;
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

            % Construct the state
            x = [z; self.z_dot];
            
            % Compute the state feedback controller
            z_tilde = -self.K*x + self.kr*z_r;


            force = self.saturate(z_tilde + 0);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z-self.z_d1) / self.Ts);
            self.z_d1 = z;
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