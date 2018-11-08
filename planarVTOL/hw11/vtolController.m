classdef vtolController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        hCtrl
        zCtrl
        thetaCtrl
        fe
        d
        limit
        h_dot
        th_dot
        z_dot
        h_d1
        th_d1
        z_d1
        K
        kr
        beta
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = vtolController(P)
            self.fe = P.f_e;
            self.d = P.d;
            self.limit = P.fmax;
            self.h_dot = 0.0;
            self.th_dot = 0.0;
            self.z_dot = 0.0;
            self.h_d1 = 0.0;
            self.th_d1 = 0.0;
            self.z_d1 = P.zv0;
            self.K = P.K;
            self.kr = P.kr;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function [Fl,Fr] = u(self, y_r, y2_r, y)
            % y_r is the referenced input
            % y is the current state
            h_r = y_r;
            z_r = y2_r;
            h = y(1);
            z = y(2);
            theta = y(3);
            
            % differentiate phi and theta
            self.differentiateTheta(theta);
            self.differentiateZ(z);
            self.differentiateH(h);
            
            x = [h; z; theta; self.h_dot; self.z_dot; self.th_dot];
            
%          
            
            F = [self.fe; 0] - self.K*x + self.kr*[h_r; z_r];
            Km = 1;
            Fltemp = (1/(2*Km))*(F(1)-F(2)/self.d);
            Frtemp = (1/(2*Km))*(F(1)+F(2)/self.d);
            Fl = self.saturate(Fltemp);
            Fr = self.saturate(Frtemp);
            
%             force = self.saturate(forceTemp);
        end
        %----------------------------
        function self = differentiateH(self, h)
            self.h_dot = ...
                self.beta*self.h_dot...
                + (1-self.beta)*((h - self.h_d1) / self.Ts);
            self.h_d1 = h;            
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
            self.th_dot = ...
                self.beta*self.th_dot...
                + (1-self.beta)*((theta-self.th_d1) / self.Ts);
            self.th_d1 = theta;
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