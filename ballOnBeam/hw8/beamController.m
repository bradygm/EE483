classdef beamController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
        m2
        g
        ell
        m1    
        limit
    end
    %----------------------------
    methods
        %----------------------------
        function self = beamController(P)
            % Instantiates the SS_ctrl object
            self.zCtrl = beamPDControl(P.kp_z, P.kd_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = beamPDControl(P.kp_th, P.kd_th, P.F_max, P.beta, P.Ts);
            self.m2 = P.m2;
            self.g = P.g;
            self.ell = P.ell;
            self.m1 = P.m1;
            self.limit = P.F_max;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.zCtrl.PD(z_r, z, false);
            % the force applied to the cart comes from the inner loop PD control
            F_tilda = self.thetaCtrl.PD(theta_r, theta, false);
            Ftemp = F_tilda + (self.m2*self.g*self.ell+2*self.m1*self.g*z)/(2*self.ell);
            F = self.saturate(Ftemp);
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