classdef vtolController
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
    end
    %----------------------------
    methods
        %----------------------------
        function self = vtolController(P)
            % Instantiates the SS_ctrl object
            self.hCtrl = vtolPDControl(P.kp_h, P.kd_h, P.beta, P.Ts);
            self.zCtrl = vtolPDControl(P.kp_z, P.kd_z, P.beta, P.Ts);
            self.thetaCtrl = vtolPDControl(P.kp_th, P.kd_th, P.beta, P.Ts);
            % plant parameters known to controller
            self.fe = P.f_e;
            self.d = P.d;
        end
        %----------------------------
        function [Fl,Fr] = u(self, y_r, y2_r, y)
            % y_r is the referenced input
            % y is the current state
            h_r = y_r;
            z_r = y2_r;
            z = y(1);
            h = y(2);
            theta = y(3);
            
            % compute equilibrium force
%             f_e = self.k*z;
            % compute the linearized force using PID
            f_tilde = self.hCtrl.PD(h_r, h, false);
            theta_r = self.zCtrl.PD(z_r, z, false);
            tau_tilde = self.thetaCtrl.PD(theta_r, theta, false);
            Tau = tau_tilde + 0;
%             force = self.zCtrl.PD(z_r, z, false);
            % compute total torque
            F = self.fe + f_tilde;
            Km = 1;
            Fl = (1/(2*Km))*(F-Tau/self.d);
            Fr = (1/(2*Km))*(F+Tau/self.d);
            
            
%             force = self.saturate(forceTemp);
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