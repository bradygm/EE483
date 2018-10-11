classdef vtolController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        hCtrl
        fe
    end
    %----------------------------
    methods
        %----------------------------
        function self = vtolController(P)
            % Instantiates the SS_ctrl object
            self.hCtrl = vtolPDControl(P.kp, P.kd, P.beta, P.Ts);
            % plant parameters known to controller
            self.fe = P.f_e;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            h_r = y_r;
            h = y(2);
            
            % compute equilibrium force
%             f_e = self.k*z;
            % compute the linearized force using PID
            f_tilde = self.hCtrl.PD(h_r, h, false);
%             force = self.zCtrl.PD(z_r, z, false);
            % compute total torque
            force = self.fe + f_tilde;
            
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