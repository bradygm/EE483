classdef springController
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
    end
    %----------------------------
    methods
        %----------------------------
        function self = springController(P)
            % Instantiates the SS_ctrl object
            self.zCtrl = springPDControl(P.kp, P.kd, P.beta, P.Ts);
            % plant parameters known to controller
            self.m = P.m;
            self.b = P.b;
            self.k = P.k;
            self.g = P.g;
            self.limit = P.f_max;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            % compute equilibrium force
%             f_e = self.k*z;
            % compute the linearized force using PID
%             f_tilde = self.zCtrl.PD(z_r, z, false);
            forceTemp = self.zCtrl.PD(z_r, z, false);
            if abs(forceTemp) > self.limit
                forceTemp = self.limit*sign(forceTemp);
            end
            force = forceTemp;
            % compute total torque
%             force = f_e + f_tilde;
        end
    end
end