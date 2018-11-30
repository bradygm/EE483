classdef vtolController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        x_hat
        fe
        d
        limit
        h_d1
        th_d1
        z_d1
        K
        ki
        Llat
        Llon
        beta
        Ts
        integrator_lat
        integrator_lon
        errorlat_d1
        errorlon_d1
        Alon
        Blon
        Clon
        Alat
        Blat
        Clat
        tau_d1
        f_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = vtolController(P)
            self.x_hat = [P.h0; P.zv0; 0.0; 0.0; 0.0; 0.0];
            self.fe = P.f_e;
            self.d = P.d;
            self.limit = P.fmax;
            self.Llat = P.Llat;
            self.Llon = P.Llon;
            self.h_d1 = 0.0;
            self.th_d1 = 0.0;
            self.z_d1 = P.zv0;
            self.K = P.K;
            self.ki = P.ki;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.integrator_lat = 0.0;
            self.integrator_lon = 0.0;
            self.errorlat_d1 = 0.0;
            self.errorlon_d1 = 0.0;
            self.Alon = P.Alon;
            self.Blon = P.Blon;
            self.Clon = P.Clon;
            self.Alat = P.Alat;
            self.Blat = P.Blat;
            self.Clat = P.Clat;
            self.tau_d1 = 0.0;
            self.f_d1 = 0.0;

        end
        %----------------------------
        function [Fl,Fr] = u(self, y_r, y2_r, y)
            % y_r is the referenced input
            % y is the current state
            h_r = y_r;
            z_r = y2_r;
            
            % update the observer and extract phi_hat
            self.updateObserverLat(y([2 3]));
            self.updateObserverLon(y(1));
            
            %integrate the error
            self.integrateErrorZ(z_r-self.x_hat(1));
            self.integrateErrorH(h_r-self.x_hat(2));
            
%             x = [h; z; theta; self.h_dot; self.z_dot; self.th_dot];
            
            xtemp = [self.x_hat(2);self.x_hat(1);self.x_hat(3);self.x_hat(5);self.x_hat(4);self.x_hat(6)];
            
            F = [self.fe; 0] - self.K*xtemp - self.ki*[self.integrator_lon; self.integrator_lat];
            Km = 1;
            self.updateTorque(F(2));
            self.updateForce(F(1));
            Fltemp = (1/(2*Km))*(F(1)-F(2)/self.d);
            Frtemp = (1/(2*Km))*(F(1)+F(2)/self.d);
            Fl = self.saturate(Fltemp);
            Fr = self.saturate(Frtemp);
            
%             force = self.saturate(forceTemp);
        end
        %----------------------------
        function self = updateObserverLon(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat([2,5]) = self.x_hat([2,5]) + self.Ts/N*(...
                    self.Alon*self.x_hat([2,5])...
                    + self.Blon*(self.f_d1-self.fe)...
                    + self.Llon*(y_m-self.Clon*self.x_hat([2,5])));
            end
        end
        %----------------------------
        function self = updateObserverLat(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat([1,3,4,6]) = self.x_hat([1,3,4,6]) + self.Ts/N*(...
                    self.Alat*self.x_hat([1,3,4,6])...
                    + self.Blat*self.tau_d1...
                    + self.Llat*(y_m-self.Clat*self.x_hat([1,3,4,6])));
            end
        end
        %----------------------------
        function self = integrateErrorZ(self, error)
            self.integrator_lat = self.integrator_lat + (self.Ts/2.0)*(error+self.errorlat_d1);
            self.errorlat_d1 = error;
        end
        %----------------------------
        function self = integrateErrorH(self, error)
            self.integrator_lon = self.integrator_lon + (self.Ts/2.0)*(error+self.errorlon_d1);
            self.errorlon_d1 = error;
        end
        %----------------------------
        function self = updateTorque(self, tau)
            self.tau_d1 = tau;
        end
        %----------------------------
        function self = updateForce(self, F)
            self.f_d1 = F;
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