classdef plotVTOLData < handle
    %    This class plots the time histories for the satellite data.
    %----------------------------
    properties
        % data histories
        time_history
        theta_history
        zv_history
        zt_history
        h_history
        fl_history
        fr_history
        index
        % figure handles
        zv_handle
        zt_handle
        h_handle
        fl_handle
        fr_handle
        theta_handle
    end
    methods
        %--constructor--------------------------
        function self = plotVTOLData(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.zv_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.zt_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.fl_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.fr_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(3, 1, 1)
                hold on
                self.zt_handle = plot(self.time_history, self.zt_history, 'g');
                self.zv_handle    = plot(self.time_history, self.h_history, 'b');
                ylabel('Zt and Zv')
                title('VTOL Data')
            subplot(3, 1, 2)
                hold on
                self.theta_handle    = plot(self.time_history, self.theta_history, 'b');
                ylabel('theta(deg)')
            subplot(3, 1, 3)
                hold on
                self.fl_handle    = plot(self.time_history, self.fl_history, 'b');
                self.fr_handle    = plot(self.time_history, self.fr_history, 'r');
                ylabel('force (Nm)')
        end
        %----------------------------
        function self = updatePlots(self, t, reference, states, ctrl1,ctrl2)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.zt_history(self.index) = reference;
            self.zv_history(self.index) = states(1);
            self.h_history(self.index) = states(2);
            self.theta_history(self.index) = 180/pi*states(3);
            self.fl_history(self.index) = ctrl1;
            self.fr_history(self.index) = ctrl2;
            self.index = self.index + 1;

            
            % update the plots with associated histories
            set(self.zv_handle, 'Xdata', self.time_history, 'Ydata', self.h_history)
            set(self.zt_handle, 'Xdata', self.time_history, 'Ydata', self.zt_history)
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.fl_handle, 'Xdata', self.time_history, 'Ydata', self.fl_history)
            set(self.fr_handle, 'Xdata', self.time_history, 'Ydata', self.fr_history)
        end
    end
end
