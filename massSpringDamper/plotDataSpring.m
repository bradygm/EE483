classdef plotDataSpring < handle
    %    This class plots the time histories for the pendulum data.
    %----------------------------
    properties
        % data histories
        time_history
        ref_history
        z_history
        f_history
        index
        % figure handles
        ref_handle
        z_handle
        f_handle
    end
    methods
        %--constructor--------------------------
        function self = plotDataSpring(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.ref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.f_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(2, 1, 1)
                hold on
                self.ref_handle = plot(self.time_history, self.ref_history, 'g');
                self.z_handle    = plot(self.time_history, self.z_history, 'b');
                ylabel('z(meters)')
                title('Mass Data')
            subplot(2, 1, 2)
                hold on
                self.f_handle    = plot(self.time_history, self.f_history, 'b');
                ylabel('Force')
        end
        %----------------------------
        function self = updatePlots(self, t, reference, states, ctrl)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.ref_history(self.index) = reference;
            self.z_history(self.index) = states(1);
            self.f_history(self.index) = ctrl;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.ref_handle, 'Xdata', self.time_history, 'Ydata', self.ref_history)
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.f_handle, 'Xdata', self.time_history, 'Ydata', self.f_history)
        end
    end
end
