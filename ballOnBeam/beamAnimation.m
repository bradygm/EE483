classdef beamAnimation < handle
    %
    %    Create pendulum animation
    %
    %--------------------------------
    properties
        beam_handle
        ball_handle
        ell
        width
        height
        gap
        radius
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = beamAnimation(P)
            self.ell = P.ell;
            self.width = P.w;
            self.radius = P.radius;
           
            figure(1), clf
            plot([0, self.ell],[0,0],'k--'); % draw track
            hold on
            % initialize the base, rod, and bob to initial conditions
            self=self.drawBeam(P.theta0);
            self=self.drawBall(P.z0, P.theta0);
            axis([-.5*self.ell, 2*self.ell, -1.5*self.ell, 1.5*self.ell]); % Change the x,y axis limits
            pbaspect([(2*self.ell+.5*self.ell)/(1.5*self.ell+1.5*self.ell) 1 1])
            xlabel('z'); % label x-axis
        end
        %---------------------------
        function self=drawBeamBall(self, x)
            % Draw pendulum is the main function that will call the functions:
            % drawCart, drawCircle, and drawRod to create the animation.
            % x is the system state
            z= x(1);        % Horizontal position of cart, m
            theta = x(2);   % Angle of pendulum, rads

            self=self.drawBall(z, theta);
            self=self.drawBeam(theta);
            drawnow
        end
        %---------------------------
        function self=drawBeam(self, x)
            theta = x(1);
            pts = [...
                0, -self.width;...
                self.ell, -self.width;...
                self.ell, 0;...
                0, 0;...
                ]';
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            pts = R*pts;
  
            if isempty(self.beam_handle)
                self.beam_handle = fill(pts(1,:), pts(2,:), 'b');
            else
                set(self.beam_handle, 'XData', pts(1,:), 'YData', pts(2,:));
                drawnow
            end
        end
        %---------------------------
        function self=drawBall(self, z, theta)
            th = 0:2*pi/20:2*pi;
            center = [ z , self.radius];
            pts = center + [self.radius*cos(th)', self.radius*sin(th)'];

            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            pts = pts*R' ;
            
            if isempty(self.ball_handle)
                self.ball_handle = fill(pts(:,1),pts(:,2),'g');
%                 self.ball_handle = viscircles(center',self.radius);
            else
                set(self.ball_handle,'XData',pts(:,1));
                set(self.ball_handle,'YData',pts(:,2));
%                 set(self.ball_handle,'Children',[center',self.radius]);
            end
        end 
    end
end