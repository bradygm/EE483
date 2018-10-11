classdef VTOLAnimation < handle
    %
    %    Create satellite animation
    %
    %--------------------------------
    properties
        center_handle
        panel1_handle
        panel2_handle
        stick_handle
        target_handle
        length
        width
        square
        distance
        panelLength
        panelWidth
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = VTOLAnimation(P)
            self.length = P.length;
            self.width = P.width;
            self.distance = P.d;
            self.square = P.square;
            self.panelLength = P.pl;
            self.panelWidth = P.pw;
            figure(1), clf
            plot([-2*3,2*3],[P.h0,P.h0],'k--'); % plot track
            hold on
            plot([-2*3,2*3],[0 0]); %Ground
            % initialize the base and panel to initial conditions
            self=self.drawStick(P.theta0,P.zv0,P.h0);
            self=self.drawCenter(P.theta0,P.zv0,P.h0);
            self=self.drawPanel1(P.theta0,P.zv0,P.h0);
            self=self.drawPanel2(P.theta0,P.zv0,P.h0);
            self=self.drawTarget(P.zt0);
            axis([-.5*3, 2*3, -.5*2, 2*2]);
            pbaspect([(2*3+.5*3)/(2*2+.5*2) 1 1])
        end
        %---------------------------
        function self=drawVTOL(self, x,u)
            % Draw satellite is the main function that will call the functions:
            % drawBase and drawPanel create the animation.
            % x is the system state
            zv = x(1);    % angle of VTOL
            h   = x(2);    % z position of VTOL
            theta = x(3); % z position of target
            zt = u; % height of vtol
            
            
            self=self.drawStick(theta, zv, h);
            self=self.drawCenter(theta, zv, h);
            self=self.drawPanel1(theta, zv, h);
            self=self.drawPanel2(theta, zv, h);
            self=self.drawTarget(zt);
            drawnow
        end
        %---------------------------
        function self=drawStick(self, theta, zv, h)
            % define points on base (without rotation)
            pts = [...
                (+self.distance), 0;... %xy sets
                (-self.distance), 0;...
                ]';
            
            % define rotation matrix
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % rotate points
            pts = R*pts;
            center = [zv, h]';
            pts = pts + center;

            if isempty(self.stick_handle)
                self.stick_handle = plot(pts(1,:),pts(2,:),'b','LineWidth',2);
            else
                set(self.stick_handle,'XData',pts(1,:));
                set(self.stick_handle,'YData',pts(2,:));
            end
        end
        %---------------------------
        function self=drawCenter(self, theta, zv, h)
            % define points on base (without rotation)
            pts = [...
                (-self.square/2), (-self.square/2);...
                (+self.square/2), (-self.square/2);...
                (+self.square/2), (+self.square/2);...
                (-self.square/2), (+self.square/2);...
                ]';
            % define rotation matrix
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % rotate points
            pts = R*pts;
            center = [zv, h]';
            pts = pts + center;
            if isempty(self.center_handle)
                self.center_handle = fill(pts(1,:),pts(2,:),'g');
            else
                set(self.center_handle,'XData',pts(1,:));
                set(self.center_handle,'YData',pts(2,:));
            end
        end
        %---------------------------
        function self=drawPanel1(self, theta, zv, h)
            % define points on base (without rotation)
            th = 0:2*pi/20:2*pi;
            center = [ zv , h]';
            shift = [self.distance, 0];
            pts = shift + [self.panelLength*cos(th)', self.panelWidth*sin(th)'];

            % define rotation matrix
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % rotate points
            pts = R*pts';
            pts = pts + center;
            if isempty(self.panel1_handle)
                self.panel1_handle = fill(pts(1,:),pts(2,:),'r');
            else
                set(self.panel1_handle,'XData',pts(1,:));
                set(self.panel1_handle,'YData',pts(2,:));
            end
        end
        %---------------------------
        function self=drawPanel2(self, theta, zv, h)
            % define points on base (without rotation)
            th = 0:2*pi/20:2*pi;
            center = [ zv , h]';
            shift = [-self.distance, 0];
            pts = shift + [self.panelLength*cos(th)', self.panelWidth*sin(th)'];
            % define rotation matrix
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % rotate points
            pts = R*pts';
            pts = pts + center;
            if isempty(self.panel2_handle)
                self.panel2_handle = fill(pts(1,:),pts(2,:),'r');
            else
                set(self.panel2_handle,'XData',pts(1,:));
                set(self.panel2_handle,'YData',pts(2,:));
            end
        end
        %---------------------------
        function self=drawTarget(self, zt)
            % define points on base (without rotation)
            pts = [...
                (zt-self.length/2), 0;...
                (zt+self.length/2), 0;...
                (zt+self.length/2),  self.width;...
                (zt-self.length/2),  self.width;...
                ]';
            
            if isempty(self.target_handle)
                self.target_handle = fill(pts(1,:),pts(2,:),'g');
            else
                set(self.target_handle,'XData',pts(1,:));
                set(self.target_handle,'YData',pts(2,:));
            end
        end
    end
end