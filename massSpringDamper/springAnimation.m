classdef springAnimation < handle
    %
    %    Create pendulum animation
    %
    %--------------------------------
    properties
        link_handle
        spring_handle
        width
        height
        gap
        
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = springAnimation(P)
            self.width = P.w;       
            self.height = P.h;      
            self.gap = P.gap;
            figure(1), clf
            %plot([0,self.length],[0,0],'k--'); % plot track
            rectangle('Position',[P.z0 0 self.width self.height],'LineStyle','--','EdgeColor',[0 0 1]) %Initial Position
            hold on
            plot([(P.z0-self.gap) (P.z0-self.gap)],[0 5],'b')
            plot([(P.z0-self.gap) 5],[0 0],'b')
            self.drawSpring([P.z0; P.zdot0]);
            axis([-3, 5, -2, 5]);
        end
        %---------------------------
        function self = drawSpring(self, x)
            z = x(1);
  
            self=self.drawBox(z);
            self=self.drawLine(z);
            drawnow
        end
        %---------------------------
        function self = drawBox(self, x)
            z = x(1);
  
            if isempty(self.link_handle)
                self.link_handle = rectangle('Position',[z 0 self.width self.height],'FaceColor',[0 .5 .5]);
            else
                set(self.link_handle, 'Position', [z 0 self.width self.height]);
                drawnow
            end
        end
        %---------------------------
        function self = drawLine(self, x)
            z = x(1);
            pts = [z,self.height/2;...
                   z-(z+self.gap)*1/6,self.height*3/4;...
                   z-(z+self.gap)*2/6,self.height*1/4;...
                   z-(z+self.gap)*3/6,self.height*3/4;...
                   z-(z+self.gap)*4/6,self.height*1/4;...
                   z-(z+self.gap)*5/6,self.height*3/4;...
                   z-(z+self.gap),self.height/2];
  
            if isempty(self.spring_handle)
                self.spring_handle = plot(pts(:,1),pts(:,2),'r');
            else
                set(self.spring_handle, 'XData', pts(:,1));
                drawnow
            end
        end
    end
end