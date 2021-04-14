function [ax,hlines,fh] = jzplotys(xy_pairs,ax_groups,xrange,leg_space)
% jzplotys - Inspired by multiplotyyy (from matlab central) but uses a
%            different approach.  In this code all the axis are the same
%            size that contain data and dummy axis are used to make the
%            extra Y axis.  This was done so that the linkzoom.m function
%            from matlab function could be used in parallel with this plot
%            function (when the x - axis sizes are different as in the
%            multiplotyyy, it skews the x data and when zooming the data is
%            no longer aligned on the x - axis.
%
% Syntax:  [ax,hlines,fh] = jzplotys(xy_pairs,ax_groups,xrange,leg_space)
%
% Inputs:
% xy_pairs = {x1,y1,x2,y2,x3,y3,...} cell array with all xdata and ydata
% ax_groups = [nax1 nax2 nax3 ...] vector indicating the number of xy
%         pairs to plot on each axes, where the number of axis will be
%         equal to the length of the vector.  The xy pairs will be taken in
%         order so the 1st nax1 pairs will be plotted on the 1rst axis,
%         the next nax2 pairs will be plotted on the 2nd axes and so on.
%         The length of ax_group must equal the number of xy pairs
% xrange = [xmin xmax] xlimits of plot   
% leg_space = [pixels] space to make available on right side of plot for
%                       legned (should be around 100 pixels)
% 
%
% Outputs: ax = double array containing the axes' handles, note that
%               ax(1) will be the handle for the first axes and data
%               ax(2) handle of second Y axis data sets
%               ax(3) handle of dummy axes to display Yticks for ax(2)
%               ax(4) handle of third Y axis data sets
%               ax(5) handle of dummy axes to display Yticks for ax(4)
%               etc.
%
%          hlines =  cell array containing the line handles (alternates
%          like ax with emty handles for dummy axis
%          fh = figure handle 
%
% Example:
% x1 = (0:0.01:1)';
% x2 = (0:0.1:1)';
% x3 = (0:0.05:1)';
% y1 = x1;
% y2 = x2.^2;
% y3 = x3.^3;
% y4 = sin(x1);
% y5 = fliplr(2*x1.^2);
% y6 = 7*cos(x1);
% y7 = 7*log(x1+1.2);
% [ax,hlines,fh] = jzplotys({x1,y1,x2,y2,x3,y3,x1,y4,x1,y5,x1,y6,x1,y7},[2 1 2 2],[.25 .75],100);
% legend(cat(1,hlines{:}),'a','b','c','d','e','f','g','location',[.88 .5 .01 .01])
% ylabel(ax(1),'Y1');
% ylabel(ax(3),'Y2');
% ylabel(ax(5),'Y3');
% ylabel(ax(7),'Y4');

%
% Inspired by multiplotyyy.m (available at www.matlabcentral.com) by :
% Laura L. Proctor
%
% Author: Josh Zimmerman
% May 21, 2014
%



validateattributes(xy_pairs,{'cell'},{})

if max(size(xy_pairs))/2 == sum(ax_groups)
    
    fh = figure('units','pixels');
    set(fh,'Color','w');
    
    % size figure based on number of Y axis
    nax = length(ax_groups); % numberof y-axis
    fpos_def = [400 400 560 420]; % default figure position and size
    y_ofst = 80; % [pixles] ofset for each Y axis to allow labels
    fpos = [fpos_def(1) fpos_def(2) (nax-1)*y_ofst+fpos_def(3)+leg_space fpos_def(4)];
    set(fh,'Position',fpos);
    
    % Preallocate the outputs
    ax = zeros(2*nax-1,1);
    hlines = cell(2*nax-1,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the first set of lines
    
    ax(1) = axes('Parent',fh);
    set(ax(1),'Units','pixels');    
    % determine axes size and position 
    pos = [fpos(3)-leg_space-.8*fpos_def(3) .13*fpos_def(4) .7*fpos_def(3) .8*fpos_def(4)];
    set(ax(1),'Position',pos);
    
    hlines{1} = plot(xy_pairs{1:2*ax_groups(1)},'Color','b');
    
    set(ax(1),'YColor','b','Color','none','Xlim',xrange)
    
    lines = {'-.';'-';'--';':'};
    nlines = numel(lines);
    % markers = set(hlines{1}(1),'Marker');
    % markers(end) = [];
    % nmarkers = numel(markers);
    colors = {'b';[0 .5 0];'r';'c';'m';'k'};
    ncol = numel(colors);
    
    if numel(hlines{1}) > 1
        for idx = 1:numel(hlines{1})
            set(hlines{1}(idx),'LineStyle',lines{rem(idx,nlines)+1}) % alternate line types
            set(hlines{1}(idx),'LineWidth',ceil(idx/nlines))  % change line size after all types are used
            %         if numel(hlines{1}) > 4
            %             set(hlines{1}(idx),'Marker',markers{rem(idx,nmarkers)+1});
            %         end
        end
    end
    y1tick = get(ax(1),'Ytick');
    hold on;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the remaining lines
    for i = 2:2*nax-1
        if rem(i,2) == 0  % create data axes
            ax(i) = axes('Parent',fh,'Units','pixels');
            ycol = colors{rem(i/2,ncol)+1}; % rotate Y axis colors
            hlines{i} = plot(xy_pairs{1+2*sum(ax_groups(1:i/2)):2*sum(ax_groups(1:i/2+1))},...
                'Color',ycol);
            set(ax(i),'Position',pos,'Color','none','Xlim',xrange,'Xtick',[],...
                'Ytick',[],'box','off');
            y2range = get(ax(i),'Ylim');
            y2tick = linspace(y2range(1),y2range(2),length(y1tick));
            y2tick = num2str(y2tick,4); % limit precision to 4
            y2tick = str2num(y2tick); % make number again
            pos2 = [pos(1)-y_ofst*i/2 pos(2) pos(3)+y_ofst*i/2 pos(4)];
            if numel(hlines{i}) > 1
                for idx = 1:numel(hlines{i})
                    set(hlines{i}(idx),'LineStyle',lines{rem(idx,nlines)+1}) % alternate line types
                    set(hlines{i}(idx),'LineWidth',ceil(idx/nlines))  % change line size after all types are used
                %             if numel(hlines{2}) > 4
                %                 set(hlines{2}(idx),'Marker',markers{rem(idx,nmarkers)+1});
                %             end
                end
            end
        else % create dummy axes
            ax(i) = axes('Parent',fh,'Units','pixels');
            set(ax(i),'Position',pos2,'Color','none','Xtick',[],'Ylim',y2range,...
                'Ytick',y2tick,'Ycolor',ycol,'Xcolor','w','box','off');
        end
    end
    
    
    % restack plots in order and normalize axis
    for i = 1:2*nax-1
        axes(ax(2*nax-i));
        set(ax(2*nax-i),'Units','Normalized');
    end
    
    % add legend placeholder
    if leg_space > 0
        for i = 1:sum(ax_groups)
            leg_data{i} = num2str(i);
        end
        axes(ax(1));
        leg_loc = [.88 .5 .01 .01];
        legend(cat(1,hlines{:}),leg_data,'Location',leg_loc)
    end
    
    grid(ax(1),'on');
    
    % link axis for zooming (must have linkzoom function in path)
    linkzoom(ax,'xy');
   
    % error in function inputs
    else
        display('Mismatch of # of xy pairs and specified ax_groups')
        fh = nan
        ax = nan
        hlines = nan
end

end
