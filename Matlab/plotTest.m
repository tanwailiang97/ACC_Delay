 x1 = (0:0.01:1)';
 x2 = (0:0.1:1)';
 x3 = (0:0.05:1)';
 y1 = x1;
 y2 = x2.^2;
 y3 = x3.^3;
 y4 = sin(x1);
 y5 = fliplr(2*x1.^2);
 y6 = 7*cos(x1);
 y7 = 7*log(x1+1.2);
 [ax,hlines,fh] = jzplotys({x1,y1,
                             x2,y2,
                             x3,y3,
                             x1,y4,
                             x1,y5,
                             x1,y6,
                             x1,y7},[2 1 2 2],[.25 .75],100);
 legend(cat(1,hlines{:}),'a','b','c','d','e','f','g','location',[.88 .5 .01 .01])
 ylabel(ax(1),'Y1');
 ylabel(ax(3),'Y2');
 ylabel(ax(5),'Y3');
 ylabel(ax(7),'Y4');