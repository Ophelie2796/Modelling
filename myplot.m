function myplot(origin,x,y,z,xLineColor,yLineColor,zLineColor,originColor,lineWidth)

    plot3([origin(1) x(1)],[origin(2) x(2)],[origin(3) x(3)], xLineColor,'LineWidth',lineWidth)
    hold on
    plot3([origin(1) y(1)],[origin(2) y(2)],[origin(3) y(3)], yLineColor,'LineWidth',lineWidth)
    plot3([origin(1) z(1)],[origin(2) z(2)],[origin(3) z(3)], zLineColor,'LineWidth',lineWidth)
    plot3([origin(1)], [origin(2)],[origin(3)],originColor)