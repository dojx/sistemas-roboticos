function Dibujar_Sistema_Referencia_3D (aTb,s)
    hold on
    grid on
    axis([-10 10 -10 10 0 10])
    xlabel('x')
    ylabel('y')
    zlabel('z')

    apx = aTb*[2 0 0 1]';
    apy = aTb*[0 2 0 1]';
    apz = aTb*[0 0 2 1]';
    
    atb = aTb(1:3,4);
    
    line([atb(1) apx(1)],[atb(2) apx(2)],[atb(3) apx(3)],'color',[1 0 0],'LineWidth',3)
    line([atb(1) apy(1)],[atb(2) apy(2)],[atb(3) apy(3)],'color',[0 1 0],'LineWidth',3)
    line([atb(1) apz(1)],[atb(2) apz(2)],[atb(3) apz(3)],'color',[0 0 1],'LineWidth',3)
    
    plot3(atb(1),atb(2),atb(3),'k.','MarkerSize',25)
    text(atb(1),atb(2),atb(3)-0.75,s,'Color','blue','FontSize',15)
    view([-45,30])
end