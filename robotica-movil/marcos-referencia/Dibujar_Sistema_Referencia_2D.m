function Dibujar_Sistema_Referencia_2D (aRb,atb,s)
    hold on
    grid on
    axis([-10 10 -10 10])
    xlabel('x')
    ylabel('y')

    apx = aRb*[1 0]' + atb;
    apy = aRb*[0 1]' + atb;

    line([atb(1) apx(1)],[atb(2) apx(2)],'color',[1 0 0],'LineWidth',3)
    line([atb(1) apy(1)],[atb(2) apy(2)],'color',[0 1 0],'LineWidth',3)
    plot(atb(1),atb(2),'b.','MarkerSize',25)
    text(atb(1)+0.5,atb(2)-0.5,s,'Color','blue','FontSize',15)
end