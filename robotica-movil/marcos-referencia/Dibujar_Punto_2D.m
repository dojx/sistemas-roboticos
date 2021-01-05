function Dibujar_Punto_2D (ax)
    hold on
    grid on
    axis([-10 10 -10 10])
    xlabel('x')
    ylabel('y')

    plot(ax(1),ax(2),'bx','MarkerSize',10,'LineWidth',2)
    plot(ax(1),ax(2),'ro','MarkerSize',10,'LineWidth',2)
end