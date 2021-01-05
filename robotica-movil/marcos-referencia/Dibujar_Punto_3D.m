function Dibujar_Punto_3D (at)
    hold on
    grid on
    axis([-10 10 -10 10 0 10])
    xlabel('x')
    ylabel('y')
    zlabel('z')

    plot3(at(1),at(2),at(3),'bx','MarkerSize',10,'LineWidth',2)
    plot3(at(1),at(2),at(3),'ro','MarkerSize',10,'LineWidth',2)
    view([-45,30])
end