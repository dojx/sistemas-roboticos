function Dibujar_Omnidireccional_3 (p,L)
    x = p(1);
    y = p(2);
    theta = p(3);

    axis([-2.0 2.0 -2.0 2.0])
    hold on
    grid on
    xlabel('x')
    ylabel('y')
    
	Lo = L*0.2;
    lo = L*0.4;
    Tob1 = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
    Tob2 = [cos(theta+2*pi/3) -sin(theta+2*pi/3) x; sin(theta+2*pi/3) cos(theta+2*pi/3) y; 0 0 1];
    Tob3 = [cos(theta+4*pi/3) -sin(theta+4*pi/3) x; sin(theta+4*pi/3) cos(theta+4*pi/3) y; 0 0 1];
    Tb1 = [1 0 L; 0 1 0; 0 0 1];
    Tb2 = [1 0 L; 0 1 0; 0 0 1];
    Tb3 = [1 0 L; 0 1 0; 0 0 1];
    To1 = Tob1*Tb1;
    To2 = Tob2*Tb2;
    To3 = Tob3*Tb3;
    
    % Ruedas
    p1 = To1*[+Lo -lo 1]';
    p2 = To1*[-Lo -lo 1]';
    p3 = To1*[+Lo +lo 1]';
    p4 = To1*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2,'MarkerSize',10)
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',2,'MarkerSize',10)
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',2,'MarkerSize',10)
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',2,'MarkerSize',10)

    p1 = To2*[+Lo -lo 1]';
    p2 = To2*[-Lo -lo 1]';
    p3 = To2*[+Lo +lo 1]';
    p4 = To2*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2,'MarkerSize',10)
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',2,'MarkerSize',10)
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',2,'MarkerSize',10)
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',2,'MarkerSize',10)

    p1 = To3*[+Lo -lo 1]';
    p2 = To3*[-Lo -lo 1]';
    p3 = To3*[+Lo +lo 1]';
    p4 = To3*[-Lo +lo 1]';
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2,'MarkerSize',10)
    line([p1(1) p3(1)],[p1(2) p3(2)],'LineWidth',2,'MarkerSize',10)
    line([p2(1) p4(1)],[p2(2) p4(2)],'LineWidth',2,'MarkerSize',10)
    line([p3(1) p4(1)],[p3(2) p4(2)],'LineWidth',2,'MarkerSize',10)
    
    % Base
    phi = linspace(0,2*pi,50);

    cx = x+(L-Lo)*cos(phi); 
    cy = y+(L-Lo)*sin(phi);
    plot(cx,cy,'LineWidth',2,'MarkerSize',10)
    
    pc = Tob1*[L*0.4 0 1]';
    cx = pc(1)+L*0.15*cos(phi); 
    cy = pc(2)+L*0.15*sin(phi);
    plot(cx,cy,'LineWidth',2,'MarkerSize',10,'color',[1 0 0])