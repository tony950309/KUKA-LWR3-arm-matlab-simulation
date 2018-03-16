% Plots a square patch representing a plane
% m - centre point of the plane
% n - normal of the plane (magnitude will scale the normal length)
% n1 - patch 1st principal direction (magnitude = half width of the patch)
% n2 - patch 2st principal direction (magnitude = half height of the patch)
% [0,0,0] , [0,0,1], [0.5,0,0], [0,0.5,0]
function plotPlane(m, n, n1, n2)
    plot3([m(1),m(1)+n(1)],[m(2),m(2)+n(2)],[m(3),m(3)+n(3)], 'linewidth',2);
    hold on;
    [P,Q] = meshgrid([-1,1]);
    X = m(1)+n1(1)*P+n2(1)*Q;
    Y = m(2)+n1(2)*P+n2(2)*Q;
    Z = m(3)+n1(3)*P+n2(3)*Q;
    surf(X,Y,Z,'facealpha',0.2);
end