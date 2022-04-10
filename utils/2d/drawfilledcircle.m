function circles = drawfilledcircle(x,y,r,c)
hold on
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
circles = plot(x_circle, y_circle,'b');
h=fill(x_circle, y_circle, c);
set(h,'EdgeColor','none');
hold off;
axis equal;
alpha(.5);
end