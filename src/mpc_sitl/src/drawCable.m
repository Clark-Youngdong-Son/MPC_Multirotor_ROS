function object = drawCable(states_l,p,l)

x_l = states_l(1);
y_l = states_l(2);
z_l = states_l(3);

origin = [x_l y_l z_l].';

x_c = x_l -p(1)*l;
y_c = y_l -p(2)*l;
z_c = z_l -p(3)*l;

object{1} = plot3([origin(1) x_c],[origin(2) y_c],[origin(3) z_c],'-b','LineWidth',2); hold on;

end