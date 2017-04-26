function object = drawLoad(states)

[x,y,z] = sphere;
r = 0.07;
x = r*x + states(1);
y = r*y + states(2);
z = r*z + states(3);

object = surf(x,y,z);

end