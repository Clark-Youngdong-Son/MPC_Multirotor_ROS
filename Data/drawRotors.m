function objects = drawRotors(states,armlength)
rotMat = @(angle) [cos(angle) -sin(angle); sin(angle) cos(angle)];
arm1 = [-armlength armlength; 0 0];
arm2 = rotMat(90*pi/180)*arm1;
% arm3 = rotMat(180*pi/180)*arm1;
% arm4 = rotMat(270*pi/180)*arm1;

x = states(1);
y = states(2);
z = states(3);
phi = states(4);
theta = states(5);
psi = states(6);

objects = cell(6,1);

arm1 = vec2RotMat(phi,theta,psi)*[arm1; zeros(1,2)] + [[x y z].' [x y z].'];
arm2 = vec2RotMat(phi,theta,psi)*[arm2; zeros(1,2)] + [[x y z].' [x y z].'];
% arm3 = vec2RotMat(phi,theta,psi)*[arm3; zeros(1,2)] + [[x y z].' [x y z].'];
% arm4 = vec2RotMat(phi,theta,psi)*[arm4; zeros(1,2)] + [[x y z].' [x y z].'];

objects{1} = plot3(arm1(1,:),arm1(2,:),arm1(3,:),'-k','LineWidth',2); hold on;
objects{2} = plot3(arm2(1,:),arm2(2,:),arm2(3,:),'-k','LineWidth',2);
% objects{3} = plot3(arm3(1,:),arm3(2,:),arm3(3,:),'-k','LineWidth',3);
% objects{4} = plot3(arm4(1,:),arm4(2,:),arm4(3,:),'-k','LineWidth',3);

end