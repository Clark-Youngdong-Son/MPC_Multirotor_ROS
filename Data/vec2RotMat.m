function R = vec2RotMat(phi,theta,psi)

%Rotation matrix from 'body' to 'inertia' frame
%X : -phi, Y : -theta, Z : -psi
rotX = [1       0          0;
    0 cos(phi) -sin(phi);
    0 sin(phi)  cos(phi)];
rotY = [cos(theta)   0 sin(theta);
    0            1          0;
    -sin(theta)  0 cos(theta)];
rotZ = [cos(psi) -sin(psi) 0;
    sin(psi)  cos(psi) 0;
    0         0 1];
R = rotZ*rotY*rotX;

end