function [rotationMatrix] = rotationMatrix3D(a, b, c)
%returns the rotationMatrix in 3D, all angles have to be in radian
%authors: Simon Roß, Pablo Vega Perez

rotationMatrix = [cos(a)*cos(b) cos(a)*sin(b)*sin(c)-sin(a)*cos(c) cos(a)*sin(b)*cos(c)+sin(a)*sin(c);
                      sin(a)*cos(b) sin(a)*sin(b)*sin(c)+cos(a)*cos(c) sin(a)*sin(b)*cos(c)-cos(a)*sin(c);
                      -sin(b) cos(b)*sin(c) cos(b)*cos(c)];
end