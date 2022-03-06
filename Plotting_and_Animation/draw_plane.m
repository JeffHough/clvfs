function [drawing] = draw_plane(point, n, size)
% Draws a plane based on a single point and a normal vector "n":

   w = null(n'); % Find two orthonormal vectors which are orthogonal to v
   [P,Q] = meshgrid(-size:2*size:size); % Provide a gridwork (you choose the size)
   X = point(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
   Y = point(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
   Z = point(3)+w(3,1)*P+w(3,2)*Q;
   drawing = surf(X,Y,Z);

end

