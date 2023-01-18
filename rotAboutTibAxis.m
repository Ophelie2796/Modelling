function [v_rot] = rotAboutTibAxis(O, v, K, angle)
%% Rotation about an axis passing through the origin:
% O = [0,0,0]; % origin
% v = [1,0,0]; % v is the vector to be rotated 
% K = [-0.3787,-0.0275,-0.9251]; % K denotes the distal point for rotation axis
k = K/norm(K); % k is the normalized rotation axis vector 

%plot3([O(1),v(1)],[O(2),v(2)],[O(3),v(3)],'-*k'); % plot initial vector
hold on;
%plot3([O(1),K(1)],[O(2),K(2)],[O(3),K(3)],'-k'); % plot rotation axis
for theta = angle % define the angle of rotation (right-handed CS)
    v_rot = v*cosd(theta)+cross(k,v)*sind(theta)+k*(dot(k,v))*(1-cosd(theta)); % rotated vector (Rodrigues' formula)
    %plot3([O(1),v_rot(1)],[O(2),v_rot(2)],[O(3),v_rot(3)],':.k','LineWidth',1);
    hold on;
end
% title('Rotation about an axis through the origin');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% axis equal