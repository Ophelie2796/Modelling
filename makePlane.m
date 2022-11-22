function [L_tib_plane, M_tib_plane, A_tib_plane, P_tib_plane] = makePlane(L_tib, M_tib, A_tib, P_tib)

%% Objective: Adjust the MLAP tibial plateau points to all be on the same Z
% plane

%%
tib_plateau = [L_tib; M_tib;A_tib;P_tib];
z_avg = mean(tib_plateau(:,3),1);
tib_plateau(:,3) = z_avg;

L_tib_plane = tib_plateau(1,:);
M_tib_plane = tib_plateau(2,:);
A_tib_plane = tib_plateau(3,:);
P_tib_plane = tib_plateau(4,:);
