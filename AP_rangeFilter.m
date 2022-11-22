function [P_range, A_range, L_tib,M_tib, O_distal_tib] = AP_rangeFilter(x,y,z)

%% Remove patella and fibula 
x_knee = x;
x_knee(6200:6750) = [];
y_knee = y;
y_knee(6200:6750) = [];
z_knee = z;
z_knee(6200:6750) = [];
Leg = [x_knee, y_knee, z_knee];

%% Plot leg without patella
figure;
plot3(x_knee,y_knee,z_knee,'k*')
axis equal
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')

%% Identify tibia: Filter points 0<z<400 
Tibia = Leg(Leg(:,3)<400,:);
Tibia = Tibia(Tibia(:,3)>0,:);
hold on
plot3(Tibia(:,1),Tibia(:,2),Tibia(:,3),'r*')

x_tib = Tibia(:,1);
y_tib = Tibia(:,2);
z_tib = Tibia(:,3);

%% Identify MLAP distal tibia bone
distal_tib = Leg(Leg(:,3)<-150,:);
distal_tib = distal_tib(distal_tib(:,3)>-210,:);
O_distal_tib = mean(distal_tib,1);
plot3(O_distal_tib(1),O_distal_tib(2),O_distal_tib(3),'r*')

%% Posterior point of tibia
post_tib_y = max(y_tib);
post_tib_ID = find(Tibia(:,2) == post_tib_y);
P_tib = Tibia(post_tib_ID,:);
P_ind = find(Tibia(:,2)>(post_tib_y-15));
P_range = zeros(length(P_ind),3);
for n = 1:length(P_ind)
    ind = P_ind(n);
    P_point = Tibia(ind,:);
    P_range(n,:) = P_point;
    hold on
    plot3(P_point(1),P_point(2),P_point(3),'b*')
end

%% Anterior point of tibia
% Anterior point in line with posterior point
A_inLineWithP = Tibia(Tibia(:,3)<(P_tib(3)+5) & Tibia(:,3)>(P_tib(3)-5),:); 
ant_tib_y = min(A_inLineWithP(:,2));
A_ind = find(A_inLineWithP(:,2)<(ant_tib_y+15));
A_range = zeros(length(A_ind),3);
for n = 1:length(A_ind)
    ind = A_ind(n);
    A_point = A_inLineWithP(ind,:);
    A_range(n,:) = A_point;
    plot3(A_point(1),A_point(2),A_point(3),'b*')
end

%% Lateral point of tibia
lat_tib_x = min(x_tib);
lat_tib_ID = find(Tibia(:,1) == lat_tib_x);
L_tib = Tibia(lat_tib_ID,:);

%% Medial point of tibia
med_tib_x = max(x_tib);
med_tib_ID = find(Tibia(:,1) == med_tib_x);
M_tib = Tibia(med_tib_ID,:);
