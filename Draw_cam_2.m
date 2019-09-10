clc;clear;close all;
dtr = pi/180;
beta_up = 180*dtr;
beta_down = 90*dtr;


L = 2 ;
rb = 5;
ro = 1;
origin = [0,0];
a1 = 0;
b0 = 10;   %any number
l0 = rb+b0+ro;
origin = [0 0 0 1]';
%% Follower motion

%Parabolic
for i = 1:1:180
    theta_star = i*dtr;
    if i <= 90
        b(i) = 2*L*(theta_star/beta_up)^2;
    elseif i > 90
        b(i) = L*(-1 + 4*(theta_star/beta_up) - 2*(theta_star/beta_up)^2);
    end
end
%Harmonic

for i = 180:1:361
    if i < 270
        
        theta_star = (i-180)*dtr;
        b(i) = L/2*(1+cos(pi*theta_star/beta_down));
        
    elseif i >270
        b(i) = 0;
    end
    
end


for i = 1:1:361
    b2(i) = b0-b(i);
end

%% 

for i = 1:1:361
    theta = i*dtr;
    
    A_1_0(:,:,i) = [cos(theta) -sin(theta) 0 0;
                    sin(theta) cos(theta)  0 0;
                     0      0      1    0;
                     0      0      0    1]*[1 0 0 0;
                                            0 0 -1 0;
                                            0 1 0 0;
                                            0 0 0 1]*[1 0 0 a1;
                                                      0 1 0 0;
                                                      0 0 1 l0;
                                                      0 0 0 1];
                                             
    A_2_1(:,:,i) = [1 0 0 0;
                    0 0 1 0;
                    0 -1 0 0;
                    0 0 0 1]*[1 0 0 0;
                              0 1 0 b2(i);
                              0 0 1 0;
                              0 0 0 1];         
    
    A_2_0(:,:,i) = A_1_0(:,:,i)*A_2_1(:,:,i);

    A_0_2(:,:,i) = inv(A_2_0(:,:,i));   
    roller_center(i,:) = A_2_0(:,:,i)*origin;    
    %roller coordinate seen in origin
    
    
    
%     roller_center(i,:) = roller_center(i,:)*[0 1 0 0;-1 0 0 0;0 0 1 0;0 0 0 1];    
    x(i) = roller_center(i,1);                
    y(i) = roller_center(i,2);    
    
    for j = 1:1:361
    theta_star = j*dtr;
    
    roller_path(j,1,i) =  roller_center(i,1) + ro*cos(theta_star);
    roller_path(j,2,i) =  roller_center(i,2) + ro*sin(theta_star);
    end
    
end

%% Find beta

%Parabolic
for i = 1:1:180
    theta = i*dtr;
    if i <= 90
        beta(i) = -atan2(l0-b2(i),a1 - 4*L/beta_up^2*theta);
    elseif i > 90
        beta(i) = -atan2(l0-b2(i),a1 - 4*L/beta_up + 4*L*theta/beta_up^2);
    end
end
%Harmonic

for i = 180:1:361
    
    if i < 270
        
        theta = (i-180)*dtr;
        beta(i) = -atan2(l0-b2(i),a1+L/2*pi/beta_down*sin(pi*theta/beta_down));
        
    elseif i >=270
        beta(i) = -atan2(l0-b2(i),a1);
    end
    
end







%% Find contact point

for i = 1:1:361
    theta = i*dtr;
    contect_point(i,:) = [x(i)+ro*cos(beta(i)+theta+pi),y(i)+ro*sin(beta(i)+theta+pi)];

 
end




figure(1)
figure_chienli2(300,260,0.23,0.25,0.7,0.6)
hold all
plot(origin(1),origin(2),'o','MarkerSize',8,'color','r'); 

plot(contect_point(:,1),contect_point(:,2),'r')

xlabel('\it{x} (mm)'); 
ylabel('\it{y} (mm)');
title('Total diagram');

plot(x,y,'black','Linewidth',1.5);
axis equal

%% Plot path
for i = 1:5:360   
    
    hold on
    plot(roller_path(:,1,i),roller_path(:,2,i),'b');
    pause(0.01);
end


% figure(1)
% figure_chienli2(300,260,0.23,0.25,0.7,0.6)
% plot(beta);
% xlabel('\circ (degree)'); 
% ylabel('\circ (rad)');
% title('\beta');
% xlim([0, 360]);

 



















