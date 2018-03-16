% This code depends on Peter Corke's Robotics Toolbox:
% http://petercorke.com/wordpress/toolboxes/robotics-toolbox

%--------------------------------------------------
% Robot setup for the assignment 

% Create a Kuka LWR3 from Denavit-Hartnbergh parameters
%%
clear
clc
close all
% Recorded datasets
load 'dataset.mat'
DH = [0.0, 0.31, 0.0, pi/2;
      0.0, 0.0, 0.0, -pi/2;
      0.0, 0.4, 0.0, -pi/2;
      0.0, 0.0, 0.0, pi/2;
      0.0, 0.39, 0.0, pi/2;
      0.0, 0.0, 0.0, -pi/2;
      0.0, 0.21, 0.0, 0.0];
robot = SerialLink(DH);

% Weld line ,centre :0.7 0.1 0.4
line=-0.3:0.01:0.3;
%% Q1 Experimental data preparation: Generated dataset
clc;
position0 = zeros(length(line),3);
data0 = [];
for i=1:length(line)
   tr1 = SE3(0, line(i), 0);
   tr2 = SE3.rpy(30, 0, 0);         % matlab 2017b   degrees
   tr3 = SE3(0.7,0.1,0.4);
   position0(i,:) = tr3 * tr2 * tr1.t;
   data0 = [data0; ikunceff(robot, SE3( position0(i,:) ))  ];
end
x_begin = data0(1,:);
x_end   = data0(end,:);

%%  Q1 1 plot
f = figure('pos',[10 10 800 600]);
robot.plot(x_begin);
title('Begin');
input('Press enter to continue');
f.delete()

g = figure('pos',[10 10 800 600]);
robot.plot(x_end);
title('End');
input('Press enter to continue');
g.delete()


% weld_line = ones(length(line),3);
% weld_line(:,1) = 0;
% weld_line(:,2) = line;
% weld_line(:,3) = 0;
% weld_line_rotated = weld_line * Rx(-pi / 6);
% 
% weld_line(:,1) = weld_line_rotated(:,1) + 0.7;
% weld_line(:,2) = weld_line_rotated(:,2) + 0.1;
% weld_line(:,3) = weld_line_rotated(:,3) + 0.4;
% position0 = weld_line;
% x_Q1 =[];
% for i=1:length(weld_line_rotated)
%     x_Q1 = [x_Q1;ikunceff(robot, SE3(weld_line_rotated(i,:)))];
% end
% data0 = x_Q1;
% figure('pos',[10 10 800 600]);
% makevideo('Q1_validation', 30, length(x_Q1), @(i) robot.plot( x_Q1(i,:),'noarrow','view',[150 30] ));


%% Q1 Experimental data preparation: Recorded datasets
clc;
data1 = dataset{1,1};
data2 = dataset{2,1};
data3 = dataset{3,1};
position1 = [];
position2 = [];
position3 = [];
for i=1:length(data1)
    position1 = [position1; (robot.fkine(data1(i,:)).t)' ];
    position2 = [position2; (robot.fkine(data2(i,:)).t)' ];
    position3 = [position3; (robot.fkine(data3(i,:)).t)' ];
    
end
%% plot
figure
plot3(position1(:,1),position1(:,2),position1(:,3),'r',...
    position2(:,1),position2(:,2),position2(:,3),'g',...
    position3(:,1),position3(:,2),position3(:,3),'b');
legend('first dataset','second dataset','third dataset')
title('Q1_2:plot containing the end-effector trajectories from all three datasets')
grid on

%% 2 Video 1) load the data and IK
clc;
x_Q2 = [data0; data1;data2;data3];

%% 2)save the video. This process takes minutes!!!!!!!!!!!!!!!!!!!
figure('pos',[10 10 800 600]);
makevideo('Q2_video', 30, length(x_Q2), @(i) robot.plot( x_Q2(i,:),'noarrow','view',[150 30] ));

%% 3 Dimensionality analysis: Joint angle data analysis
clc;
[~,~,latent_angle0,~,explained_angle0,~] = pca(data0);
[~,~,latent_angle1,~,explained_angle1,~] = pca(data1);
[~,~,latent_angle2,~,explained_angle2,~] = pca(data2);
[~,~,latent_angle3,~,explained_angle3,~] = pca(data3);
disp('Eigenvalue');
latent_angle0'
latent_angle1'
latent_angle2'
latent_angle3'

%% End-effector position data analysis
clc;
[coeff0,~,latent_position0,~,explained_position0,mu0] = pca(position0);
[coeff1,~,latent_position1,~,explained_position1,mu1] = pca(position1);
[coeff2,~,latent_position2,~,explained_position2,mu2] = pca(position2);
[coeff3,~,latent_position3,~,explained_position3,mu3] = pca(position3);
disp('Eigenvalue');
latent_position0'
latent_position1'
latent_position2'
latent_position3'

%% Dimensionality analysis
clc;
disp('7D joint angle data')

explained_angle0'
explained_angle1'
explained_angle2'
explained_angle3'

largerThan99(explained_angle0)
largerThan99(explained_angle1)
largerThan99(explained_angle2)
largerThan99(explained_angle3)

disp('3D position data')

explained_position0'
explained_position1'
explained_position2'
explained_position3'

largerThan99(explained_position0)
largerThan99(explained_position1)
largerThan99(explained_position2)
largerThan99(explained_position3)


%% Validation against ground-truth data
% no code for this subsection

%% 4 Model fitting: Model fitting on generated data
clc;
M0 = [ones(length(position0) , 1 ), position0(:,1), position0(:,2)];
z0 =  position0(:,3);

a0 = M0 \ z0;
z_observe0 = [ones(length(position0),1), position0(:,1), position0(:,2)] * a0;
rmse0 = RMSE(z0,z_observe0)
%% Model fitting on recorded data
clc;
% % data 1 
M1 = [ones(length(position1) , 1 ), position1(:,1), position1(:,2)];
z1 =  position1(:,3);
a1 = M1 \ z1;
z_observe1 = [ones(length(position1),1), position1(:,1), position1(:,2)]...
            * a1;
rmse1 = RMSE(z1,z_observe1)

% % data 2 
M2 = [ones(length(position2) , 1 ), position2(:,1), position2(:,2)];
z2 =  position2(:,3);
a2 = M2 \ z2;
z_observe2 = [ones(length(position2) , 1 ), position2(:,1), position2(:,2)]...
            * a2;
rmse2 = RMSE(z2,z_observe2)

% % data 3 
M3 = [ones(length(position3) , 1 ), position3(:,1), position3(:,2)];
z3 =  position1(:,3);
a3 = M3 \ z3;
z_observe3 = [ones(length(position3) , 1 ), position3(:,1), position3(:,2)]...
            * a3;
rmse3 = RMSE(z3,z_observe3)
%% plot plane
figure
plot3(position1(:,1),position1(:,2),position1(:,3),'r',...
    position2(:,1),position2(:,2),position2(:,3),'g',...
    position3(:,1),position3(:,2),position3(:,3),'b');
legend('first dataset','second dataset','third dataset')
title('Q4_2')
grid on
hold on
plotPlane(mu1 , coeff1(:,3), coeff1(:,1), coeff1(:,2))
hold on
plotPlane(mu2 , coeff2(:,3), coeff2(:,1), coeff2(:,2))
hold on
plotPlane(mu3 , coeff3(:,3), coeff3(:,1), coeff3(:,2))
%% plot indivual  1
figure
plot3(position1(:,1),position1(:,2),position1(:,3),'r')
hold on
title('Q4 1st plane')
grid on
plotPlane(mu1 , coeff1(:,3), coeff1(:,1), coeff1(:,2))
axis equal

%% plot indivual   2
figure
plot3(position2(:,1),position2(:,2),position2(:,3),'r')
hold on
title('Q4 2nd plane')
grid on
plotPlane(mu2 , coeff2(:,3), coeff2(:,1), coeff2(:,2))
axis equal

%% plot indivual   3
figure
plot3(position3(:,1),position3(:,2),position3(:,3),'r')
hold on
title('Q4 3rd plane')
grid on
plotPlane(mu3 , coeff3(:,3), coeff3(:,1), coeff3(:,2))
axis equal




%%
function rmse =RMSE(a,b)
    rmse = sqrt( sum( (a-b).^2 ) / length(a) );
end


function [R_x] = Rx(theta)
    R_x = [1     0           0
          0  cos(theta)  -sin(theta)
          0  sin(theta)   cos(theta)];
end


function [n] = largerThan99(explained)
    sum = 0;
    n = 0;
    for i=1:length(explained)
        sum = sum + explained(i);
        if sum>=99
            n = i;
            break
        end
    end
end