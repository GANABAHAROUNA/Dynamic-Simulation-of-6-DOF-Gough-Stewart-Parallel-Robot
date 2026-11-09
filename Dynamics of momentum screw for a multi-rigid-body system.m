clc;
clear;


%% Define Parameters
r = 0.4;                          %% Radius of the moving platform
M = 40;                           %% Mass of the moving platform
J = diag([1.6; 1.6; 3.2]);        %% Inertia matrix of the moving platform


miu= 0.2;                         %% Damping
g = [0; 0; -9.8];                 %% Gravity vector


%%Parameters for lower limb
l_1 = 0.8;
m_1 = 8;
J_1 = diag([0.376; 0.376; 0.0114]);


%%Parameters for upper limb
l_2 = 0.6;
m_2 = 12;
J_2 = diag([0.241; 0.241; 0.0114]);


%%Platform radius
R = 0.6;
n = 20000;


%%forces F_E components
F_Ex = 60;  
F_Ey = 50;
F_Ez = 40;

F = [F_Ex; F_Ey; F_Ez ];


%%Torque components
M_Ex = 6;  
M_Ey = 8;
M_Ez = 15;

T_load = [M_Ex; M_Ey; M_Ez];


iteration_time = 20;  
zero = zeros(3, 1);
time = linspace(0, iteration_time, n);   


% Initialize variables Pre-allocate
theta_Om = zeros(1, n);         
x_Om = zeros(1, n);                
y_Om = zeros(1, n);                
z_Om = zeros(1, n);                
r_Om = zeros(3, n);                
alpha_Om = zeros(1, n);            
beta_Om = zeros(1, n);             
gamma_Om = zeros(1, n);           
omega_alpha_om = zeros(1, n);      
omega_beta_om = zeros(1, n);       
omega_gamma_om = zeros(1, n);      
omega_platform = zeros(3, n);      
T = zeros(3, 3, n);                
r_A = zeros(3, 6, n);                    
r_OmC = zeros(3, 6, n);            
SS = zeros(6, 6, n);   
S = zeros(3, 6, n); 
r_C = zeros(3, 6, n);              
l = zeros(6, n);                    
x_l = zeros(6, n);                  
x_u = zeros(6, n);                  
e = zeros(3, 6, n);                
e_p = zeros(3, 6, n);             
e_Ax = zeros(3, 6, n);             
e_Ay = zeros(3, 6, n);             
x_l_direction = zeros(3, 6, n);    
x_u_direction = zeros(3, 6, n);  
K = zeros(3, 6, n);
tau = zeros(3, 6, n);
Screw_moving_platform = zeros(6, n);   
r_C_initial = zeros(3, 6);
v_1 = zeros(3, 6, n);
v_2 = zeros(3, 6, n);
v_1_value = zeros(1, 6, n);
J_L = zeros(3, 3, n);
J_M = zeros(3, 3, n);
J_U = zeros(3, 3, n);
Leg = zeros(3, 6);
Leg_norm = zeros(1, 6);
Rt = zeros(3, 3, n);
r_Uc = zeros(3, 1, n);
r_om_initial = zeros(3,1);
W_1 = zeros(6, 1, n);
wl = zeros(3, 6, n);
P_1 = zeros(3, 6, n);
P_2 = zeros(3, 6, n);
P_Lower = zeros(3, 6, n);
P_Upper = zeros(3, 6, n);
P_moving = zeros(3, 6, n);
P_Omx = zeros(1, n);
P_Omy = zeros(1, n);
P_Omz = zeros(1, n);
momentum_P1 = zeros(3, n);
momentum_P2 = zeros(3, n);
momentum_moving_platform_d = zeros(3, n);
momentum_moving_platform = zeros(3, n);
nu = zeros(3, 6, n);
diff_Screw_moving_platform = zeros(6, 1, n);
external_loads_W_1 = zeros(6, 1, n);
r_Lc = zeros(3, 1, n);
diff_Lower = zeros(3, 6, n);
diff_upper = zeros(3, 6, n);
displacement_leg  = zeros(1,6);
W_Lower = zeros(6, 6, n);
W_Upper = zeros(6, 6, n);
W_Lower_without_friction = zeros(6, 6, n);
W_Upper_without_friction = zeros(6, 6, n);
f = zeros(30,1, n);
f_without_friction = zeros(30,1, n);
matrix_W = zeros(78,1, n);
matrix_W_without_friction = zeros(78, 1, n);
Matrix_S = zeros(78,30, n);
r_P = zeros(3,6, n);
L_1 = zeros(6, 5, n);
L_2 = zeros(6, 5, n);
L_3 = zeros(6, 5, n);
L_4 = zeros(6, 5, n);
L_5 = zeros(6, 5, n);
L_6 = zeros(6, 5, n);
U_1 = zeros(6, 5, n);
U_2 = zeros(6, 5, n);
U_3 = zeros(6, 5, n);
U_4 = zeros(6, 5, n);
U_5 = zeros(6, 5, n);
U_6 = zeros(6, 5, n);
M_1 = zeros(6, 5, n);
M_2 = zeros(6, 5, n);
M_3 = zeros(6, 5, n);
M_4 = zeros(6, 5, n);
M_5 = zeros(6, 5, n);
M_6 = zeros(6, 5, n);
momentum_momemtum_Upper = zeros(3, 6, n);
momentum_momentum_Lower = zeros(3, 6, n);
Screw_Upper = zeros(6, 6, n);
Screw_Lower = zeros(6, 6, n);
derivative_Screw_Upper = zeros(6, 6, n);
derivative_Screw_Lower = zeros(6, 6, n);
external_loads_Limb_Upper = zeros(6, 6, n);
external_loads_Limb_Lower = zeros(6, 6, n);
external_loads_Limb_Upper_without_friction = zeros(6, 6, n);
external_loads_Limb_Lower_without_friction = zeros(6, 6, n);
v_c = zeros(3, 6, n);
v_Om = zeros(3, 6, n); 
damping_only = zeros(6, 6, n);



%% Define the motion of the moving platform
theta_deg_b = [0, 30, 120, 150, 240, 270];             %%Moving joint angles in degrees
theta_deg_m = [330, 60, 90, 180, 210, 300];            %%Moving joint angles in degrees


theta_rad_m = deg2rad(theta_deg_m);                    %%Convert angles to radians
theta_rad_b = deg2rad(theta_deg_b);                    %%Convert angles to radians


for t = 1:n
     %% Centrode position of the moving platform
    x_Om(t) = 0.1 * exp(-0.25 * time(t)) * cos(2 * pi * time(t));
    y_Om(t) = 0.1 * exp(-0.25 * time(t)) * sin(2 * pi * time(t));
    z_Om(t) = 0.05 * time(t) + 0.78; 
    r_Om(:, t) = [x_Om(t); y_Om(t); z_Om(t)];

    initial_time = 0;
    r_om_initial = [0.1 * exp(-0.25 * initial_time ) * cos(2 * pi * initial_time );0.1 * exp(-0.25 * initial_time ) * sin(2 * pi * initial_time );0.05 * initial_time+0.78];
    
    %%Platform orientation
    alpha_Om(t) = 0.2 * pi * cos(time(t));                    %% Roll angle
    beta_Om(t) = pi / 6 * cos(time(t));                       %% Pitch angle
    gamma_Om(t) = -0.2 * pi * cos(time(t));                   %% Yaw angle

    angle_Platform = [alpha_Om(t); beta_Om(t); gamma_Om(t)];

    %%omega_platform deduction 
    omega_alpha_om(t) = -0.2*pi*sin(time(t));
    omega_beta_om(t) = -pi/6*sin(time(t));
    omega_gamma_om(t) = 0.2*pi*sin(time(t));
 
    omega_platform(:, t) = [omega_alpha_om(t); omega_beta_om(t); omega_gamma_om(t)];

    %%Transformation matrix T (rotation matrix)
    T(:,:,t) = [1 0 0; 0 cos(alpha_Om(t)) -sin(alpha_Om(t)); 0 sin(alpha_Om(t)) cos(alpha_Om(t))] * ...
                [cos(beta_Om(t)) 0 sin(beta_Om(t)); 0 1 0; -sin(beta_Om(t)) 0 cos(beta_Om(t))] * ...
                [cos(gamma_Om(t)) -sin(gamma_Om(t)) 0; sin(gamma_Om(t)) cos(gamma_Om(t)) 0; 0 0 1];


%% The for loop code
    for i = 1:6
        r_A(:, i, t) = [R * cos(theta_rad_b(i)); R * sin(theta_rad_b(i)); 0];
        r_OmC(:, i, t) = [r * cos(theta_rad_m(i)); r * sin(theta_rad_m(i)); 0];
        S(:, i, t) = r_Om(:, t) + T(:,:,t) * r_OmC(:, i, t) - r_A(:, i, t);
        r_C(:, i, t) = r_Om(:, t) + T(:,:,t) * r_OmC(:, i, t);
        r_C_initial(:, i) = r_Om(:, 1) + T(:,:,1) * r_OmC(:, i, 1);
        Leg(:, i) = r_C(:, i, 1)-r_A(:, i, t);
        Leg_norm(:, i) = norm(Leg(:, i) );
        l(i, t) = norm(S(:, i, t));  


        %% Oscillation radii of the lower and upper limbs
        x_l(i, t) = l_1 / 2;                                                                       %% Lower limb oscillation radius
        x_u(i, t) = (l(i, t) - l_2 / 2);                                                           %% Upper limb oscillation radius
        e(:, i, t) = S(:, i, t) / l(i, t);                                                         %% Unit vector in direction of AiCi
        e_p(:, i, t) = [S(1, i, t); S(2, i, t); 0] / sqrt(S(1, i, t)^2 + S(2, i, t)^2);            %% Projection in the horizontal plane
        e_Ax(:, i, t) = cross(e(:, i, t), e_p(:, i, t)) / norm(cross(e(:, i, t), e_p(:, i, t)));   %% x-direction in the horizontal plane
        e_Ay(:, i, t) = cross(e(:, i, t), e_Ax(:, i, t));                                          %% y-direction in the horizontal plane
        x_l_direction(:, i, t) = l_1 / 2 * e(:, i, t);                                             %% Oscillation radius of the lower limb's center of mass
        x_u_direction(:, i, t) = (l(i, t) - l_2 / 2) * e(:, i, t);                                 %% Oscillation radius of the upper limb's center of mass
        K(:, i, t) = (cross(g,e(:, i, t) ))/(norm(cross(g,e(:, i, t) )));
        tau(:, i, t)  = cross( K(:, i, t), e(:, i, t) );

        %%Derivatives
        v_c(:, i, t) = gradient( S(:, i, t) , t);
        v_Om(:, i, t) = gradient( r_Om(:, t), t); 

    end
end



%% Forces and Torques Calculation
for t = 1:n
    for i = 1:6
        %%Calculate velocities, directions, and moments
        v_1(:, i, t) = v_c(:, i, t)' * e(:, i, t) * e(:, i, t);             %%Projection of velocity onto direction e
        v_1_value(:, i, t) = v_c(:, i, t)' * e(:, i, t);                    %%V_C1
        v_2(:, i, t) = v_c(:, i, t) - v_1(:, i, t);                         %%V_C2
        
        %%Rotation matrix for leg direction (based on the local axis)
        Rt(:, :, t) = [e_Ax(:, i, t), e_Ay(:, i, t), e(:, i, t)];        %%Rotation matrix for leg direction

        %%Nu is the unit vector in the direction of the leg for cross product calculations
        nu(:, i, t) = cross([sin(theta_rad_b(i)); -cos(theta_rad_b(i)); 0], cross(e(:, i, t), [sin(theta_rad_b(i)); -cos(theta_rad_b(i)); 0]));

        %%Angular velocity (using the cross product of the velocity and the vector)
        wl(:, i, t) = (cross(e(:, i, t), v_c(:, i, t))) / l(i, t) - cross(e(:, i, t), v_c(:, i, t)) * dot(cross(e(:, i, t), v_c(:, i, t)), nu(:, i, t)) / (l(i, t) * dot(e(:, i, t), nu(:, i, t)));

        %%Forces and torques for the lower limb
        P_1(:, i, t) = m_1 * cross(e(:, i, t), cross(wl(:, i, t), l_1 / 2 * e(:, i, t)));

        %%Inertia matrix for lower limb (J_L)
        J_L(:,:,t) = Rt(:,:,t) * J_1 * Rt(:,:,t)'; 
        
        %%Forces and torques for the upper limb
        P_2 (:, i, t) = m_2 * v_1(:, i, t) + m_2 * cross(wl(:, i, t), x_u(i, t) * e(:, i, t));

        %%Inertia matrix for upper limb (J_U)
        J_U(:,:,t) = Rt(:,:,t) * J_2 * Rt(:,:,t)';

        %%Platform inertia
        J_M(:,:,t) = T(:,:,t) * J * T(:,:,t);                         %%Update inertia matrix for the moving platform
        
        %%%The momentum of the moving platform
        P_moving(:, t) = M * v_Om(:, i, t);                             

        %%Update platform momentum
        momentum_P1(:, t) = cross(x_l_direction(:, i, t), P_1(:, i, t)) + J_1 * wl(:, i, t);
        momentum_P2(:, t) = cross(x_u_direction(:, i, t), P_2(:, i, t)) + J_2 * wl(:, i, t);

        %%Platform angular momentum
        momentum_moving_platform_d(:, t) = J_M(:,:,t) * omega_platform(:, t);

        %%Calculate the momentum components along each axis
        P_Omx(:, t) = M * cross(r_Om(:, t), v_Om(:, t))' * [1; 0; 0] + momentum_moving_platform_d(1,t);
        P_Omy(:, t) = M * cross(r_Om(:, t), v_Om(:, t))' * [0; 1; 0] + momentum_moving_platform_d(2,t);
        P_Omz(:, t) = M * cross(r_Om(:, t), v_Om(:, t))' * [0; 0; 1] + momentum_moving_platform_d(3,t);

        %%Total momentum of the moving platform
        momentum_moving_platform(:, t) = [P_Omx(:, t); P_Omy(:, t); P_Omz(:, t)];

        %%Screws for the moving platform
        Screw_moving_platform(:, t) = [P_moving(1,t); P_moving(2,t); P_moving(3,t); P_Omx(:, t); P_Omy(:, t); P_Omz(:, t)];
        
        %Derivative of the scre moving platform
        diff_Screw_moving_platform(:, :, t) = gradient( Screw_moving_platform(:, t), t);
        
        %%External loads acting on the moving system M = T
        external_loads_W_1(:, :, t) =[F; cross(r_Om(:, t), F ) + T_load] + [M*g; cross(r_Om(:, t), M*g )];

        %%Total force and torque os moving platform
        W_1(:, :, t) = diff_Screw_moving_platform(:, :, t) -  external_loads_W_1(:, :, t);

        
  
         %% Upper limb
        
        displacement_leg(:, i) = norm(S(:, i, t)) - Leg_norm(:, i);
        r_Uc( :, i, t) = (displacement_leg(:, i)+l_2/2+l_1)*e(:, i, t);
        P_Upper(:,i, t) = m_2 * dot(v_c(:, i, t),e(:, i, t))*e(:, i, t) + m_2*cross(wl(:, i, t),r_Uc( :, i, t));                              %%Use the appropriate time step for v_Om
        momentum_momemtum_Upper(:,i, t) = cross((r_Uc( :, i, t) + r_A(:, i, t)), P_Upper(:,i, t))  + J_U(:,:,t)*wl(:, i, t);
        Screw_Upper(:,i, t) = [P_Upper(:,i, t);momentum_momemtum_Upper(:,i, t)];
        derivative_Screw_Upper(:,i, t) = gradient( Screw_Upper(:,i, t) ,t);

        external_loads_Limb_Upper_without_friction(:, i, t) = [m_2*g; cross( r_Uc(:, i, t), m_2*g)];
        external_loads_Limb_Upper(:, i, t) = [m_2*g; cross( r_Uc(:, i, t), m_2*g)] + (miu*v_1_value(:, i, t) + 20*sign(v_1_value(:, i, t)))*[e(:, i, t); cross( r_Uc(:, i, t), e(:, i, t))]; % Damping force.
        
        W_Upper(:,i, t) = derivative_Screw_Upper(:,i, t) +  external_loads_Limb_Upper(:, i, t);
        W_Upper_without_friction(:,i, t) = derivative_Screw_Upper(:,i, t) +  external_loads_Limb_Upper_without_friction(:, i, t);
      
        
        %% Lower limb
        r_Lc(:, i, t) = l_1/2*e(:, i, t);
        P_Lower(:,i, t) = m_1 *cross(wl(:, i, t),r_Lc(:, i, t));  
        momentum_momentum_Lower(:,i, t) = cross((r_Lc(:, i, t)+ r_A(:, i, t)), P_Lower(:,i, t)) +  J_L(:,:,t)*wl(:, i, t);
        Screw_Lower(:,i, t) = [P_Lower(:,i, t); momentum_momentum_Lower(:,i, t) ];
        derivative_Screw_Lower(:,i, t) = gradient( Screw_Lower(:,i, t) ,t);
        external_loads_Limb_Lower_without_friction(:, i, t) = [m_1*g; cross( r_Lc(:, i, t), m_1*g)];
        external_loads_Limb_Lower(:, i, t) = [m_1*g; cross( r_Lc(:, i, t), m_1*g)] - (miu*v_1_value(:, i, t)+20*sign(v_1_value(:, i, t)))*[e(:, i, t); cross( r_Uc(:, i, t), e(:, i, t))]-10; % Damping force.
        W_Lower(:,i, t) = derivative_Screw_Lower(:,i, t) - external_loads_Limb_Lower(:, i, t);
        W_Lower_without_friction(:,i, t) = derivative_Screw_Lower(:,i, t) - external_loads_Limb_Lower_without_friction(:, i, t);
        damping_only(:, i, t) = miu*v_1_value(:, i, t)*[e(:, i, t); cross( r_Lc(:, i, t), e(:, i, t))];    % Damping force.

        %% the r_P values
        r_P(:, i, t) =  r_A(:, i, t) + l_1*e(:, i, t);

        
        %% The different values of M_i
        M_1(:, :, t) = [e(:, 1, t), tau(:, 1, t), zero, zero, zero;
                        cross( r_C(:, 1, t), e(:, 1, t)), cross( r_C(:, 1, t), tau(:, 1, t)), zero, zero, zero];

        M_2(:, :, t) = [e(:, 2, t), tau(:, 2, t), zero, zero, zero;
                        cross( r_C(:, 2, t), e(:, 2, t)), cross( r_C(:, 2, t), tau(:, 2, t)), zero, zero, zero];

        M_3(:, :, t) = [e(:, 3, t), tau(:, 3, t), zero, zero, zero;
                        cross( r_C(:, 3, t), e(:, 3, t)), cross( r_C(:, 3, t), tau(:, 3, t)), zero, zero, zero];

        M_4(:, :, t) = [e(:, 4, t), tau(:, 4, t), zero, zero, zero;
                        cross( r_C(:, 4, t), e(:, 4, t)), cross( r_C(:, 4, t), tau(:, 4, t)), zero, zero, zero];

        M_5(:, :, t) = [e(:, 5, t), tau(:, 5, t), zero, zero, zero;
                        cross( r_C(:, 5, t), e(:, 5, t)), cross( r_C(:, 5, t), tau(:, 5, t)), zero, zero, zero];

        M_6(:, :, t) = [e(:, 6, t), tau(:, 6, t), zero, zero, zero;
                        cross( r_C(:, 6, t), e(:, 6, t)), cross( r_C(:, 6, t), tau(:, 6, t)), zero, zero, zero];


        %% The different values of U_i
        U_1(:, :, t) = [-e(:, 1, t), -tau(:, 1, t), e(:, 1, t), zero, zero;
                        cross( -r_C(:, 1, t), e(:, 1, t)), cross( -r_C(:, 1, t), tau(:, 1, t)), cross( r_P(:, 1, t), e(:, 1, t)), zero, zero];

        U_2(:, :, t) = [-e(:, 2, t), -tau(:, 2, t), e(:, 2, t), zero, zero;
                        cross( -r_C(:, 2, t), e(:, 2, t)), cross( -r_C(:, 2, t), tau(:, 2, t)), cross( r_P(:, 2, t), e(:, 2, t)), zero, zero];

        U_3(:, :, t) = [-e(:, 3, t), -tau(:, 3, t), e(:, 3, t), zero, zero;
                        cross( -r_C(:, 3, t), e(:, 3, t)), cross( -r_C(:, 3, t), tau(:, 3, t)), cross( r_P(:, 3, t), e(:, 3, t)), zero, zero];

        U_4(:, :, t) = [-e(:, 4, t), -tau(:, 4, t), e(:, 4, t), zero, zero;
                        cross( -r_C(:, 4, t), e(:, 4, t)), cross( -r_C(:, 4, t), tau(:, 4, t)), cross( r_P(:, 4, t), e(:, 4, t)), zero, zero];

        U_5(:, :, t) = [-e(:, 5, t), -tau(:, 5, t), e(:, 5, t), zero, zero;
                        cross( -r_C(:, 5, t), e(:, 5, t)), cross( -r_C(:, 5, t), tau(:, 5, t)), cross( r_P(:, 5, t), e(:, 5, t)), zero, zero];

        U_6(:, :, t) = [-e(:, 6, t), -tau(:, 6, t), e(:, 6, t), zero, zero;
                        cross( -r_C(:, 6, t), e(:, 6, t)), cross(- r_C(:, 6, t), tau(:, 6, t)), cross( r_P(:, 6, t), e(:, 6, t)), zero, zero];



        %% The different values of L_i
         L_1(:, :, t) = [zero zero -e(:, 1, t), e(:, 1, t), tau(:, 1, t);
                        zero zero cross( -r_P(:, 1, t), e(:, 1, t)), cross( r_A(:, 1, t), e(:, 1, t)), cross( r_A(:, 1, t), tau(:, 1, t))];

         L_2(:, :, t) = [zero zero -e(:, 2, t), e(:, 2, t), tau(:, 2, t);
                        zero zero cross( -r_P(:, 2, t), e(:, 2, t)), cross( r_A(:, 2, t), e(:, 2, t)), cross( r_A(:, 2, t), tau(:, 2, t))];

         L_3(:, :, t) = [zero zero -e(:, 3, t), e(:, 3, t), tau(:, 3, t);
                        zero zero cross( -r_P(:, 3, t), e(:, 3, t)), cross( r_A(:, 3, t), e(:, 3, t)), cross( r_A(:, 3, t), tau(:, 3, t))];

         L_4(:, :, t) = [zero zero -e(:, 4, t), e(:, 4, t), tau(:, 4, t);
                        zero zero cross( -r_P(:, 4, t), e(:, 4, t)), cross( r_A(:, 4, t), e(:, 4, t)), cross( r_A(:, 4, t), tau(:, 4, t))];

         L_5(:, :, t) = [zero zero -e(:, 5, t), e(:, 5, t) tau(:, 5, t);
                        zero zero cross( -r_P(:, 5, t), e(:, 5, t)), cross( r_A(:, 5, t), e(:, 5, t)), cross( r_A(:, 5, t), tau(:, 5, t))];

         L_6(:, :, t) = [zero zero -e(:, 6, t), e(:, 6, t), tau(:, 6, t);
                        zero zero cross( -r_P(:, 6, t), e(:, 6, t)), cross( r_A(:, 6, t), e(:, 6, t)), cross( r_A(:, 6, t), tau(:, 6, t))];

         

        %% The matrix S in the new formula
        Matrix_S(:, :, t)  = [M_1(:, :, t) M_2(:, :, t) M_3(:, :, t) M_4(:, :, t) M_5(:, :, t) M_6(:, :, t);
                              U_1(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) U_2(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) zeros(6,5) U_3(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) zeros(6,5) zeros(6,5) U_4(:, :, t) zeros(6,5) zeros(6,5);
                              zeros(6,5)  zeros(6,5)  zeros(6,5)  zeros(6,5) U_5(:, :, t) zeros(6,5);
                              zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) U_6(:, :, t);
                              L_1(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) L_2(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) zeros(6,5) L_3(:, :, t) zeros(6,5) zeros(6,5) zeros(6,5);
                              zeros(6,5) zeros(6,5) zeros(6,5) L_4(:, :, t) zeros(6,5) zeros(6,5);
                              zeros(6,5)  zeros(6,5)  zeros(6,5)  zeros(6,5) L_5(:, :, t) zeros(6,5);
                              zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) zeros(6,5) L_6(:, :, t)]; 


      %% The matrix W in the new formula
         matrix_W(:, :, t) = [W_1(:, :, t)'...
                              W_Upper(:,1,t)'  W_Upper(:,2,t)'  W_Upper(:,3,t)'  W_Upper(:,4,t)'  W_Upper(:,5,t)'  W_Upper(:,6,t)'...
                              W_Lower(:,1,t)'  W_Lower(:,2,t)'  W_Lower(:,3,t)'  W_Lower(:,4,t)'  W_Lower(:,5,t)'  W_Lower(:,6,t)']';

       

          matrix_W_without_friction(:, :, t) = [W_1(:, :, t)'...
                              W_Upper_without_friction(:,1,t)'  W_Upper_without_friction(:,2,t)'  W_Upper_without_friction(:,3,t)'  W_Upper_without_friction(:,4,t)'  W_Upper_without_friction(:,5,t)'  W_Upper_without_friction(:,6,t)'...
                              W_Lower_without_friction(:,1,t)'  W_Lower_without_friction(:,2,t)'  W_Lower_without_friction(:,3,t)'  W_Lower_without_friction(:,4,t)'  W_Lower_without_friction(:,5,t)'  W_Lower_without_friction(:,6,t)']';

        
        %% The different forces calculation in the new formmula
        f(:, :, t)  =  ((Matrix_S(:, :, t)'*Matrix_S(:, :, t))^-1)*(Matrix_S(:, :, t)')*(matrix_W(:, :, t));


        f_without_friction(:, :, t)  =  ((Matrix_S(:, :, t)'*Matrix_S(:, :, t))^-1)*(Matrix_S(:, :, t)')*(matrix_W_without_friction(:, :, t));

    end
end


%% F_S_e
figure;
subplot(2,3,1);
plot(time(1:end), f(1, :) , 'r',time(1:end), f_without_friction(1, :) , 'b--', 'LineWidth', 0.1);
title('Leg 1', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(1 , :) ) )-50  ( max( f(1 , :) ) )+100]);
grid on;

subplot(2,3,2);
plot(time(1:end), f(6, :) , 'r',time(1:end), f_without_friction(6, :) , 'b--', 'LineWidth', 0.1);
title('Leg 2', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(6 , :) ) )-50  ( max( f(6 , :) ) )+100]);
grid on;


subplot(2,3,3);
plot(time(1:end), f(11, :) , 'r',time(1:end), f_without_friction(11, :) , 'b--', 'LineWidth', 0.1);
title('Leg 3', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(11 , :) ) )-50  ( max( f(11 , :) ) )+100]);
grid on;


subplot(2,3,4);
plot(time(1:end),f(16, :), 'r',time(1:end),f_without_friction(16, :), 'b--', 'LineWidth', 0.1);
title('Leg 4', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(16 , :) ) )-100  ( max( f(16 , :) ) )+200]);
grid on;

subplot(2,3,5);
plot(time(1:end),f(21, :), 'r',time(1:end),f_without_friction(21, :), 'b--', 'LineWidth', 0.1);
title('Leg 5', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(21 , :) ) )-100  ( max( f(21 , :) ) )+100]);
grid on;

subplot(2,3,6);
plot(time(1:end),f(26, :), 'r',time(1:end),f_without_friction(26, :), 'b--',  'LineWidth', 0.1);
title('Leg 6', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(26 , :) ) )-100  ( max( f(26 , :) ) )+200]);
grid on;


%% F_S_t
figure;
subplot(2,3,1);
plot(time(1:end), f(2, :) , 'r',time(1:end), f_without_friction(2, :) , 'b--', 'LineWidth', 0.1);
title('Leg 1', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(2 , :) ) )-15  ( max( f(2 , :) ) )+15]);
grid on;

subplot(2,3,2);
plot(time(1:end), f(7, :) , 'r',time(1:end), f_without_friction(7, :) , 'b--',  'LineWidth', 0.1);
title('Leg 2', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(7 , :) ) )-15  ( max( f(7 , :) ) )+15]);
grid on;


subplot(2,3,3);
plot(time(1:end), f(12, :) , 'r',time(1:end), f_without_friction(12, :) , 'b--', 'LineWidth', 0.1);
title('Leg 3', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(12 , :) ) )-15  ( max( f(12 , :) ) )+15]);
grid on;


subplot(2,3,4);
plot(time(1:end),f(17, :), 'r',time(1:end),f_without_friction(17, :), 'b--', 'LineWidth', 0.1);
title('Leg 4', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(17 , :) ) )-15  ( max( f(17 , :) ) )+15]);
grid on;

subplot(2,3,5);
plot(time(1:end),f(22, :), 'r', time(1:end), f_without_friction(22, :), 'b--', 'LineWidth', 0.1);
title('Leg 5', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(22 , :) ) )-15  ( max( f(22 , :) ) )+15]);
grid on;

subplot(2,3,6);
plot(time(1:end),f(27, :), 'r',time(1:end),f_without_friction(27, :), 'b--', 'LineWidth', 0.1);
title('Leg 6', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_S_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(27 , :) ) )-15  ( max( f(27 , :) ) )+15]);
grid on;


%% F_P
figure;
subplot(2,3,1);
plot(time(1:end), f(3, :) , 'r',time(1:end), f_without_friction(3, :) , 'b--', 'LineWidth', 0.1);
title('Leg 1', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(3 , :) ) )-100  ( max( f(3 , :) ) )+100]);
grid on;

subplot(2,3,2);
plot(time(1:end), f(8, :) , 'r', time(1:end), f_without_friction(8, :) , 'b--','LineWidth', 0.1);
title('Leg 2', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(8 , :) ) )-100  ( max( f(8 , :) ) )+100]);
grid on;


subplot(2,3,3);
plot(time(1:end), f(13, :) , 'r',time(1:end), f_without_friction(13, :) , 'b--', 'LineWidth', 0.1);
title('Leg 3', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(13 , :) ) )-100  ( max( f(13 , :) ) )+100]);
grid on;


subplot(2,3,4);
plot(time(1:end),f(18, :), 'r',time(1:end),f_without_friction(18, :), 'b--', 'LineWidth', 0.1);
title('Leg 4', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(18, :) ) )-100  ( max( f(18 , :) ) )+200]);
grid on;

subplot(2,3,5);
plot(time(1:end),f(23, :), 'r',time(1:end),f_without_friction(23, :), 'b--', 'LineWidth', 0.1);
title('Leg 5', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(23 , :) ) )-100  ( max( f(23 , :) ) )+200]);
grid on;

subplot(2,3,6);
plot(time(1:end),f(28, :), 'r',time(1:end), f_without_friction(28, :), 'b--', 'LineWidth', 0.1);
title('Leg 6', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_P (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
ylim([( min( f(28, :) ) )-100  ( max( f(28 , :) ) )+200]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
grid on;



%% F_U_e
figure;
subplot(2,3,1);
plot(time(1:end), f(4, :) , 'r',time(1:end), f_without_friction(4, :) , 'b--', 'LineWidth', 0.1);
title('Leg 1', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(4 , :) ) )-100  ( max( f(4 , :) ) )+100]);
grid on;
 
subplot(2,3,2);
plot(time(1:end), f(9, :) , 'r',time(1:end), f_without_friction(9, :) , 'b--', 'LineWidth', 0.1);
title('Leg 2', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(9 , :) ) )-100  ( max( f(9 , :) ) )+100]);
grid on;


subplot(2,3,3);
plot(time(1:end), f(14, :) , 'r', time(1:end), f_without_friction(14, :) , 'b--','LineWidth', 0.1);
title('Leg 3', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(14 , :) ) )-100  ( max( f(14 , :) ) )+100]);
grid on;


subplot(2,3,4);
plot(time(1:end),f(19, :), 'r',time(1:end),f_without_friction(19, :), 'b--', 'LineWidth', 0.1);
title('Leg 4', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(19 , :) ) )-100  ( max( f(19 , :) ) )+200]);
grid on;

subplot(2,3,5);
plot(time(1:end),f(24, :), 'r',time(1:end),f_without_friction(24, :), 'b--',  'LineWidth', 0.1);
title('Leg 5', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(24 , :) ) )-100  ( max( f(24 , :) ) )+100]);
grid on;

subplot(2,3,6);
plot(time(1:end),f(29, :), 'r',time(1:end),f_without_friction(29, :), 'b--',  'LineWidth', 0.1);
title('Leg 6', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_e (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(29 , :) ) )-100  ( max( f(29 , :) ) )+200]);
grid on;


%% F_U_t
figure;
subplot(2,3,1);
plot(time(1:end), f(5, :) , 'r',time(1:end), f_without_friction(5, :) , 'b--', 'LineWidth', 0.1);
title('Leg 1', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(5 , :) ) )-15  ( max( f(5 , :) ) )+15]);
grid on;

subplot(2,3,2);
plot(time(1:end), f(10, :) , 'r',time(1:end), f_without_friction(10, :) , 'b--', 'LineWidth', 0.1);
title('Leg 2', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(10 , :) ) )-15  ( max( f(10 , :) ) )+15]);
grid on;


subplot(2,3,3);
plot(time(1:end), f(15, :) , 'r',time(1:end), f_without_friction(15, :) , 'b--',  'LineWidth', 0.1);
title('Leg 3', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(15 , :) ) )-15  ( max( f(15 , :) ) )+15]);
grid on;


subplot(2,3,4);
plot(time(1:end),f(20, :), 'r',time(1:end),f_without_friction(20, :), 'b--',  'LineWidth', 0.1);
title('Leg 4', 12, 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(20 , :) ) )-15  ( max( f(20 , :) ) )+15]);
grid on;

subplot(2,3,5);
plot(time(1:end),f(25, :), 'r',time(1:end),f_without_friction(25, :), 'b--', 'LineWidth', 0.1);
title('Leg 5', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(25 , :) ) )-15  ( max( f(25 , :) ) )+15]);
grid on;

subplot(2,3,6);
plot(time(1:end),f(30, :), 'r',time(1:end),f_without_friction(30, :), 'b--', 'LineWidth', 0.1);
title('Leg 6', 'FontSize', 12,'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('F_U_t (N)', 'FontSize', 12,'FontName', 'Times New Roman');
xlim([0 iteration_time]);
legend('With Friction','Without Friction','FontName', 'Times New Roman');
ylim([( min( f(30 , :) ) )-15  ( max( f(30 , :) ) )+15]);
grid on;


%% Comparison 'F_U_e', 'F_P', 'F_S_e'
figure;
for i = 1:6
    subplot(2,3,i);
    plot(time(1:end), f(4 + (i-1)*5, :), 'r', 'LineWidth', 0.1); 
    hold on;
    plot(time(1:end), f(3 + (i-1)*5, :), 'b-.', 'LineWidth', 0.1); 
    plot(time(1:end), f(1 + (i-1)*5, :), 'k--', 'LineWidth', 0.1);
%     plot(time(1:end), f_without_friction(4 + (i-1)*5, :), 'b--', 'LineWidth', 1.5);  
%     plot(time(1:end), f_without_friction(3 + (i-1)*5, :), 'y--', 'LineWidth', 1.5); 
%     plot(time(1:end), f_without_friction(1 + (i-1)*5, :), 'c--', 'LineWidth', 1.5);
    hold off;
    title(['Leg ', num2str(i)], 'FontSize', 12,'FontName', 'Times New Roman');
    xlabel('Time (s)', 'FontSize', 12,'FontName', 'Times New Roman');
    ylabel('Force (N)', 'FontSize', 12,'FontName', 'Times New Roman');
    xlim([0 iteration_time]);
    grid on;
    legend('F_U_e', 'F_P', 'F_S_e', 'Orientation', 'horizontal','Location', 'north', 'Box', 'off');
    ylim([( min( f(3 + (i-1)*5, :) ) )-50  ( max( f(3 + (i-1)*5, :) ) )+200]);

end


%% Plot motion Trajectory of Platform
figure;
plot3(r_Om(1, :), r_Om(2, :), r_Om(3, :), 'LineWidth', 1.5);
xlabel('{\it x}(m)', 'FontSize', 12,'FontName', 'Times New Roman');
ylabel('{\it y}(m)', 'FontSize', 12,'FontName', 'Times New Roman');
zlabel('{\it z}(m)', 'FontSize', 12,'FontName', 'Times New Roman');
grid on;
title('Motion Trajectory of Platform', 'FontSize', 12,'FontName', 'Times New Roman');
hold off;





%% Create a figure for kinematics animation
kinematicsFig = figure('Name', 'Kinematics Animation', 'NumberTitle', 'off', ...
                      'IntegerHandle', 'off', 'Tag', 'KinematicsAnimation');
% Set up the animation axes
ax = axes('Parent', kinematicsFig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
view(ax, 3);
title(ax, 'Kinematics of Moving Platform');

% Animation loop
for t = 1:100:n 
    % Clear only axes
    cla(ax);
    % Form closed shapes for platforms
    base_points = [r_A(:, :, t), r_A(:, 1, t)];
    moving_platform_points = [r_Om(:, t) + r_OmC(:, :, t), r_Om(:, t) + r_OmC(:, 1, t)];
    
    % Plot platforms
    plot3(ax, base_points(1, :), base_points(2, :), base_points(3, :), ...
          'ro-', 'LineWidth', 0.1, 'MarkerSize', 8);
    plot3(ax, moving_platform_points(1, :), moving_platform_points(2, :), ...
          moving_platform_points(3, :), 'bo-', 'LineWidth', 0.1, 'MarkerSize', 8);
    
    % Plot connections
    for i = 1:6
        plot3(ax, [r_A(1, i, t), moving_platform_points(1, i)], ...
              [r_A(2, i, t), moving_platform_points(2, i)], ...
              [r_A(3, i, t), moving_platform_points(3, i)], ...
              'k-', 'LineWidth', 0.1);
    end
    drawnow;
    pause(0.01);
end





%%Another waw to write kinematics of Moving Platform - Animation
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% view(3);
% title('Kinematics of Moving Platform');
% 
% 
% for t = 1:100:n  
%     cla;
%     %%Add the closing point to form closed shapes for both platforms
%     base_points = [r_A(:, :, t), r_A(:, 1, t)];                                                        %Close base platform
%     moving_platform_points = [r_Om(:, t) + r_OmC(:, :, t), r_Om(:, t) + r_OmC(:, 1, t)];               %Close moving platform
%     
%     %%Plot the base platform as a closed figure
%     plot3(base_points(1, :), base_points(2, :), base_points(3, :), 'ro-', 'LineWidth', 0.1, 'MarkerSize', 8);
% 
%     %%Plot the moving platform as a closed figure
%     plot3(moving_platform_points(1, :), moving_platform_points(2, :), moving_platform_points(3, :), ...
%           'bo-', 'LineWidth', 0.1, 'MarkerSize', 8);
% 
%     for i = 1:6
%         plot3([r_A(1, i, t), moving_platform_points(1, i)], ...
%               [r_A(2, i, t), moving_platform_points(2, i)], ...
%               [r_A(3, i, t), moving_platform_points(3, i)], ...
%               'k-', 'LineWidth', 0.1); 
%     end
%     pause(0.01);
% end
% hold off;
