function [traslacion,rotacion] = cinematica_dir(robot,articulaciones,t)
%CINEMATICA_DIR Summary of this function goes here
%   Detailed explanation goes here
q = articulaciones(:,:,1);
dq = articulaciones(:,:,2);
ddq = articulaciones(:,:,3);

pos = zeros(3,length(t));
ori = pos;
v_l = pos;
v_r = pos;
a_l = pos;
a_r = pos;
R = zeros(3,3,length(t));
Jv = zeros(3,robot.NGDL,length(t));
dJv = Jv;
Jw = Jv;
dJw = Jw;

for k = 1:length(t)
%     Actualiza la configuración del robot con los valores articulares 
    robot = actualizar_robot_completo(robot, q(:,k));
%     Extraer la posición y orientación del efector final
    pos(:,k) = robot.T(1:3,4,end);
    R(:,:,k) = robot.T(1:3,1:3,end);
%     Obtener la orientación en ángulos de Euler a partir de la matriz R del efector final
    ori(:,k) = rotMat2euler(R(:, :, k),secuencia);
            Calcular el Jacobiano geométrico a partir de la transformación global
    [Jv(:,:,k),Jw(:,:,k)] = jac_geometrico_completo(robot);
%     Calcular la velocidad lineal y angular
            
    v_l(:,k)  = Jv(:,:,k)*dq(:,k);
    v_r(:,k) = Jw(:,:,k)*dq(:,k);
%     Calcular la derivada temporal del Jacobiano usando diferencias finitas
    if k > 1
        dt = t(k) - t(k-1);
        dJv(:,:,k) = (Jv(:,:,k) - Jv(:,:,k-1)) / dt;
        dJw(:,:,k) = (Jw(:,:,k) - Jw(:,:,k-1)) / dt;
    end
%     Calcular la aceleración lineal y angular
            
    a_l(:,k) = Jv(:,:,k)*ddq(:,k) + dJv(:,:,k)*dq(:,k);
    a_r(:,k) = Jw(:,:,k)*ddq(:,k) + dJw(:,:,k)*dq(:,k);
end

traslacion(:,:,1) = pos;
traslacion(:,:,2) = v_l;
traslacion(:,:,3) = a_l;
rotacion(:,:,1) = ori;
rotacion(:,:,2) = v_r;
rotacion(:,:,3) = a_r;