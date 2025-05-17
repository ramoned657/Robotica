function [pos,v_l,a_l,ori,v_r,a_r] = cinematica_dir(robot,q,dq,ddq,t,secuencia)
% CINEMATICA_DIR Calcula la cinemática directa y diferencial del efector final.
%
%   Calcula la posición, orientación (ángulos de Euler), velocidad lineal,
%   velocidad angular, aceleración lineal y aceleración angular del
%   efector final del robot a lo largo de una trayectoria temporal.
%
%   Entradas:
%       robot      - Estructura del robot (con NGDL, etc., y funciones aux.)
%       q          - Matriz de posiciones articulares (NGDL x M)
%       dq         - Matriz de velocidades articulares (NGDL x M)
%       ddq        - Matriz de aceleraciones articulares (NGDL x M)
%       t          - Vector de tiempo (1 x M)
%       secuencia  - String con la secuencia de ángulos de Euler (ej. 'ZYX')
%
%   Salidas:
%       pos        - Posición cartesiana (3 x M)
%       v_l        - Velocidad lineal (3 x M)
%       a_l        - Aceleración lineal (3 x M)
%       ori        - Orientación en ángulos de Euler (3 x M)
%       v_r        - Velocidad angular (3 x M)
%       a_r        - Aceleración angular (3 x M)

%Inicializar variables
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
dt = 0.05;

for k = 1:length(t)
     robot = actualizar_robot(robot, q(:,k));
     pos(:,k) = robot.T(1:3,4,end);
     R(:,:,k) = robot.T(1:3,1:3,end);
     ori(:,k) = rotMat2euler(R(:, :, k),secuencia);
     [Jv(:,:,k),Jw(:,:,k)] = jac_geometrico(robot);
      v_l(:,k)  = Jv(:,:,k) * dq(:,k); 
      v_r(:,k) = Jw(:,:,k) * dq(:,k);  
       if k > 1
        dJv(:,:,k) = (Jv(:,:,k) - Jv(:,:,k-1)) / dt;
        dJw(:,:,k) = (Jw(:,:,k) - Jw(:,:,k-1)) / dt;
       end
   a_l(:,k)  = Jv(:,:,k) * ddq(:,k) + dJv(:,:,k) * dq(:,k); 
   a_r(:,k) = Jw(:,:,k) * ddq(:,k) + dJw(:,:,k) * dq(:,k);
end