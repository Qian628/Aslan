classdef KinematicsBicycleModelWithDelay
    properties
        wheelbase
        steer_lim
        steer_tau
        dim_x = 3;
        dim_u = 1;
        dim_y = 2;
    end
    
    methods
        function obj = KinematicsBicycleModelWithDelay(wheelbase, steer_lim, steer_tau)
            obj.wheelbase = wheelbase;
            obj.steer_lim = steer_lim;
            obj.steer_tau = steer_tau;
        end
        
        function [Ad, Bd, Cd, Wd] = calculateDiscreteMatrix(obj, velocity, curvature, dt)
            sign = @(x) (x > 0) - (x < 0);
            vel = max(velocity, 0.01);
            
            % Linearize delta around delta_r (reference delta)
            delta_r = atan(obj.wheelbase * curvature);
            if abs(delta_r) >= obj.steer_lim
                delta_r = obj.steer_lim * sign(delta_r);
            end
            cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

            Ad = [0, velocity, 0;
                  0, 0, velocity / obj.wheelbase * cos_delta_r_squared_inv;
                  0, 0, -1.0 / obj.steer_tau];
            I = eye(obj.dim_x, obj.dim_x);
            
            Ad = I + Ad * dt;
            % Ad_inverse = inv(I - dt * 0.5 * Ad);
            % Ad = Ad_inverse * (I + dt * 0.5 * Ad); % bilinear discretization
            
            Bd = [0; 0; 1.0 / obj.steer_tau];
            Bd = Bd * dt;
            % Bd = (Ad_inverse * dt) * Bd;
            
            Cd = [1, 0, 0;
                  0, 1, 0];
              
            Wd = [0;
                  -velocity * curvature + velocity / obj.wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv);
                  0];
            Wd = Wd * dt;
            % Wd = (Ad_inverse * dt) * Wd;
        end
        
        function Uref = calculateReferenceInput(obj, curvature)
            Uref = atan(obj.wheelbase * curvature);
        end
    end
end
