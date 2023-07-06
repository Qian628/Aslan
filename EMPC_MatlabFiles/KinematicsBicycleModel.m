classdef KinematicsBicycleModel
    properties
        wheelbase
        steer_lim
        dim_x = 2;
        dim_u = 1;
        dim_y = 2;
    end
    
    methods
        function obj = KinematicsBicycleModel(wheelbase, steer_lim)
            obj.wheelbase = wheelbase;
            obj.steer_lim = steer_lim;
        end
        
        function [Ad, Bd, Cd, Wd] = calculateDiscreteMatrix(obj, velocity, curvature, dt)
            sign = @(x) (x > 0) - (x < 0);

            % Linearize delta around delta_r (reference delta)
            delta_r = atan(obj.wheelbase * curvature);
            if abs(delta_r) >= obj.steer_lim
                delta_r = obj.steer_lim * sign(delta_r);
            end
            cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

            Ad = [0, velocity;
                  0, 0];
            I = eye(obj.dim_x, obj.dim_x);
            Ad = I + Ad * dt;
            
            Bd = [0; velocity / obj.wheelbase * cos_delta_r_squared_inv];
            Bd = Bd * dt;
            
            Cd = [1, 0;
                  0, 1];
              
            Wd = [0;
                  -velocity / obj.wheelbase * delta_r * cos_delta_r_squared_inv];
            Wd = Wd * dt;
        end
        
        function Uref = calculateReferenceInput(obj, curvature)
            Uref = atan(obj.wheelbase * curvature);
        end
    end
end
