classdef DynamicsBicycleModel
    properties
        dim_x = 4
        dim_u = 1
        dim_y = 2
        wheelbase
        mass
        lf
        lr
        iz
        cf
        cr
    end
    
    methods
        function obj = DynamicsBicycleModel(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr)
            obj.wheelbase = wheelbase;
            mass_front = mass_fl + mass_fr;
            mass_rear = mass_rl + mass_rr;
            obj.mass = mass_front + mass_rear;
            obj.lf = obj.wheelbase * (1.0 - mass_front / obj.mass);
            obj.lr = obj.wheelbase * (1.0 - mass_rear / obj.mass);
            obj.iz = obj.lf^2 * mass_front + obj.lr^2 * mass_rear;
            obj.cf = cf;
            obj.cr = cr;
        end

        function [Ad, Bd, Cd, Wd] = calculateDiscreteMatrix(obj, velocity, curvature, dt)
            vel = max(velocity, 0.01);

            Ad = zeros(obj.dim_x, obj.dim_x);
            Ad(1, 2) = 1.0;
            Ad(2, 2) = -(obj.cf + obj.cr) / (obj.mass * vel);
            Ad(2, 3) = (obj.cf + obj.cr) / obj.mass;
            Ad(2, 4) = (obj.lr * obj.cr - obj.lf * obj.cf) / (obj.mass * vel);
            Ad(3, 4) = 1.0;
            Ad(4, 2) = (obj.lr * obj.cr - obj.lf * obj.cf) / (obj.iz * vel);
            Ad(4, 3) = (obj.lf * obj.cf - obj.lr * obj.cr) / obj.iz;
            Ad(4, 4) = -(obj.lf^2 * obj.cf + obj.lr^2 * obj.cr) / (obj.iz * vel);

            I = eye(obj.dim_x, obj.dim_x);
            Ad_inverse = inv(I - dt * 0.5 * Ad);

            Ad = Ad_inverse * (I + dt * 0.5 * Ad); % bilinear discretization

            Bd = zeros(obj.dim_x, obj.dim_u);
            Bd(2, 1) = obj.cf / obj.mass;
            Bd(4, 1) = obj.lf * obj.cf / obj.iz;

            Wd = zeros(obj.dim_x, 1);
            Wd(2, 1) = (obj.lr * obj.cr - obj.lf * obj.cf) / (obj.mass * vel) - vel;
            Wd(4, 1) = -(obj.lf^2 * obj.cf + obj.lr^2 * obj.cr) / (obj.iz * vel);

            Bd = (Ad_inverse * dt) * Bd;
            Wd = (Ad_inverse * dt * curvature * vel) * Wd;

            Cd = zeros(obj.dim_y, obj.dim_x);
            Cd(1, 1) = 1.0;
            Cd(2, 3) = 1.0;
        end
        function Uref = calculateReferenceInput(obj, velocity, curvature)
            vel = max(velocity, 0.01);
            Kv = obj.lr * obj.mass / (2 * obj.cf * obj.wheelbase) - obj.lf * obj.mass / (2 * obj.cr * obj.wheelbase);
            Uref = obj.wheelbase * curvature + Kv * vel^2 * curvature;
        end
    end
end
