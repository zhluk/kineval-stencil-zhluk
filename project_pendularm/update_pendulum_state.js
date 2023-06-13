function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = pendulum.angle_previous + pendulum.angle_dot * dt;
        pendulum.angle_dot = pendulum.angle_dot + pendulum.angle_dot_dot * dt;
        

    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
        var temp = pendulum.angle;
        var a = pendulum_acceleration(pendulum, gravity)
        pendulum.angle = 2*pendulum.angle - pendulum.angle_previous + a * dt**2;
    
        pendulum.angle_dot = (pendulum.angle - pendulum.angle_previous)/2/dt;

        pendulum.angle_previous = temp;

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
        pendulum.angle = pendulum.angle + pendulum.angle_dot*dt + 1/2*pendulum.angle_dot_dot * dt**2;
        var theta_ddot_dt = pendulum_acceleration(pendulum, gravity);
        pendulum.angle_dot = pendulum.angle_dot + (pendulum.angle_dot_dot + theta_ddot_dt)/2*dt;
        pendulum.angle_previous = pendulum.angle;
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
        var k_x1 = pendulum.angle;
        var k_v1 = pendulum.angle_dot;
        var k_x2 = k_x1 + (1/2*k_v1*dt);
        var k_v2 = k_v1 + (1/2*acceleration(pendulum, k_x1, gravity)*dt);
        var k_x3 = k_x1 + (1/2*k_v2*dt);
        var k_v3 = k_v1 + (1/2*acceleration(pendulum, k_x2, gravity)*dt);
        var k_x4 = k_x1 + (1*k_v3*dt);
        var k_v4 = k_v1 + (1*acceleration(pendulum, k_x3, gravity)*dt);

        pendulum.angle = pendulum.angle + dt*(k_v1 + 2*k_v2 + 2*k_v3 + k_v4)/6;

        var a_x1 = acceleration(pendulum, k_x1, gravity);
        var a_x2 = acceleration(pendulum, k_x2, gravity);
        var a_x3 = acceleration(pendulum, k_x3, gravity);
        var a_x4 = acceleration(pendulum, k_x4, gravity);
        pendulum.angle_dot = pendulum.angle_dot + dt*(a_x1 + 2*a_x2 + 2*a_x3 + a_x4)/6;
        pendulum.angle_previous = pendulum.angle;
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = wrap(pendulum.angle);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function acceleration(pendulum, k_x, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    
    var l = pendulum.length;
    var tau = pendulum.control;
    var m = pendulum.mass;
    
    theta_ddot = - gravity / l * Math.sin(k_x) + tau / m / l**2;

    return theta_ddot;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    
    var l = pendulum.length;
    var theta = pendulum.angle;
    var tau = pendulum.control;
    var m = pendulum.mass;
    
    theta_ddot = - gravity / l * Math.sin(theta) + tau / m / l**2;
     
    return theta_ddot;
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    pendulum.angle_previous = pendulum.angle;
    pendulum.angle = pendulum.angle + dt * pendulum.angle_dot + dt**2 * 1/2 * pendulum_acceleration(pendulum, gravity);
    
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:6800, kd:285, ki:2750}
    //pendulum.servo = {kp:2200, kd:200, ki:500}; // test with rk4
    //pendulum.servo = {kp:1000, kd:90, ki:160}; // test with verlet
    //pendulum.servo = {kp:1300, kd:200, ki:350}; // test with eular
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    var e_t = pendulum.desired - pendulum.angle;

    var k_p = pendulum.servo.kp;
    var k_i = pendulum.servo.ki;
    var k_d = pendulum.servo.kd;

    accumulated_error = accumulated_error + e_t;

    pendulum.control = (k_p * e_t + k_i * accumulated_error - k_d * pendulum.angle_dot);    
    return [pendulum, accumulated_error];
}

function wrap(angle){
    return (angle+Math.PI/180)%(2*Math.PI);
}