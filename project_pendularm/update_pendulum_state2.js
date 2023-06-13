function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment

    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
        var k_x1 = pendulum.angle;
        var k_v1 = pendulum.angle_dot;

        var a_1 = acceleration(pendulum, k_x1, k_v1, gravity);

        var k_x2 = [k_x1[0] + (1/2*k_v1[0]*dt),k_x1[1] + (1/2*k_v1[1]*dt)];
        var k_v2 = [k_v1[0] + 1/2*a_1[0]*dt,k_v1[1] + 1/2*a_1[1]*dt];

        var a_2 = acceleration(pendulum, k_x2, k_v2, gravity);

        var k_x3 = [k_x1[0] + (1/2*k_v2[0]*dt),k_x1[1] + (1/2*k_v2[1]*dt)];
        var k_v3 = [k_v1[0] + 1/2*a_2[0]*dt,k_v1[1] + 1/2*a_2[1]*dt];

        var a_3 = acceleration(pendulum, k_x3, k_v3, gravity);

        var k_x4 = [k_x1[0] + (1*k_v3[0]*dt),k_x1[1] + (1*k_v3[1]*dt)];
        var k_v4 = [k_v1[0] + 1*a_3[0]*dt,k_v1[1] + 1*a_3[1]*dt];

        var a_4 = acceleration(pendulum, k_x4, k_v4, gravity);

        pendulum.angle[0] = pendulum.angle[0] + dt*(k_v1[0] + 2*k_v2[0] + 2*k_v3[0] + k_v4[0])/6;
        pendulum.angle[1] = pendulum.angle[1] + dt*(k_v1[1] + 2*k_v2[1] + 2*k_v3[1] + k_v4[1])/6;

        pendulum.angle_dot[0] = pendulum.angle_dot[0] + dt*(a_1[0] + 2*a_2[0] + 2*a_3[0] + a_4[0])/6;
        pendulum.angle_dot[1] = pendulum.angle_dot[1] + dt*(a_1[1] + 2*a_2[1] + 2*a_3[1] + a_4[1])/6;
        pendulum.angle_previous = pendulum.angle;

    } 
    else {
        pendulum.angle_previous[0] = pendulum.angle[0];
        pendulum.angle[0] = (pendulum.angle[0]+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[0] = (pendulum.angle[0]-pendulum.angle_previous[0])/dt;
        pendulum.angle_previous[1] = pendulum.angle[1];
        pendulum.angle[1] = (pendulum.angle[1]-Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[1] = (pendulum.angle[1]-pendulum.angle_previous[1])/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function acceleration(pendulum, k_x, k_v, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    var m1 = pendulum.mass[0];
    var m2 = pendulum.mass[1];
    var l1 = pendulum.length[0];
    var l2 = pendulum.length[1];

    var theta1 = k_x[0];
    var theta2 = k_x[1];
    var theta1d = k_v[0];
    var theta2d = k_v[1];

    var tau1 = pendulum.control[0];
    var tau2 = pendulum.control[1];

    var a =  (m1+m2)*l1**2;
    var b =  m2*l1*l2*Math.cos(theta2-theta1);
    var c =  tau1 + m2*l1*l2*(theta2d**2)*Math.sin(theta2-theta1) - (m1+m2)*gravity*l1*Math.sin(theta1);
    var d =  m2*(l2**2);
    var e =  tau2 - m2*l1*l2*(theta1d**2)*Math.sin(theta2-theta1)-m2*gravity*l2*Math.sin(theta2);
    
    var theta_ddot = [0,0];
    theta_ddot[0] = (d*c-b*e)/(d*a-b**2);
    theta_ddot[1] = (a*e-b*c)/(d*a-b**2);

    return theta_ddot;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion
    var m1 = pendulum.mass[0];
    var m2 = pendulum.mass[1];
    var l1 = pendulum.length[0];
    var l2 = pendulum.length[1];

    var theta1 = pendulum.angle[0];
    var theta2 = pendulum.angle[1];
    var theta1d = pendulum.angle_dot[0];
    var theta2d = pendulum.angle_dot[1];

    var tau1 = pendulum.control[0];
    var tau2 = pendulum.control[1];

    var a =  (m1+m2)*l1**2;
    var b =  m2*l1*l2*Math.cos(theta2-theta1);
    var c =  tau1 + m2*l1*l2*(theta2d**2)*Math.sin(theta2-theta1) - (m1+m2)*gravity*l1*Math.sin(theta1);
    var d =  m2*(l2**2);
    var e =  tau2 - m2*l1*l2*(theta1d**2)*Math.sin(theta2-theta1) - m2*gravity*l2*Math.sin(theta2);
    
    var theta_ddot = [0,0];
    theta_ddot[0] = (d*c-b*e)/(d*a-b**2);
    theta_ddot[1] = (a*e-b*c)/(d*a-b**2);

    return theta_ddot;
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:[1200,1200], kd:[120,120], ki:[160,160]}; // test with rk4
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    console.log(accumulated_error) 
    var e_t = [0,0];
    e_t[0] = pendulum.desired[0] - pendulum.angle[0];
    e_t[1] = pendulum.desired[1] - pendulum.angle[1];

    var k_p = pendulum.servo.kp;
    var k_i = pendulum.servo.ki;
    var k_d = pendulum.servo.kd;

    var e_accu = [0,0];
    e_accu[0] = accumulated_error[0] + e_t[0];
    e_accu[1] = accumulated_error[1] + e_t[1];
   

    pendulum.control[0] = k_p[0] * e_t[0] + k_i[0] * e_accu[0] - k_d[0] * pendulum.angle_dot[0];
    pendulum.control[1] = k_p[1] * e_t[1] + k_i[1] * e_accu[1] - k_d[1] * pendulum.angle_dot[1];
    

    return [pendulum, e_accu];
}

function wrap(angle){
    return (angle+Math.PI/180)%(2*Math.PI);
}
