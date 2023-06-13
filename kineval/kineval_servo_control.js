
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) {
        song.pause();
        return;
        
    }

    if(song.paused){
        song.load();
        song.play();
    }

    // STENCIL: implement FSM to cycle through dance pose setpoints
    // var error = 0;
    // // check if dance pose is achieved
    // for (x in robot.joints) {
    //     var ref = kineval.params.setpoint_target[x];
    //     var angle_cur = robot.joints[x].angle;

    //     error += (ref - angle_cur)**2;
    // }
    // error =  Math.sqrt(error);
    // //console.log(error);




    count_loop += 1;
    if (kineval.params.dance_pose_index < kineval.params.dance_sequence_index.length){
        var setpoint_index = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
        kineval.params.setpoint_target = kineval.setpoints[setpoint_index];
        if(count_loop>23){
            kineval.params.dance_pose_index = kineval.params.dance_pose_index+1;
            //console.log(kineval.params.dance_pose_index)
            count_loop = 0;
        }
        
    }

    if (kineval.params.dance_pose_index == kineval.params.dance_sequence_index.length){
        kineval.params.dance_pose_index = 0;
    }

}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints) {
        var ref = kineval.params.setpoint_target[x];
        var angle_cur = robot.joints[x].angle;
        var k_p = robot.joints[x].servo.p_gain;

        var error = ref - angle_cur;

        robot.joints[x].control = k_p * error;
    }

}


