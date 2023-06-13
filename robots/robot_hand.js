//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "hand";

robot.partner_name = "None";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

// specify and create data objects for the links of the robot
robot.links = { "base": {},
               "upper_arm": {},
                "forearm": {},
                    "palm": {},
                   "thumb": {},
            "index_finger": {},
           "middle_finger": {},
             "ring_finger": {},
           "little_finger": {}
        };

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.shoulder = {parent:"base", child:"upper_arm"};
robot.joints.shoulder.origin = {xyz: [0,1.5,0], rpy:[0,0,0]};
robot.joints.shoulder.axis = [1.0,0.0,0];  // simpler axis

robot.joints.elbow = {parent:"upper_arm", child:"forearm"};
robot.joints.elbow.origin = {xyz: [-0.9,0,0], rpy:[0,0,Math.PI/2]};
robot.joints.elbow.axis = [0,1,0];

robot.joints.wrist = {parent:"forearm", child:"palm"};
robot.joints.wrist.origin = {xyz: [1,0,0], rpy:[-Math.PI/2,0,-Math.PI/2]};
robot.joints.wrist.axis = [0,1,0];

robot.joints.thumb_joint = {parent:"palm", child:"thumb"};
robot.joints.thumb_joint.origin = {xyz: [0.3,0,0.3], rpy:[Math.PI/5,0,-Math.PI/2]};
robot.joints.thumb_joint.axis = [0,0,1];

robot.joints.index_joint = {parent:"palm", child:"index_finger"};
robot.joints.index_joint.origin = {xyz: [0.15,0,0.4], rpy:[Math.PI/2.2,0,-Math.PI/2]};
robot.joints.index_joint.axis = [0,0,1];

robot.joints.middle_joint = {parent:"palm", child:"middle_finger"};
robot.joints.middle_joint.origin = {xyz: [0,0,0.5], rpy:[Math.PI/2,0,-Math.PI/2]};
robot.joints.middle_joint.axis =  [0,0,1];

robot.joints.ring_joint = {parent:"palm", child:"ring_finger"};
robot.joints.ring_joint.origin = {xyz: [-0.15,0,0.4], rpy:[Math.PI/1.8,0,-Math.PI/2]};
robot.joints.ring_joint.axis = [0,0,1];

robot.joints.little_joint = {parent:"palm", child:"little_finger"};
robot.joints.little_joint.origin = {xyz: [-0.23,0,0.3], rpy:[Math.PI/1.6,0,-Math.PI/2]};
robot.joints.little_joint.axis = [0,0,1];


// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "middle_joint";
robot.endeffector.position = [[0.5],[0],[0],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 2, 1);
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0.5, 1, 0) );

links_geom["upper_arm"] = new THREE.CubeGeometry( 1, 0.2, 0.2 );
links_geom["upper_arm"].applyMatrix( new THREE.Matrix4().makeTranslation(-1/2, 0, 0) );

links_geom["forearm"] = new THREE.CubeGeometry(1, 0.2, 0.2 );
links_geom["forearm"].applyMatrix( new THREE.Matrix4().makeTranslation(1/2, 0, 0) );

links_geom["palm"] = new THREE.CylinderGeometry(0.25, 0.25, 0.2 );
links_geom["palm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.2) );

links_geom["thumb"] = new THREE.CylinderGeometry( 0.03, 0.05, 0.4 );
links_geom["thumb"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["index_finger"] = new THREE.CylinderGeometry( 0.03, 0.05, 0.5 );
links_geom["index_finger"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["middle_finger"] = new THREE.CylinderGeometry( 0.03, 0.05, 0.5 );
links_geom["middle_finger"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["ring_finger"] = new THREE.CylinderGeometry( 0.03, 0.05, 0.5 );
links_geom["ring_finger"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["little_finger"] = new THREE.CylinderGeometry(  0.03, 0.05, 0.5 );
links_geom["little_finger"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );