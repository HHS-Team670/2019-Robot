var date = new Date();
document.getElementById('big-warning').style.display = "none";
document.getElementById('auton-chooser').style.display = "none";

// initial camera settings
var cameraMode = "double";
var driveReversed = false;
var allKeysPressed = new Array();

// highlight the front camera since the robot will be facing forward at the start of the match
document.getElementById('front-indicator').style.display = "inline";
document.getElementById('back-indicator').style.display = "none";

// sets default positions for robot diagram
var angle = 0;
var armLength = 110;
document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
document.getElementById('claw').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
document.getElementById('intake').style = "transform: rotate(" + 0 + "deg)";
document.getElementById('arm-extension').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";

// set default values for arm destination drawing
document.getElementById('arm-destination').style = "transform: rotate(" + angle + "deg)";
document.getElementById('claw-destination').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
document.getElementById('arm-extension-destination').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";

// hide everything to do with single camera view
document.getElementById('camera-large').style.display = "none";
document.getElementById('v-crosshairs-large').style.display = "none";

// switches between single and double camera views
NetworkTables.addKeyListener('/SmartDashboard/driver-camera-mode', (key, value) => {
  if (value === "single") {
    cameraMode = "single";
    
    // hide everything to do with double camera view
    document.getElementById('camera1').style.display = "none";
    document.getElementById('camera2').style.display = "none";
    document.getElementById('back-indicator').style.display = "none";
    document.getElementById('front-indicator').style.display = "none";
    document.getElementById('v-crosshairs-left').style.display = "none";
    document.getElementById('v-crosshairs-right').style.display = "none";

    // show everything to do with single camera view
    document.getElementById('camera-large').style.display = "inline";
    document.getElementById('v-crosshairs-large').style.display = "inline";
    // show appropriate stream on the single camera
    if (driveReversed) {
      document.getElementById('camera-stream-large').src = "http://10.6.70.26:443/?action=stream";
      document.getElementById('camera-stream-large').onerror = "this.src='http://10.6.70.26:443/?action=stream'";
    } else {
      document.getElementById('camera-stream-large').src = "http://10.6.70.26:80/?action=stream";
      document.getElementById('camera-stream-large').onerror = "this.src='http://10.6.70.26:80/?action=stream'";
    }
  } else if (value === "double") {
    cameraMode = "double";
    
    // hide everything to do with single camera view
    document.getElementById('camera-large').style.display = "none";
    document.getElementById('v-crosshairs-large').style.display = "none";

    // show everything to do with double camera view
    document.getElementById('camera1').style.display = "inline";
    document.getElementById('camera2').style.display = "inline";
    document.getElementById('v-crosshairs-left').style.display = "inline";
    document.getElementById('v-crosshairs-right').style.display = "inline";
    // highlight the appropriate camera
    if (driveReversed) {
      document.getElementById('front-indicator').style.display = "inline";
    } else {
      document.getElementById('back-indicator').style.display = "inline";
    }

    // show the streams
    document.getElementById('camera-stream-1').src = "http://10.6.70.26:443/?action=stream";
    document.getElementById('camera-stream-1').onerror = "this.src='http://10.6.70.26:443/?action=stream'";
    document.getElementById('camera-stream-2').src = "http://10.6.70.26:80/?action=stream";
    document.getElementById('camera-stream-2').onerror = "this.src='http://10.6.70.26:80/?action=stream'";
  }
  NetworkTables.putValue("cameraMode", cameraMode);
});

// listens for robot-state and updates status lights and auton chooser accordingly
NetworkTables.addKeyListener('/SmartDashboard/robot-state', (key, value) => {
  if (value === "autonomousInit()" || value === "disabledPeriodic()") {
    document.getElementById('auton-chooser').style.display = "none";
  } else if (value === "autonomousPeriodic()") {
    document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
    document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
  } else if (value === "teleopInit()" || value === "teleopPeriodic()") {
    document.getElementById('auton-status').style.fill = "none";
    document.getElementById('auton-status').style.stroke = "rgb(255,255,255)";
  }
});

// listens for warnings
NetworkTables.addKeyListener('/SmartDashboard/warnings', (key, value) => {
  document.getElementById('big-warning').style.display = "inline";
  document.getElementById('warnings').innerHTML += (value + "\n");
  var timeSinceWarningFlashed = date.getTime();

  //Time is stored with prompt from Network Tables Listener, and disappears after a second
  var timing = true;
  while (timing == true) {
    if(date.getTime() - timeSinceWarningFlashed > 1000){
      document.getElementById('big-warning').style.display = "none";
      timing = false;
    }
  }
});

// displays the current arm state
NetworkTables.addKeyListener('/SmartDashboard/current-arm-state', (key, value) => {
  if (value != null) document.getElementById('current-arm-state').innerHTML = value.split("$")[1].split("@")[0];
});

NetworkTables.addGlobalListener((key, value) => {
  console.log(key + ": " + value);
})

// updates the robot diagram based on the angle of the robot's elbow
NetworkTables.addKeyListener('/SmartDashboard/elbow-angle', (key, value) => {
  console.log("elbow: " + value);
  if (value == null) return;
  var angle = value;
  document.getElementById('claw').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
  document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
  document.getElementById('arm-extension').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";
});

// updates the robot diagram with the extension of the arm
NetworkTables.addKeyListener('/SmartDashboard/arm-extension', (key, value) => {
  if (value != null) {
    if (value <= 0) {
      document.getElementById('arm-extension').setAttribute('height', 0);
    } else {
      document.getElementById('arm-extension').setAttribute('height', value * 60);
    }
  }
});

// draws the elbow angle of the destination arm state
NetworkTables.addKeyListener('/SmartDashboard/destination-elbow-angle', (key, value) => {
  if (value == null) return;
  var angle = value;
  document.getElementById('claw-destination').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
  document.getElementById('arm-destination').style = "transform: rotate(" + angle + "deg)";
  document.getElementById('arm-extension-destination').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";
});

// draws the extension of the destination arm state
NetworkTables.addKeyListener('/SmartDashboard/destination-extension-length', (key, value) => {
  if (value != null) {
    if (value <= 0) {
      document.getElementById('arm-extension-destination').setAttribute('height', 0);
    } else {
      document.getElementById('arm-extension-destination').setAttribute('height', value * 60);
    }
  }
});

// updates the angle of the intake in the robot diagram
NetworkTables.addKeyListener('/SmartDashboard/intake-angle', (key, value) => {
  if (value != null) document.getElementById('intake').style = "transform: rotate(" + (value - 90) + "deg)";
});

// updates drawing of the claw based on status
NetworkTables.addKeyListener('/SmartDashboard/claw-status', (key, value) => {
  if (value === "open") {
    document.getElementById('claw').style.stroke = "rgb(0, 200, 0)";
    document.getElementById('claw').style.fill = "rgb(0, 200, 0)";
  } else if (value === "close") {
    document.getElementById('claw').style.stroke = "rgb(0, 0, 200)";
    document.getElementById('claw').style.fill = "rgb(0, 0, 200)";
  } else if (value === "Hatch") {
    document.getElementById('claw').style.stroke = "rgb(65, 169, 244)";
    document.getElementById('claw').style.fill = "rgb(65, 169, 244)";
    document.getElementById('claw-held-item').innerHTML = "Hatch";
  } else if (value === "Ball") {
    document.getElementById('claw').style.stroke = "rgb(244, 151, 65)";
    document.getElementById('claw').style.fill = "rgb(244, 151, 65)";
    document.getElementById('claw-held-item').innerHTML = "Ball";
  } else if (value === "None") {
    document.getElementById('claw-held-item').innerHTML = "None";
  }
})

// updates status lights for intake ir
NetworkTables.addKeyListener('/SmartDashboard/intake-ir-sensor', (key, value) => {
  if (value === true) {
    document.getElementById('intake-status').style.fill = "rgb(244, 160, 65)";
    document.getElementById('intake-status').style.stroke = "rgb(244, 160, 65)";
  } else {
    document.getElementById('intake-status').style.fill = "none";
    document.getElementById('intake-status').style.stroke = "rgb(255,255,255)";
  }
});

// updates status lights for intake
NetworkTables.addKeyListener('/SmartDashboard/intake-status', (key, value) => {
  if (value === 'running-in') {
    document.getElementById('intake-status').style.fill = "rgb(0,255,0)";
    document.getElementById('intake-status').style.stroke = "rgb(0,255,0)";
  } else if (value === 'running-out') {
    document.getElementById('intake-status').style.fill = "rgb(255,0,0)";
    document.getElementById('intake-status').style.stroke = "rgb(255,0,0)";
  }
});

// updates status lights for vision
NetworkTables.addKeyListener('/SmartDashboard/vision-status', (key, value) => {
  if (value === 'engaged') {
    document.getElementById('vision-status').style.fill = "rgb(0,255,0)";
    document.getElementById('vision-status').style.stroke = "rgb(0,255,0)";
  } else if (value === 'invalid-target') {
    document.getElementById('vision-status').style.fill = "rgb(241,244,66)";
    document.getElementById('vision-status').style.stroke = "rgb(241,244,66)";
  } else if (value === 'error' || value === -99999) {
    document.getElementById('vision-status').style.fill = "rgb(255,0,0)";
    document.getElementById('vision-status').style.stroke = "rgb(255,0,0)";
  } else {
    document.getElementById('vision-status').style.fill = "none";
    document.getElementById('vision-status').style.stroke = "rgb(255,255,255)";
  }
});

// updates status lights and flip cameras for reversed drive
NetworkTables.addKeyListener('/SmartDashboard/drive-reversed', (key, value) => {
  // show drive reversed status in status lights
  if (value === true) {
    document.getElementById('drive-reversed-status').style.fill = "rgb(255,255,255)";
    document.getElementById('drive-reversed-status').style.stroke = "rgb(255,255,255)";
    driveReversed = true;
  } else {
    document.getElementById('drive-reversed-status').style.fill = "none";
    document.getElementById('drive-reversed-status').style.stroke = "rgb(255,255,255)";
    driveReversed = false;
  }
  // if currently using single camera mode display the correct stream on it
  if (cameraMode === "single") {
    if (value === true) {
      document.getElementById('camera-stream-large').src = "http://10.6.70.26:443/?action=stream";
      document.getElementById('camera-stream-large').onerror = "this.src='http://10.6.70.26:443/?action=stream'";
      document.getElementById('v-crosshairs-large').style = "transform: translate(50vw, 0vh); rotate(10deg)";
    } else {
      document.getElementById('camera-stream-large').src = "http://10.6.70.26:80/?action=stream";
      document.getElementById('camera-stream-large').onerror = "this.src='http://10.6.70.26:80/?action=stream'";
      document.getElementById('v-crosshairs-large').style = "transform: translate(50vw, 0vh); rotate(-10deg)";
    }
  } 
  // if currently using double camera mode display the crosshairs and appropriate indicator
  if (cameraMode === "double") {
    document.getElementById('v-crosshairs-left').style.display = "inline";
    document.getElementById('v-crosshairs-right').style.display = "inline";
    if (value === true) {
      document.getElementById('back-indicator').style.display = "none";
      document.getElementById('front-indicator').style.display = "inline";
    } else {
      document.getElementById('front-indicator').style.display = "none";
      document.getElementById('back-indicator').style.display = "inline";
    }
  }
  
});

// listens for keystrokes from the external keypad and passes the corresponding values over networktables
var keys = [];
var allKeys = '';
document.addEventListener("keyup", function(event) {
  var pressed = event.key.replace("Enter", "");
  allKeys += pressed;
  var result = allKeys[allKeys.length - 1];
  var nextTask = getFromMap(result);

  console.log(nextTask);
  allKeysPressed.push(nextTask);

  // make sure the key pressed is a valid action
  if (nextTask != null) {
    if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue('/SmartDashboard/xkeys-armstates', nextTask);
    else if (nextTask.includes("cancel")) NetworkTables.putValue('/SmartDashboard/xkeys-cancel', nextTask);
    else if (nextTask === "place" || nextTask === "grab" || nextTask === "drop_held_item" || nextTask === "toggle_held_item") NetworkTables.putValue('/SmartDashboard/xkeys-placing', nextTask);
    else if (nextTask.includes("toggle_intake") || nextTask.includes("run_intake") || nextTask.includes("bring_intake_in")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask);
    else if (nextTask === "auto_pickup_ball") NetworkTables.putValue('/SmartDashboard/xkeys-autopickup', nextTask);
    else if (nextTask.includes("vision")) NetworkTables.putValue('/SmartDashboard/xkeys-visiondrive', nextTask);
    else if (nextTask.includes("claw")) NetworkTables.putValue('/SmartDashboard/xkeys-claw', nextTask);
    else if (nextTask.includes("extension")) NetworkTables.putValue('/SmartDashboard/xkeys-extension', nextTask);
  }
});

// naming convention: UPPER_CASE for preset arm states, lower_case for other commands
function getFromMap(key) { // mapping is more aligned with arm position on robot
  if (key === "w") return "toggle_intake_in";
  if (key === "y") return "toggle_intake_out";
  if (key === "t") return "run_intake_in_with_IR";
  if (key === "v") return "auto_pickup_ball";

  if (key === "p") return "place";
  if (key === "c") return "grab";
  if (key === "1") return "drop_held_item";
  if (key === "4") return "toggle_held_item";

  if (key === "g") return "READY_PLACE_HATCH_ROCKET_MIDDLE_BACK";
  if (key === "k") return "READY_PLACE_BALL_ROCKET_MIDDLE_BACK";
  if (key === "n") return "READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD";
  if (key === "a") return "NEUTRAL";

  if (key === "f") return "GRAB_BALL_LOADINGSTATION_BACK";
  if (key === "j") return "PLACE_BALL_CARGOSHIP_BACK";
  if (key === "m") return "PLACE_BALL_CARGOSHIP_FORWARD";
  if (key === "3") return "GRAB_BALL_LOADINGSTATION_FORWARD";

  if (key === "2") return "READY_LOW_HATCH_BACK";
  if (key === "i") return "READY_PLACE_BALL_ROCKET_LOW_BACK";
  if (key === "8") return "READY_PLACE_BALL_ROCKET_LOW_FORWARD";
  if (key === "q") return "READY_LOW_HATCH_FORWARD";

  if (key === "d") return "READY_GRAB_HATCH_GROUND_BACK";
  if (key === "h") return "GRAB_BALL_GROUND_BACK";
  if (key === "x") return "GRAB_BALL_INTAKE";
  if (key === "b") return "STOW";

  if (key === "s") return "cancel_drive";
  if (key === "u") return "cancel_intake";
  if (key === "o") return "cancel_arm";
  if (key === "z") return "cancel_all";

  if (key === "5") return "toggle_claw";
  if (key === "6") return "reset_extension";
  if (key === "7") return "bring_intake_in";

  if (key === "0") return "vision_drive";

  return null;
}
