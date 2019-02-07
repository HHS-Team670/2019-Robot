// Define UI elements
var ui = {
    robotState: document.getElementById('robot-state'),
    multiCamSRC: document.getElementById('multicam-src'),
    navx: {
      arm: document.getElementById('navx-arm'),
      number: document.getElementById('navx-number')
    },
    turret: {
      arm: document.getElementById('turret-arm'),
      number: document.getElementById('turret-number')
    }
};

var date = new Date();
document.getElementById('big-warning').style.display = "none";
document.getElementById('climb-state-text').style.stroke = `rgb(60, 60, 60)`;
document.getElementById('climb-level-text').style.stroke = `rgb(60, 60, 60)`;
var angle = 0;
document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
document.getElementById('claw').style = "transform: translate(" + Math.sin(angle * Math.PI / 180) * 60 + "px, " + -1 * Math.cos(angle * Math.PI / 180) * 60 + "px)";
document.getElementById('intake').style = "transform: rotate(" + 0 + "deg)";


NetworkTables.addKeyListener('/SmartDashboard/robotTime', (key, value) => {
  var minutes = ~~(value / 60); // converts to integer
  var seconds = (value - 60*minutes) % 60;
  seconds = (seconds < 10) ? '0'+seconds : seconds;

  ui.timer.style.color = `rgb(0, 200, 0)`;
  if (value < 135){
     ui.timer.style.color = `rgb(255, 255, 255)`;
  }
  if (value < 45){ 
    ui.timer.style.color = `rgb(244,215,66)`
    document.getElementById('climb-state-text').style.stroke = `rgb(255, 255, 255)`;
    document.getElementById('climb-level-text').style.stroke = `rgb(255, 255, 255)`;
  }
  if (value < 30){
     ui.timer.style.color = `rgb(200, 0, 0)`;
  }
  ui.timer.innerHTML = minutes + ':' + seconds;
});

NetworkTables.addKeyListener('/SmartDashboard/cameraSource', (key, value) => {
  if (value == 'next') {
    window.webContents.reload();
  }
  NetworkTables.putValue('/SmartDashboard/cameraSource', '');
});

NetworkTables.addKeyListener('/SmartDashboard/robotState', (key, value) => {
  if (value === "autonomousInit()") document.getElementById('auton-chooser').style.display = "none";
});

document.getElementById('auton-chooser').style.display = "none";
document.getElementById('robot-diagram').style.display = "inline";

var timeSinceWarningFlashed = 0;

NetworkTables.addKeyListener('/SmartDashboard/warning', (key, value) => {
  document.getElementById('big-warning').style.display = "inline";
  document.getElementById('warnings').innerHTML += (value + "\n");
   timeSinceWarningFlashed = date.getTime();
});

//Time is stored with prompt from Network Tables Listener, and disappears after a second
if(date.getTime() - timeSinceWarningFlashed > 1000){
  document.getElementById('big-warning').style.display = "none";
}

NetworkTables.addKeyListener('/SmartDashboard/current-command', (key, value) => {
  document.getElementById('current-command').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/climb-state', (key, value) => {
  document.getElementById('current-command').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/climb-level', (key, value) => {
  document.getElementById('current-command').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/elbow-angle', (key, value) => {
  var angle = value;
  document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
  document.getElementById('claw').style = "transform: translate(" + Math.sin(angle * Math.PI / 180) * 50 + "px, " + -1 * Math.cos(angle * Math.PI / 180) * 50 + "px)";

  var height = 0;
  if (angle >= 225 && angle < 315) {
    var height = 50 + Math.cos((angle-225)*2 * Math.PI / 180) * 50;
  } else if (angle >= 45 && angle < 135) {
    height = 0;
  } else if (angle < 45 && angle >= 315) {
    var height = 50 - Math.cos((angle-45)*2 * Math.PI / 180) * 50;
  } else if (angle >= 135 && angle < 225) {
    height = 100;
  }
  document.getElementById('hline').setAttribute('y', height+'%');
});

NetworkTables.addKeyListener('/SmartDashboard/claw-ir-sensor', (key, value) => {
  if (value === 'holding-hatch') {
    document.getElementById('claw').style.stroke = "rgb(65, 169, 244)";
    document.getElementById('claw-status').style.fill = "rgb(65, 169, 244)";
    document.getElementById('claw-status').style.stroke = "rgb(65, 169, 244)";
  } else if (value === 'open') {
    document.getElementById('claw').style.stroke = "rgb(255,255,255)";
    document.getElementById('claw-status').style.fill = "none";
    document.getElementById('claw-status').style.stroke = "rgb(255,255,255)";
  } else if (value === 'holding-ball') {
    document.getElementById('claw').style.stroke = "rgb(244, 151, 65)";
    document.getElementById('claw-status').style.fill = "rgb(244, 151, 65)";
    document.getElementById('claw-status').style.stroke = "rgb(244, 151, 65)";
  } else if (value === 'closed') {
    document.getElementById('claw').style.stroke = "rgb(65, 169, 244)";
    document.getElementById('claw-status').style.fill = "rgb(65, 169, 244)";
    document.getElementById('claw-status').style.stroke = "rgb(65, 169, 244)";
  }
});

NetworkTables.addKeyListener('/SmartDashboard/intake-status', (key, value) => {
  if (value === 'running-in') {
    document.getElementById('intake-status').style.fill = "rgb(0,255,0)";
    document.getElementById('intake-status').style.stroke = "rgb(0,255,0)";
  } else if (value === 'running-out') {
    document.getElementById('intake-status').style.fill = "rgb(255,0,0)";
    document.getElementById('intake-status').style.stroke = "rgb(255,0,0)";
  } else if (value === 'intake-ir-triggered') {
    document.getElementById('intake-status').style.fill = "rgb(244, 160, 65)";
    document.getElementById('intake-status').style.stroke = "rgb(244, 160, 65)";
  } else {
    document.getElementById('intake-status').style.fill = "none";
    document.getElementById('intake-status').style.stroke = "rgb(255,255,255)";
  }
});

NetworkTables.addKeyListener('/SmartDashboard/intake-angle', (key, value) => {
  document.getElementById('intake').style = "transform: rotate(" + -1 * value + "deg)";
});

NetworkTables.addKeyListener('/SmartDashboard/vision-status', (key, value) => {
  if (value === 'engaged') {
    document.getElementById('vision-status').style.fill = "rgb(0,255,0)";
    document.getElementById('vision-status').style.stroke = "rgb(0,255,0)";
  } else if (value === 'error') {
    document.getElementById('vision-status').style.fill = "rgb(255,0,0)";
    document.getElementById('vision-status').style.stroke = "rgb(255,0,0)";
  } else {
    document.getElementById('vision-status').style.fill = "none";
    document.getElementById('vision-status').style.stroke = "rgb(255,255,255)";
  }
});

NetworkTables.addKeyListener('/SmartDashboard/auton-status', (key, value) => {
  if (value === 'running') {
    document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
    document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
  } else {
    document.getElementById('auton-status').style.fill = "none";
    document.getElementById('auton-status').style.stroke = "rgb(255,255,255)";
  }
});

var keys = [];

var allKeys = '';
document.addEventListener("keyup", function(event) {
  var pressed = event.key.replace("Enter", "");
  allKeys += pressed;
  var split = allKeys.split(" ");
  var result = split[split.length - 1];
  var nextTask = getFromMap(result);
  if (nextTask != null) {
    document.getElementById('warnings').innerHTML = nextTask;
    if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue('/SmartDashboard/xkeys-armstates', nextTask);
    else if (nextTask === "place" || nextTask === "grab") NetworkTables.putValue('/SmartDashboard/xkeys-placing', nextTask);
    else if (nextTask.includes("run_intake")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask);
    else if (nextTask === "auto_pickup_ball") NetworkTables.putValue('/SmartDashboard/xkeys-autopickup', nextTask);
    else if (nextTask.includes("climb")) NetworkTables.putValue('/SmartDashboard/xkeys-climber', nextTask);
  }
  document.getElementById('test2').innerHTML = armStates.length;
});

// naming convention: UPPER_CASE for preset arm states, lower_case for other commands
function getFromMap(key) { // mapping is more aligned with arm position on robot
  if (key === "x0a") return "READY_TO_CLIMB";
  if (key === "x03") return "set_climb_3";
  if (key === "x04") return "set_climb_2";
  if (key === "x05") return "set_climb_flat";
  if (key === "x0b") return "cycle_climb";
  if (key === "x0c") return "piston_climb";
  if (key === "x0d") return "cancel_arm_climb";

  if (key === "x33") return "run_intake_in";
  if (key === "x3b") return "run_intake_out";
  if (key === "x34") return "auto_pickup_ball";

  if (key === "x2e") return "place";
  if (key === "x26") return "grab";

  if (key === "x12") return "READY_PLACE_HATCH_ROCKET_MIDDLE_BACK";
  if (key === "x1a") return "READY_PLACE_BALL_ROCKET_MIDDLE_BACK";  
  if (key === "x22") return "READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD";
  if (key === "x2a") return "READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD";

  if (key === "x13") return "READY_GRAB_BALL_LOADINGSTATION_BACK";
  if (key === "x1b") return "PLACE_BALL_CARGOSHIP_BACK";
  if (key === "x23") return "PLACE_BALL_CARGOSHIP_FORWARD";
  if (key === "x2b") return "READY_GRAB_BALL_LOADINGSTATION_FORWARD";

  if (key === "x14") return "READY_LOW_HATCH_BACK";
  if (key === "x1c") return "READY_PLACE_BALL_ROCKET_LOW_BACK";
  if (key === "x24") return "READY_PLACE_BALL_ROCKET_LOW_FORWARD";
  if (key === "x2c") return "READY_LOW_HATCH_FORWARD";

  if (key === "x1d") return "GRAB_BALL_GROUND_BACK";
  if (key === "x25") return "GRAB_BALL_INTAKE";
}

// naming convention: UPPER_CASE for preset arm states, lower_case for other commands
function getActionAlternate(key) {
  if (key === "x06") return "backspace";
  if (key === "x3e") return "enter";

  if (key === "x0a") return "READY_TO_CLIMB";
  if (key === "x03") return "next_step_climb";
  if (key === "x0b") return "cancel_arm_climb";

  if (key === "x33") return "run_intake_in";
  if (key === "x3b") return "run_intake_out";

  if (key === "x12") return "READY_LOW_HATCH_BACK";
  if (key === "x13") return "READY_PLACE_BALL_ROCKET_LOW_BACK";
  
  if (key === "x15") return "GRAB_BALL_GROUND_BACK";
  
  if (key === "x1a") return "READY_PLACE_HATCH_ROCKET_MIDDLE_BACK";
  if (key === "x1b") return "READY_PLACE_BALL_ROCKET_MIDDLE_BACK";
  if (key === "x1c") return "PLACE_BALL_CARGOSHIP_BACK";
  if (key === "x1d") return "READY_GRAB_BALL_LOADINGSTATION_BACK";
  
  if (key === "x22") return "READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD";
  if (key === "x23") return "READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD";
  if (key === "x24") return "PLACE_BALL_CARGOSHIP_FORWARD";
  if (key === "x25") return "READY_GRAB_BALL_LOADINGSTATION_FORWARD";
  if (key === "x26") return "grab";
  if (key === "x2a") return "READY_LOW_HATCH_FORWARD";
  if (key === "x2b") return "READY_PLACE_BALL_ROCKET_LOW_FORWARD";
  
  if (key === "x2d") return "GRAB_BALL_INTAKE";
  if (key === "x2e") return "place";
  return null;
}

function finalize() {
  NetworkTables.putValue('armSequence', armStates);
}

function readRadioButtons() {
  var startDirection = document.forms['auto-chooser'].elements['start-direction'];
  for (var i = 0, len = startChooser.length; i < len; i++) {
    if (startChooser[i].checked) {
      auton[0] = startDirection[i].value;
      break;
    }
  }
  var startChooser = document.forms['auto-chooser'].elements['start'];
  for (var i = 0, len = startChooser.length; i < len; i++) {
    if (startChooser[i].checked) {
      auton[1] = startChooser[i].value;
      break;
    }
  }
  var target1 = document.forms['auto-chooser'].elements['target1'];
  for (var i = 0, len = target1.length; i < len; i++) {
    if (target1[i].checked) {
      auton[2] = target1[i].value;
      break;
    }
  }
  var height1 = document.forms['auto-chooser'].elements['height1'];
  for (var i = 0, len = height1.length; i < len; i++) {
    if (height1[i].checked) {
      auton[3] = height1[i].value;
      break;
    }
  }
  var target2 = document.forms['auto-chooser'].elements['target2'];
  for (var i = 0, len = target2.length; i < len; i++) {
    if (target2[i].checked) {
      auton[4] = target2[i].value;
      break;
    }
  }
  var height2 = document.forms['auto-chooser'].elements['height1'];
  for (var i = 0, len = height2.length; i < len; i++) {
    if (height2[i].checked) {
      auton[5] = height2[i].value;
      break;
    }
  }
  var target3 = document.forms['auto-chooser'].elements['target2'];
  for (var i = 0, len = target3.length; i < len; i++) {
    if (target3[i].checked) {
      auton[6] = target3[i].value;
      break;
    }
  }
  var height3 = document.forms['auto-chooser'].elements['height1'];
  for (var i = 0, len = height3.length; i < len; i++) {
    if (height3[i].checked) {
      auton[7] = height3[i].value;
      break;
    }
  }
}

document.getElementById('confirm-button').onclick = function () {
  readRadioButtons();
  document.getElementById('test').innerHTML = auton;
  NetworkTables.putValue("autonSequence", auton);
}

document.getElementById('claw-status').setAttribute("fill", "red");