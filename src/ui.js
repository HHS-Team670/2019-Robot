// Define UI elements
var ui = {
    multiCamSRC: document.getElementById('multicam-src'),
    timer: document.getElementById('timer')
};

var date = new Date();
document.getElementById('big-warning').style.display = "none";

document.getElementById('climb-state-text').style.stroke = `rgb(90, 90, 90)`;
document.getElementById('climb-level-text').style.stroke = `rgb(90, 90, 90)`;

document.getElementById('auton-chooser').style.display = "none";
ui.timer.style.color = `rgb(0, 200, 0)`;

var angle = 0;
var armLength = 110;
document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
document.getElementById('claw').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
document.getElementById('intake').style = "transform: rotate(" + 0 + "deg)";
document.getElementById('arm-extension').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";

document.getElementById('vline-right').setAttribute('x', 70+'%');
document.getElementById('vline-left').setAttribute('x', 25+'%');

var cameras = ['Back', 'Front'];
var cameraIndex = 0;
multiCamSRC.innerHTML = cameras[cameraIndex];

document.getElementById('timer').style.color = 'rgb(0,200,0)';

NetworkTables.addKeyListener('/SmartDashboard/game-time', (key, value) => {
  if (value == null) return;
  var remaining = 150 - value;
  var minutes = ~~(remaining / 60); // converts to integer
  var seconds = (remaining - 60*minutes) % 60;
  seconds = (seconds < 10) ? '0'+seconds : seconds;
  ui.timer.style.color = `rgb(0, 200, 0)`;

  if (remaining < 135){
     ui.timer.style.color = `rgb(255, 255, 255)`;
  }
  if (remaining < 45){
    ui.timer.style.color = `rgb(244,215,66)`;
    document.getElementById('climb-state-text').style.stroke = `rgb(255, 255, 255)`;
    document.getElementById('climb-level-text').style.stroke = `rgb(255, 255, 255)`;
  }
  if (remaining < 30){
     ui.timer.style.color = `rgb(200, 0, 0)`;
  }
  ui.timer.innerHTML = minutes + ':' + seconds;
});

NetworkTables.addKeyListener('/SmartDashboard/camera-source', (key, value) => {
  if (value == 'next') {
    window.webContents.reload();
    cameraIndex = (cameraIndex + 1) % cameras.length;
    multiCamSRC.innerHTML = cameras[cameraIndex];
  }
  NetworkTables.putValue('/SmartDashboard/camera-source', '');
});

NetworkTables.addKeyListener('/SmartDashboard/robot-state', (key, value) => {
  if (value === "autonomousInit()") {
    document.getElementById('auton-chooser').style.display = "none";
  } else if (value === "autonomousPeriodic()") {
    document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
    document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
  } else if (value === "teleopInit()" || value === "teleopPeriodic()") {
    document.getElementById('auton-status').style.fill = "none";
    document.getElementById('auton-status').style.stroke = "rgb(255,255,255)";
  }
});

NetworkTables.addKeyListener('/SmartDashboard/warning', (key, value) => {
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

NetworkTables.addKeyListener('/SmartDashboard/current-command', (key, value) => {
  if (value != 'undefined') document.getElementById('current-command').innerHTML = value;
  else document.getElementById('current-command').innerHTML = "NULL";
});

NetworkTables.addKeyListener('/SmartDashboard/climb-state', (key, value) => {
  if (value != null) document.getElementById('climb-state-text').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/climb-level', (key, value) => {
  if (value != null) document.getElementById('climb-level-text').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/current-arm-state', (key, value) => {
  if (value != null) document.getElementById('current-arm-state').innerHTML = value;
});

NetworkTables.addKeyListener('/SmartDashboard/elbow-angle', (key, value) => {
  if (value == null) return;
  var angle = value;
  document.getElementById('claw').style = "transform: translate(" + (Math.sin(angle * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px, " + (armLength - Math.sin((angle+90) * Math.PI / 180) * (parseInt(document.getElementById('arm-extension').getAttribute('height')) + armLength)) + "px)";
  document.getElementById('arm').style = "transform: rotate(" + angle + "deg)";
  document.getElementById('arm-extension').style = "transform: translate(" + (Math.sin((angle) * Math.PI / 180) * armLength) + "px, " + (armLength - (Math.sin((angle+90) * Math.PI / 180) * armLength)) + "px) rotate(" + (angle + 180) + "deg)";

  var frontHeight = 0;
  var backHeight = 0;
  if (angle >= 42.65 && angle < 172.35) {
    frontHeight = (angle - 42.65) / (172.35 - 42.65) * 100;
  } else if (angle >= 172.35 && angle < 187.65) {
    frontHeight = 100;
    backHeight = 100;
  } else if (angle >= 187.65 && angle < 227.35) {
    backHeight = (angle - 187.65) / (227.35 - 187.65) * 100;
  }
  // document.getElementById('hline-left').setAttribute('y', frontHeight+'%');
  // document.getElementById('hline-right').setAttribute('y', backHeight+'%');
  if (multiCamSRC.innerHTML === 'Front') document.getElementById('hline').setAttribute('y', frontHeight+'%');
  if (multiCamSRC.innerHTML === 'Back') document.getElementById('hline').setAttribute('y', backHeight+'%');
});

NetworkTables.addKeyListener('/SmartDashboard/intake-angle', (key, value) => {
  if (value != null) document.getElementById('intake').style = "transform: rotate(" + -1 * value + "deg)";
});

NetworkTables.addKeyListener('/SmartDashboard/arm-extension', (key, value) => {
  if (value != null) document.getElementById('arm-extension').setAttribute('height', value * 60);
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

NetworkTables.addKeyListener('/SmartDashboard/intake-ir-sensor', (key, value) => {
  if (value === true) {
    document.getElementById('intake-status').style.fill = "rgb(244, 160, 65)";
    document.getElementById('intake-status').style.stroke = "rgb(244, 160, 65)";
  } else {
    document.getElementById('intake-status').style.fill = "none";
    document.getElementById('intake-status').style.stroke = "rgb(255,255,255)";
  }
});

NetworkTables.addKeyListener('/SmartDashboard/intake-status', (key, value) => {
  if (value === 'running-in') {
    document.getElementById('intake-status').style.fill = "rgb(0,255,0)";
    document.getElementById('intake-status').style.stroke = "rgb(0,255,0)";
  } else if (value === 'running-out') {
    document.getElementById('intake-status').style.fill = "rgb(255,0,0)";
    document.getElementById('intake-status').style.stroke = "rgb(255,0,0)";
  }
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

var keys = [];

var allKeys = '';
document.addEventListener("keyup", function(event) {
  var pressed = event.key.replace("Enter", "");
  allKeys += pressed;
  var result = allKeys[allKeys.length - 1];
  // var result = (split[split.length - 1]).toLowerCase();
  NetworkTables.putValue('/SmartDashboard/current-command-text', result);
  var nextTask = getFromMap(result);

  if (nextTask != null) {
    if (nextTask.toUpperCase() === nextTask) {
      NetworkTables.putValue('/SmartDashboard/xkeys-armstates', nextTask);
    }
    else if (nextTask.includes("cancel")) NetworkTables.putValue('/SmartDashboard/xkeys-cancel', nextTask);
    else if (nextTask === "place" || nextTask === "grab") NetworkTables.putValue('/SmartDashboard/xkeys-placing', nextTask);
    else if (nextTask.includes("run_intake")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask);
    else if (nextTask === "auto_pickup_ball") NetworkTables.putValue('/SmartDashboard/xkeys-autopickup', nextTask);
    else if (nextTask.includes("climb")) NetworkTables.putValue('/SmartDashboard/xkeys-climber', nextTask);
  }
});

// naming convention: UPPER_CASE for preset arm states, lower_case for other commands
function getFromMap(key) { // mapping is more aligned with arm position on robot
  NetworkTables.putValue('/SmartDashboard/current-command-text', '>>>'+key+'<<<');

  if (key === "x3c") return "toggle_intake_in";
  if (key === "x3b") return "toggle_intake_out";
  if (key === "x34") return "run_intake_in_with_IR";
  if (key === "x3e") return "auto_pickup_ball";

  if (key === "w") return "place";
  if (key === "9") return "grab";

  if (key === "a") return "READY_PLACE_HATCH_ROCKET_MIDDLE_BACK";
  if (key === "b") return "READY_PLACE_BALL_ROCKET_MIDDLE_BACK";
  if (key === "c") return "READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD";

  if (key === "j") return "GRAB_BALL_LOADINGSTATION_BACK";
  if (key === "k") return "PLACE_BALL_CARGOSHIP_BACK";
  if (key === "l") return "PLACE_BALL_CARGOSHIP_FORWARD";
  if (key === "m") return "GRAB_BALL_LOADINGSTATION_FORWARD";

  if (key === "s") return "READY_LOW_HATCH_BACK";
  if (key === "t") return "READY_PLACE_BALL_ROCKET_LOW_BACK";
  if (key === "u") return "READY_PLACE_BALL_ROCKET_LOW_FORWARD";
  if (key === "v") return "READY_LOW_HATCH_FORWARD";

  if (key === "2") return "GRAB_BALL_GROUND_BACK";
  if (key === "x32") return "GRAB_BALL_INTAKE";
  if (key === "1") return "READY_GRAB_HATCH_GROUND_BACK";
  if (key === "x2d") return "STOW";

  if (key === "o") return "cancel_drive";
  if (key === "p") return "cancel_intake";
  if (key === "d") return "cancel_arm";
  if (key === "m") return "cancel_arm_climb";
  if (key === "x0e") return "cancel_piston_climb";
  if (key === "rq" || key === "qr" || key === "q" || key === "r") return "cancel_all";

  return null;
}

var auton = ["test", "", "", "", "", "", ""];

function readRadioButtons() {
  auton = new Array(7);

  var startChooser = document.forms["auto-chooser"].elements["start"];
  for (var i = 0, len = startChooser.length; i < len; i++) {
    if (startChooser[i].checked) {
      auton[0] = startChooser[i].value;
      break;
    }
  }

  var target1 = document.forms['auto-chooser'].elements['target1'];
  for (var i = 0, len = target1.length; i < len; i++) {
    if (target1[i].checked) {
      auton[1] = target1[i].value;
      break;
    }
  }

  var height1 = document.forms['auto-chooser'].elements['height1'];
  for (var i = 0, len = height1.length; i < len; i++) {
    if (height1[i].checked) {
      auton[2] = height1[i].value;
      break;
    }
  }

  var target2 = document.forms['auto-chooser'].elements['target2'];
  for (var i = 0, len = target2.length; i < len; i++) {
    if (target2[i].checked) {
      auton[3] = target2[i].value;
      break;
    }
  }

  var height2 = document.forms['auto-chooser'].elements['height2'];
  for (var i = 0, len = height2.length; i < len; i++) {
    if (height2[i].checked) {
      auton[4] = height2[i].value;
      break;
    }
  }

  var target3 = document.forms['auto-chooser'].elements['target2'];
  for (var i = 0, len = target3.length; i < len; i++) {
    if (target3[i].checked) {
      auton[5] = target3[i].value;
      break;
    }
  }

  var height3 = document.forms['auto-chooser'].elements['height3'];
  for (var i = 0, len = height3.length; i < len; i++) {
    if (height3[i].checked) {
      auton[6] = height3[i].value;
      break;
    }
  }
}

function sendAuton() {
  readRadioButtons();
  document.getElementById('warnings').innerHTML = auton;
  NetworkTables.putValue("/SmartDashboard/auton-sequence", auton);
}
