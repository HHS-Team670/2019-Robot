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

NetworkTables.addKeyListener('/SmartDashboard/gyro', (key, value) => {
  var angle = value % 360;
  ui.navx.number.innerHTML = angle + 'º';
  ui.navx.arm.style.transform = `rotate(${angle}deg)`;
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

var keys = [];

var allKeys = '';
document.addEventListener("keyup", function(event) {
  var pressed = event.key.replace("Enter", "");
  allKeys += pressed;
  var split = allKeys.split(" ");
  var result = split[split.length - 1];
  var nextTask = getFromMap(result);
  if (nextTask != null) {
    document.getElementById('test').innerHTML = nextTask;
    if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue("xkeys-armstates", nextTask);
    else if (nextTask === "place" || nextTask === "grab") NetworkTables.putValue("xkeys-placing", nextTask);
    else if (nextTask.includes("run_intake")) NetworkTables.putValue("xkeys-intake", nextTask);
    else if (nextTask === "auto_pickup_ball") NetworkTables.putValue("xkeys-autopickup", nextTask);
    else if (nextTask.includes("climb")) NetworkTables.putValue("xkeys-climber", nextTask);
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

document.getElementById('test-button').onclick = function () {
  readRadioButtons();
  document.getElementById('test').innerHTML = auton;
  NetworkTables.putValue("autonSequence", auton);
}
