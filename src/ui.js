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

NetworkTables.addKeyListener('/SmartDashboard/teleopState', (key, value) => {
  document.getElementById('auton-chooser').style.display = "none";
});

// NetworkTables.addKeyListener(/* claw grip status */'', (key, value) => {
//   var clawLight = document.getElementById('claw-status');
//   if (value == 'open') clawLight.style.fill = `white`;
//   else if (value == 'soft') clawLight.style.fill = `green`;
//   else if (value == 'hard') clawLight.style.fill = `blue`;
// });

var keys = [];
var armStates = [];
var auton = new Array(7);

window.onkeyup = function(event) {
    let key = event.key;

    if (key == '9') {
      keys.pop();
    } else if (key == 'w') {
      finalize();
    } else {
      keys.push(key);
    }

    if (keys.length > 0) {

    }
    if (keys.length > 1) {

    }
    
}

/*
a b c d
j k l m
s t u v
1 2 3 4
9 0 a w
*/

// returns the location on the field corresponding with a given key
function getArmState(key) {
  // still unmapped: START_BALL, START_HATCH, START_EMPTY, INTAKE_HATCH_GROUND
  if (key == 'a') return 'NEUTRAL';
  if (key == 'b') return 'READY_TO_CLIMB';
  if (key == 'c') return 'STOW';
  if (key == 'd') return 'DEFENSE';
  if (key == 'j') return 'INTAKE_BALL_GROUND';
  if (key == 'k') return 'INTAKE_BALL_INTAKE';
  if (key == 'l') return 'INTAKE_BALL_LOADINGSTATION';
  if (key == 'm') return 'INTAKE_HATCH_LOADINGSTATION';
  if (key == 's') return 'PLACE_BALL_CARGOSHIP';
  if (key == 't') return 'PLACE_HATCH_CARGOSHIP';
  if (key == 'u') return '';
  if (key == 'v') return '';
  if (key == '1') return 'PLACE_BALL_ROCKET_LOW';
  if (key == '2') return 'PLACE_BALL_ROCKET_MIDDLE';
  if (key == '3') return 'PLACE_HATCH_ROCKET_LOW';
  if (key == '4') return 'PLACE_HATCH_ROCKET_MIDDLE';
  if (key == '9') return '';
  if (key == 'w') return '';
  return null;
}

function getSide(key) {
  if (key == 'a') return 'BACK';
  if (key == 'd') return 'FRONT';
  return null;
}

function finalize() {
  NetworkTables.putValue('armSequence', armStates);
  NetworkTables.putValue('autonSequence', auton);
}
document.getElementById('test').innerHTML = 'changed';

function readRadioButtons() {
  var startChooser = document.forms['auto-chooser'].elements['start'];
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
  var height2 = document.forms['auto-chooser'].elements['height1'];
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
  var height3 = document.forms['auto-chooser'].elements['height1'];
  for (var i = 0, len = height3.length; i < len; i++) {
    if (height3[i].checked) {
      auton[6] = height3[i].value;
      break;
    }
  }
}

document.getElementById('test-button').onclick = function () {
  readRadioButtons();
  document.getElementById('test').innerHTML = auton;
}
