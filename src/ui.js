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
});

var keys = [];
var armStates = [];
var auton = new Array(6);

window.onkeyup = function(event) {
    let key = event.key;

    if (key == '9') {
      keys.pop();
    } else if (key == 'w') {
      finalize();
    } else {
      keys.push(key);
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
  if (key == 'a') return 'NEUTRAL';
  if (key == 'b') return 'READY_TO_CLIMB';
  if (key == 'c') return 'STOW';
  if (key == 'd') return 'DEFENSE';
  if (key == 'j') return '';
  if (key == 'k') return '';
  if (key == 'l') return '';
  if (key == 'm') return '';
  if (key == 's') return '';
  if (key == 't') return '';
  if (key == 'u') return '';
  if (key == 'v') return '';
  if (key == '1') return '';
  if (key == '2') return '';
  if (key == '3') return '';
  if (key == '4') return '';
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

var startChooser = document.forms['auto-chooser'].elements['start'];
for (var i = 0, len = startChooser.length; i < len; i++) {
  startChooser[i].onclick = function() {
    auton[0] = this.value;
  }
}
var target1 = document.forms['auto-chooser'].elements['target1'];
for (var i = 0, len = target1.length; i < len; i++) {
  target1[i].onclick = function() {
    auton[1] = this.value;
  }
}
var height1 = document.forms['auto-chooser'].elements['height1'];
for (var i = 0, len = height1.length; i < len; i++) {
  height1[i].onclick = function() {
    auton[2] = this.value;
  }
}
var target2 = document.forms['auto-chooser'].elements['target2'];
for (var i = 0, len = target2.length; i < len; i++) {
  target2[i].onclick = function() {
    auton[1] = this.value;
  }
}
var height2 = document.forms['auto-chooser'].elements['height1'];
for (var i = 0, len = height2.length; i < len; i++) {
  height2[i].onclick = function() {
    auton[3] = this.value;
  }
}
var target3 = document.forms['auto-chooser'].elements['target2'];
for (var i = 0, len = target3.length; i < len; i++) {
  target3[i].onclick = function() {
    auton[4] = this.value;
  }
}
var height3 = document.forms['auto-chooser'].elements['height1'];
for (var i = 0, len = height3.length; i < len; i++) {
  height3[i].onclick = function() {
    auton[5] = this.value;
  }
}