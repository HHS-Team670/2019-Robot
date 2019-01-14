// Define UI elements
var ui = {
    timer: document.getElementById('timer'),
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

let allFieldLocations = ["left-rocket-top", "left-rocket-middle", "left-rocket-bottom", "left-cargo-far", "left-cargo-mid", "left-cargo-close", "left-cargo-alliance", "right-cargo-alliance", "right-cargo-far", "right-cargo-mid", "right-cargo-close", "right-rocket-top", "right-rocket-middle", "right-rocket-bottom", "left-loading-cargo", "left-loading-plate", "right-loading-cargo", "right-loading-plate"];

NetworkTables.addKeyListener('/SmartDashboard/robotTime', (key, value) => {
  var minutes = ~~(value / 60); // converts to integer
  var seconds = (value - 60*minutes) % 60;
  seconds = (seconds < 10) ? '0'+seconds : seconds;
  ui.timer.style.color = `rgb(0, 200, 0)`;
  if (value < 135) ui.timer.style.color = `rgb(255, 255, 255)`;
  if (value < 30) ui.timer.style.color = `rgb(200, 0, 0)`;
  ui.timer.innerHTML = minutes + ':' + seconds;
});

NetworkTables.addKeyListener('/SmartDashboard/gyro', (key, value) => {
  var angle = value % 360;
  ui.navx.number.innerHTML = angle + 'º';
  ui.navx.arm.style.transform = `rotate(${angle}deg)`;
});

// NetworkTables.addKeyListener('/SmartDashboard/Safety\ Mode\ Enabled', (key, value) => {
//   document.getElementById('test').innerHTML = value;
// });

var keys = [];

window.onkeyup = function(event) {
    let key = event.key;

    if (key == '9') {
      keys = [];
    } else if (key == '0') {
      var toRemove = keys.pop();
      highlight(getMap(toRemove), 'clear');
    } else if (key == 'w') {
      finalize();
      keys = [];
    } else {
      if (keys.length == 0 && getMap(key) != 'unmapped') clearHighlight();
      if (keys.length < 2) {
        if (getMap(key) != 'unmapped') keys.push(key);
        highlight(getMap(key), 'blue');
      }
    }

    document.getElementById('test').innerHTML = keys;
}

// returns the value the key is mapped to
function getMap(key) {
  if (key == 'a') return 'left-rocket-top';
  if (key == 'j') return 'left-rocket-middle';
  if (key == 's') return 'left-rocket-bottom';
  if (key == 'b') return 'left-cargo-far';
  if (key == 'k') return 'left-cargo-mid';
  if (key == 't') return 'left-cargo-close';
  if (key == '2') return 'left-cargo-alliance';
  if (key == '3') return 'right-cargo-alliance';
  if (key == 'c') return 'right-cargo-far';
  if (key == 'l') return 'right-cargo-mid';
  if (key == 'u') return 'right-cargo-close';
  if (key == 'd') return 'right-rocket-top';
  if (key == 'm') return 'right-rocket-middle';
  if (key == 'v') return 'right-rocket-bottom';
  if (key == '!') return 'left-loading-cargo';
  if (key == '1') return 'left-loading-plate';
  if (key == '$') return 'right-loading-cargo';
  if (key == '4') return 'right-loading-plate';

  if (key == '9') return 'clear';
  if (key == '0') return 'backspace';
  if (key == 'w') return 'enter';

  return 'unmapped';
}

function highlight(toHighlight, color) {
  var location = document.getElementById(toHighlight);
  if (color == 'green') location.style.fill = `rgb(0,255,0)`;
  if (color == 'red') location.style.fill = `rgb(255,0,0)`;
  if (color == 'blue') location.style.fill = `rgb(0,0,255)`;
  if (color == 'clear') location.style.fill = `rgb(0,0,0)`;
}

function clearHighlight() {
  var len = allFieldLocations.length;
  for (var i = 0; i < len; i++) {
    var location = allFieldLocations[i];
    document.getElementById(location).style.fill = `rgb(0,0,0)`;
  }
}

function finalize() {
  highlight(getMap(keys[0]), 'green');
  highlight(getMap(keys[1]), 'red');

  var intakeStart = 'plate';
  if (getMap(keys[0]).includes('cargo')) {
    intakeStart = 'cargo';
  }

  var intakeEnd = 'bottom';
  if (getMap(keys[1]).includes('top')) {
    intakeEnd = 'top';
  } else if (getMap(keys[1]).includes('middle')) {
    intakeEnd = 'middle';
  }

  NetworkTables.putValue('/SmartDashboard/path/robot-start', keys[0]);
  NetworkTables.putValue('/SmartDashboard/path/robot-end', keys[1]);
  NetworkTables.putValue('/SmartDashboard/path/intake-start', intakeStart);
  NetworkTables.putValue('/SmartDashboard/path/intake-end', intakeEnd);
}
