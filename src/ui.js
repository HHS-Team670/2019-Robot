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

let allFieldLocations = ["left-rocket-top", "left-rocket-middle", "left-rocket-bottom", "left-cargo-far", "left-cargo-mid", "left-cargo-close", "left-cargo-alliance", "right-cargo-alliance", "right-cargo-far", "right-cargo-mid", "right-cargo-close", "right-rocket-top", "right-rocket-middle", "right-rocket-bottom", "right-loading-station", "left-loading-station"];

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
      keys.pop();
    } else {
      keys.push(key);
    }

    document.getElementById('test').innerHTML = keys;


    // if (key == 'a') {
    //   highlight('left-rocket-top');
    // }
    // if (key == 'j') {
    //   highlight('left-rocket-middle');
    // }
    // if (key == 's') {
    //   highlight('left-rocket-bottom');
    // }
    // if (key == 'b') {
    //   highlight('left-cargo-far');
    // }
    // if (key == 'k') {
    //   highlight('left-cargo-mid');
    // }
    // if (key == 't') {
    //   highlight('left-cargo-close');
    // }
    // if (key == '2') {
    //   highlight('left-cargo-alliance');
    // }
    // if (key == '3') {
    //   highlight('right-cargo-alliance');
    // }
    // if (key == 'c') {
    //   highlight('right-cargo-far');
    // }
    // if (key == 'l') {
    //   highlight('right-cargo-mid');
    // }
    // if (key == 'u') {
    //   highlight('right-cargo-close');
    // }
    // if (key == 'd') {
    //   highlight('right-rocket-top');
    // }
    // if (key == 'm') {
    //   highlight('right-rocket-middle');
    // }
    // if (key == 'v') {
    //   highlight('right-rocket-bottom');
    // }
    // if (key == '1') {
    //   highlight('left-loading-station');
    // }
    // if (key == '4') {
    //   highlight('right-loading-station');
    // }
    //
    // if (key == '9') {
    //   clearHighlight();
    // }
}

function highlight(toHighlight) {
  var location = document.getElementById(toHighlight);
  location.style.fill = `rgb(0,255,0)`;
}

function clearHighlight() {
  var len = allFieldLocations.length;
  for (var i = 0; i < len; i++) {
    var location = allFieldLocations[i];
    document.getElementById(location).style.fill = `rgb(0,0,0)`;
  }
}
