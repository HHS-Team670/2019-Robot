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

// naming convention starts from the midfield line and counts towards the alliance station
let allEndpoints = [
  "left-rocket-middle-1", "left-rocket-middle-2", "left-rocket-middle-3",
  "left-rocket-bottom-1", "left-rocket-bottom-2", "left-rocket-bottom-3",
  "right-rocket-middle-1", "right-rocket-middle-2", "right-rocket-middle-3",
  "right-rocket-bottom-1", "right-rocket-bottom-2", "right-rocket-bottom-3",
  "left-cargo-ball-1", "left-cargo-plate-1",
  "left-cargo-ball-2", "left-cargo-plate-2",
  "left-cargo-ball-3", "left-cargo-plate-3",
  "left-cargo-ball-4", "left-cargo-plate-4",
  "right-cargo-ball-1", "right-cargo-plate-1",
  "right-cargo-ball-2", "right-cargo-plate-2",
  "right-cargo-ball-3", "right-cargo-plate-3",
  "right-cargo-ball-4", "right-cargo-plate-4",
  "left-loading-ball", "left-loading-plate",
  "right-loading-ball", "right-loading-plate",
  "left-depot", "right-depot",
  "hab-level-1", "left-hab-level-2", "right-hab-level-2", "hab-level-3"
];
var endpoints = [];

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

NetworkTables.addKeyListener('/SmartDashboard/cameraSource', (key, value) => {
  if (value == 'next') {
    window.webContents.reload();
    // location.reload();
  }
});

var keys = [];

window.onkeyup = function(event) {
    let key = event.key;

    if (key == '9') {
      keys.pop();
    } else {
      if (keys.length % 2 == 1 && key == 'w') finalize();
      if (keys.length < 4) {
        if (keys.length % 2 == 0 && getRobotLocation(key) != null) keys.push(key);
        else if (keys.length % 2 == 1 && getHeight(key) != null) keys.push(key);
      }

      // document.getElementById(getRobotLocation(keys[keys.length-2])).innerHTML = '*';
      // (getHeight(key).split('-')[1].charAt(0).toUpperCase());
    }

    document.getElementById('test').innerHTML = keys;

    endpoints = [];
    if (keys.length > 0) {
      highlight(getRobotLocation(keys[0]), 'white');
    }
    if (keys.length > 1) {
      var endpoint = getEndpoint(getRobotLocation(keys[0]), getHeight(keys[1]));
      if (endpoint == null) keys.pop();
      else {
        var location = getRobotLocation(keys[0]);
        highlight(location, 'green');
        endpoints.push(endpoint);
        var startText = document.getElementById('start-text');
        positionText(startText, location);
        startText.innerHTML = getHeight(keys[1]).split('-')[1].charAt(0).toUpperCase();
      }
    }
    if (keys.length > 2) {
      highlight(getRobotLocation(keys[2]), 'white');
    }
    if (keys.length > 3) {
      var endpoint = getEndpoint(getRobotLocation(keys[2]), getHeight(keys[3]));
      if (endpoint == null) keys.pop();
      else {
        var location = getRobotLocation(keys[2]);
        highlight(location, 'red');
        endpoints.push(endpoint);
        var endText = document.getElementById('end-text');
        positionText(endText, location);
        endText.innerHTML = getHeight(keys[3]).split('-')[1].charAt(0).toUpperCase();
      }
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
function getRobotLocation(key) {
  if (key == 'a') return 'left-rocket-1';
  if (key == 'j') return 'left-rocket-2';
  if (key == 's') return 'left-rocket-3';
  if (key == '1') return 'left-loading';
  if (key == '0') return 'left-depot';
  if (key == 'b') return 'left-cargo-1';
  if (key == 'k') return 'left-cargo-2';
  if (key == 't') return 'left-cargo-3';
  if (key == '2') return 'left-cargo-4';
  if (key == 'c') return 'right-cargo-1';
  if (key == 'l') return 'right-cargo-2';
  if (key == 'u') return 'right-cargo-3';
  if (key == '3') return 'right-cargo-4';
  if (key == 'd') return 'right-rocket-1';
  if (key == 'm') return 'right-rocket-2';
  if (key == 'v') return 'right-rocket-3';
  if (key == '4') return 'right-loading';
  if (key == 'w') return 'right-depot';
  // return null;
}

// returns the arm height corresponding with a given key
function getHeight(key) {
  if (key == 'a') return 'rocket-middle';
  if (key == 'b') return 'rocket-bottom';
  if (key == 'j') return 'cargo-ball';
  if (key == 'k') return 'cargo-plate';
  if (key == 's') return 'loading-ball';
  if (key == 't') return 'loading-plate';
  // return null;
}

// combines location and height to return an endpoint
function getEndpoint(location, height) {
  if (location.includes('rocket') && height.includes('rocket')) {
    return location.replace('rocket', height);
  }
  if (location.includes('cargo') && height.includes('cargo')) {
    return location.replace('cargo', height);
  }
  if (location.includes('loading') && height.includes('loading')) {
    return location.replace('loading', height);
  }
  if (location.includes('depot')) {
    return location;
  }
  // return null;
}

function highlight(toHighlight, color) {
  var endpoint = document.getElementById(toHighlight);
  if (color == 'green') endpoint.style.fill = `rgb(0,255,0)`;
  if (color == 'red') endpoint.style.fill = `rgb(255,0,0)`;
  if (color == 'blue') endpoint.style.fill = `rgb(0,0,255)`;
  if (color == 'white') endpoint.style.fill = `rgb(255,255,255)`;
  if (color == 'clear') endpoint.style.fill = `rgb(0,0,0)`;
}

function clearHighlight() {
  var len = allEndpoints.length;
  for (var i = 0; i < len; i++) {
    var endpoint = allEndpoints[i];
    document.getElementById(endpoint).style.fill = `rgb(0,0,0)`;
  }
}

function positionText(text, position) {
  if (position === 'left-rocket-1') text.style.transform = `translate(15px, 20px)`;
  if (position === 'left-rocket-2') text.style.transform = `translate(15px, 50px)`;
  if (position === 'left-rocket-3') text.style.transform = `translate(15px, 80px)`;
  if (position === 'left-cargo-1') text.style.transform = `translate(120px, 20px)`;
  if (position === 'left-cargo-2') text.style.transform = `translate(120px, 50px)`;
  if (position === 'left-cargo-3') text.style.transform = `translate(120px, 80px)`;
  if (position === 'left-cargo-4') text.style.transform = `translate(130px, 110px)`;
  if (position === 'left-loading') text.style.transform = `translate(15px, 280px)`;
  if (position === 'left-depot') text.style.transform = `translate(65px, 280px)`;
  if (position === 'right-rocket-1') text.style.transform = `translate(275px, 20px)`;
  if (position === 'right-rocket-2') text.style.transform = `translate(275px, 50px)`;
  if (position === 'right-rocket-3') text.style.transform = `translate(275px, 80px)`;
  if (position === 'right-cargo-1') text.style.transform = `translate(170px, 20px)`;
  if (position === 'right-cargo-2') text.style.transform = `translate(170px, 50px)`;
  if (position === 'right-cargo-3') text.style.transform = `translate(170px, 80px)`;
  if (position === 'right-cargo-4') text.style.transform = `translate(160px, 110px)`;
  if (position === 'right-loading') text.style.transform = `translate(275px, 280px)`;
  if (position === 'right-depot') text.style.transform = `translate(225px, 280px)`;
}

function finalize() {
  document.getElementById('test').innerHTML = endpoints.length;
  NetworkTables.putValue('/SmartDashboard/endpoints', endpoints);
  keys = [];

  // highlight(getMap(keys[0]), 'green');
  // highlight(getMap(keys[1]), 'red');
  //
  // var armStart = 'plate';
  // if (getMap(keys[0]).includes('cargo')) {
  //   armStart = 'cargo';
  // }
  //
  // var armEnd = 'bottom';
  // if (getMap(keys[1]).includes('top')) {
  //   armEnd = 'top';
  // } else if (getMap(keys[1]).includes('middle')) {
  //   armEnd = 'middle';
  // }

  NetworkTables.putValue('/SmartDashboard/path/robot-start', keys[0]);
  NetworkTables.putValue('/SmartDashboard/path/robot-end', keys[1]);
  NetworkTables.putValue('/SmartDashboard/path/arm-start', armStart);
  NetworkTables.putValue('/SmartDashboard/path/arm-end', armEnd);
}
