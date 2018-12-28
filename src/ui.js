// Define UI elements
var ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state'),
    cameraURL: 'http://10.0.0.231:8080/?action=stream',
    multiCamSRC: document.getElementById('multicam-src'),
    navx: {
      container: document.getElementById('navx'),
      circle: document.getElementById('navx-circle'),
      arm: document.getElementById('navx-arm'),
      number: document.getElementById('navx-number'),
      name: document.getElementById('navx-name')
    }
};

// position the text that says NavX
ui.navx.name.setAttribute('x', ui.navx.circle.getAttribute('r')/2);
ui.navx.name.setAttribute('y', ui.navx.name.getAttribute('x'));

NetworkTables.addKeyListener('/FRCDashboard/robotTime', (key, value) => {
  var minutes = ~~(value / 60); // converts to integer
  var seconds = (value - 60*minutes) % 60;
  seconds = (seconds < 10) ? '0'+seconds : seconds;
  ui.timer.style.color = `rgb(0, 200, 0)`;
  if (value < 135) ui.timer.style.color = `rgb(255, 255, 255)`;
  if (value < 30) ui.timer.style.color = `rgb(200, 0, 0)`;
  ui.timer.innerHTML = minutes + ':' + seconds;
});

NetworkTables.addKeyListener('/FRCDashboard/gyro', (key, value) => {
  var angle = value % 360;
  ui.navx.number.innerHTML = angle + 'º';
  ui.navx.arm.style.transform = `rotate(${angle}deg)`;
})
