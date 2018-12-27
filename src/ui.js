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
  ui.timer.innerHTML = value;
});

NetworkTables.addKeyListener('/FRCDashboard/gyro', (key, value) => {
  var angle = value % 360;
  ui.navx.number.innerHTML = angle + 'º';
  ui.navx.arm.style.transform = `rotate(${angle}deg)`
})
