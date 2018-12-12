// Define UI elements
var ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state'),
    cameraURL: 'http://10.0.0.231:8080/?action=stream',
    multiCamSRC: document.getElementById('multicam-src')
};

NetworkTables.addKeyListener('/FRCDashboard/robotTime', (key, value) => {
  ui.timer.innerHTML = value;
});
