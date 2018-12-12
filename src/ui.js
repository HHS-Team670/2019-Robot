// Define UI elements
var ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state'),
    key: document.getElementById('key'),
    cameraURL: 'http://10.0.0.231:8080/?action=stream',
    multiCamID: document.getElementById('multicam-id'),
    multiCamSRC: document.getElementById('multicam-src')
};

NetworkTables.addKeyListener('/FRCDashboard/robotTime', (key, value) => {
  ui.timer.innerHTML = value;
});

NetworkTables.addKeyListener('/CameraPublisher/MustangCam/streams', (key, value) => {
  // ui.cameraURL = value;
  ui.key.innerHTML = value;
});
