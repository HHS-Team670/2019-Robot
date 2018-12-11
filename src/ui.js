// Define UI elements
let ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state'),
    testKey: document.getElementById('key')
};

NetworkTables.addKeyListener('/FRCDashboard/robotTime', (key, value) => {
  ui.testKey.innerHTML = "got: " + key;
  ui.timer.innerHTML = value;
});
