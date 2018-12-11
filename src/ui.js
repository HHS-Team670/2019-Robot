// Define UI elements
let ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state').firstChild,
    gyro: {
        container: document.getElementById('gyro'),
        val: 0,
        offset: 0,
        visualVal: 0,
        arm: document.getElementById('gyro-arm'),
        number: document.getElementById('gyro-number')
    },
    robotDiagram: {
        arm: document.getElementById('robot-arm'),
        team: document.getElementById('team')
    },
    example: {
        button: document.getElementById('example-button'),
        readout: document.getElementById('example-readout').firstChild
    },
    autoSelect: document.getElementById('auto-select'),
    armPosition: document.getElementById('arm-position'),
    testKey: document.getElementById('key')
};

NetworkTables.addRobotConnectionListener((connected) => {
  console.log("robot connected: " + connected);
})

NetworkTables.addKeyListener('/FRCDashboard/robotTime', (key, value) => {
  ui.testKey.innerHTML = "got: " + key;
  ui.timer.innerHTML = value;
});
// 
// NetworkTables.getKeys();
