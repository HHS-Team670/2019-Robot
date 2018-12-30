// Define UI elements
var ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state'),
    cameraURL: 'http://10.0.0.231:8080/?action=stream', // can be changed
    multiCamSRC: document.getElementById('multicam-src'),
    navx: {
      container: document.getElementById('navx'),
      circle: document.getElementById('navx-circle'),
      arm: document.getElementById('navx-arm'),
      number: document.getElementById('navx-number'),
      name: document.getElementById('navx-name'),
      toptick: document.getElementById('navx-toptick'),
      righttick: document.getElementById('navx-righttick'),
      bottomtick: document.getElementById('navx-bottomtick'),
      lefttick: document.getElementById('navx-lefttick')
    },
    turret: {
      container: document.getElementById('turret'),
      circle: document.getElementById('turret-circle'),
      arm: document.getElementById('turret-arm'),
      number: document.getElementById('turret-number'),
      name: document.getElementById('turret-name'),
      toptick: document.getElementById('turret-toptick'),
      righttick: document.getElementById('turret-righttick'),
      bottomtick: document.getElementById('turret-bottomtick'),
      lefttick: document.getElementById('turret-lefttick')
    }
};

// position NavX
var navxR = ui.navx.circle.getAttribute('r'); // radius of circle showing navx
ui.navx.container.setAttribute('width', 2*navxR);
ui.navx.container.setAttribute('height', 2*navxR);
ui.navx.name.setAttribute('x', ((navxR > 50) ? navxR-25 : navxR/2));
ui.navx.name.setAttribute('y', navxR/2);
ui.navx.circle.setAttribute('cx', navxR);
ui.navx.circle.setAttribute('cy', navxR);
ui.navx.toptick.setAttribute('x', navxR);
ui.navx.toptick.setAttribute('y', 0);
ui.navx.righttick.setAttribute('x', 2*navxR);
ui.navx.righttick.setAttribute('y', navxR);
ui.navx.bottomtick.setAttribute('x', navxR);
ui.navx.bottomtick.setAttribute('y', 2*navxR);
ui.navx.lefttick.setAttribute('x', 0);
ui.navx.lefttick.setAttribute('y', navxR);
ui.navx.arm.setAttribute('x', navxR);
ui.navx.arm.setAttribute('height', navxR);
ui.navx.number.setAttribute('x', '50%');
ui.navx.number.setAttribute('y', '50%');
ui.navx.number.setAttribute('width', 0);
ui.navx.number.setAttribute('style', 'left:50%;transform:translate(-50%,0)');

// position Turret
var turretR = ui.turret.circle.getAttribute('r'); // radius of circle showing turret
ui.turret.container.setAttribute('width', 2*turretR);
ui.turret.container.setAttribute('height', 2*turretR);
ui.turret.name.setAttribute('x', ((turretR > 50) ? turretR-25 : turretR/2));
ui.turret.name.setAttribute('y', turretR/2);
ui.turret.circle.setAttribute('cx', turretR);
ui.turret.circle.setAttribute('cy', turretR);
ui.turret.toptick.setAttribute('x', turretR);
ui.turret.toptick.setAttribute('y', 0);
ui.turret.righttick.setAttribute('x', 2*turretR);
ui.turret.righttick.setAttribute('y', turretR);
ui.turret.bottomtick.setAttribute('x', turretR);
ui.turret.bottomtick.setAttribute('y', 2*turretR);
ui.turret.lefttick.setAttribute('x', 0);
ui.turret.lefttick.setAttribute('y', turretR);
ui.turret.arm.setAttribute('x', turretR);
ui.turret.arm.setAttribute('height', turretR);
ui.turret.number.setAttribute('x', '50%');
ui.turret.number.setAttribute('y', '50%');
ui.turret.number.setAttribute('width', 0);
ui.turret.number.setAttribute('style', 'left:50%;transform:translate(-50%,0)');

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
})
