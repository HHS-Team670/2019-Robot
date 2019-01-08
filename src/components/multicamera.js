// Set a global alias for the camera and related elements.
ui.camera = {
	viewer: document.getElementById('camera'),
	id: 0,
	srcs: ['http://' + address.value + ':8000/?action=stream',
		'http://' + address.value + ':8001/?action=stream']
};

// document.getElementById('test').innerHTML = '>>>' + address.value;

ui.multiCamSRC.innerHTML = ui.camera.srcs[ui.camera.id];
ui.camera.viewer.style.backgroundImage = 'url(' + ui.camera.srcs[ui.camera.id] + ')';

// When camera is clicked on, change to the next source.
ui.camera.viewer.onclick = function() {
	ui.camera.id += 1;
	if (ui.camera.id === ui.camera.srcs.length) ui.camera.id = 0;
  ui.multiCamSRC.innerHTML = ui.camera.srcs[ui.camera.id];
	ui.camera.viewer.style.backgroundImage = 'url(' + ui.camera.srcs[ui.camera.id] + ')';
	NetworkTables.putValue('/SmartDashboard/cameraSource', ui.camera.id); // TODO implement listener on Pi
};
