// Set a global alias for the camera and related elements.
ui.camera = {
	viewer: document.getElementById('camera'),
	src: 'http://' + address.value + ':80/?action=stream'
};

ui.multiCamSRC.innerHTML = ui.camera.src;
ui.camera.viewer.style.backgroundImage = 'url(' + ui.camera.src + ')';

// When camera is clicked on, send a value over network tables.
ui.camera.viewer.onclick = function() {
	NetworkTables.putValue('/SmartDashboard/cameraSource', 'next');
};
