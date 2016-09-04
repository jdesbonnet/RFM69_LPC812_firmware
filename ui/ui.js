// Icon used to represent a boat
var boatIcon = {
	url: 'circle-red.png',
	size: new google.maps.Size(16, 16),
	origin: new google.maps.Point(0,0),
	anchor: new google.maps.Point(8,8),
};

// Icon used to represent trail point
var trailIcon = {
    //url: 'trail.png',
    //size: new google.maps.Size(16, 16),
    //origin: new google.maps.Point(0,0),
    //anchor: new google.maps.Point(8,8)
    path: google.maps.SymbolPath.CIRCLE,
	scale: 1,
	strokeColor: "black",
};

var boatMarker;
var currentPosition;
var currentLatitude = 53.2814746;
var currentLongitude = -9.0602588;
var currentHeading;
var webSocket;
var gpsEmulationInterval;
var trailCounter = 0;
var numMessageRows = 0;
var maxMessageRows = 22;

var packetListeners = [];

function addPacketListener (listener) {
	packetListeners.push(listener);
}

function initializeMap() {

	centerPoint = new google.maps.LatLng(currentLatitude,currentLongitude);

	var mapOptions = {
		center: centerPoint,
		zoom: 18
	};

	map = new google.maps.Map(document.getElementById('mapContainer'),
		mapOptions);

	boatMarker = new google.maps.Marker({
		position: centerPoint,
		map: map,
		icon: {
			path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
			anchor: new google.maps.Point(0,4),
			scale: 4,
			strokeColor: "red",
			//rotation: 90
		}
	});


}

function formatTimestamp (date) {
	var hh = date.getUTCHours();
	var mm = date.getUTCMinutes();
	var ss = date.getSeconds();
	var frac_s = (date.getMilliseconds()/1000).toFixed(1);
	if (hh < 10) {hh = "0"+hh;}
	if (mm < 10) {mm = "0"+mm;}
	if (ss < 10) {ss = "0"+ss;}
	return hh+":"+mm+":"+ss;
}
function formatNMEATimestamp (date) {
	var hh = date.getUTCHours();
	var mm = date.getUTCMinutes();
	var ss = date.getSeconds();
	var frac_s = (date.getMilliseconds()/1000).toFixed(1);
	if (hh < 10) {hh = "0"+hh;}
	if (mm < 10) {mm = "0"+mm;}
	if (ss < 10) {ss = "0"+ss;}
	return hh + "" + mm + "" + ss;
}
function logEvent (message,style) {

	var newRow = $('<tr><td>'
			+ formatTimestamp(new Date())
			+ " " + message
			+'</td></tr>').hide();
	
	if (style) {
		newRow.addClass(style);
	}

	if (numMessageRows>=maxMessageRows) {
		jQuery('#message_table tr:last').remove();
		jQuery('#message_table tr:first').before(newRow);
		newRow.fadeIn(1000);	
    	} else {
		jQuery("#message_table tr:first").before(newRow);
		newRow.fadeIn(1000);
		numMessageRows++;
	}
}

function sendCommand (cmd) {
	logEvent(cmd,"outgoing");
	webSocket.send(cmd);
}

/**
 * Convert lat/lon in NMEA format (dddmm.mmmm) into decimal degress
 */
function nmeaToDecimalDegrees(nmeaLatLon) {
	var degmin = nmeaLatLon | 0;
	var min = degmin % 100;
	var deg = (degmin/100)|0;
	var fracmin = nmeaLatLon - degmin;
	return deg + (min+fracmin)/60;
}
function decimalDegreesToNmea(degrees) {
	var degint = degrees|0;
	var fracdeg = degrees - degint;
	var mins = fracdeg * 60;
	return (degint*100 + mins);
}

/**
 * Update position marker on map.
 */
function updateMap(newLatitude, newLongitude) {

	trailCounter++;
	
	var oldPos = new google.maps.LatLng(currentLatitude,currentLongitude);
	var newPos = new google.maps.LatLng(newLatitude,newLongitude);
	boatMarker.setPosition(newPos);
	
	// Insert trail marker if enabled. Alternate black/white trail marker to enable
	// visibility on dark satellite views as well as map views.
	if ($("#trail_cb").is(":checked")) {
			trailIcon.strokeColor = (trailCounter % 2 == 0 ? "black" : "white")
			var trailMarker = new google.maps.Marker({
			position: oldPos,
			map: map,
			icon: trailIcon,
		});
	}
	
	$("#latitude").val(newLatitude.toFixed(7));
	$("#longitude").val(newLongitude.toFixed(7));
	
	// If 'center map' checkbox checked, center on marker
	if ($("#center_map_cb").is(":checked")) {
		map.setCenter (newPos);
	}
	
	currentLatitude = newLatitude;
	currentLongitude = newLongitude;
}

function updateHeading (newHeading) {

	if (newHeading == currentHeading) {
		return;
	}
	
	$("#heading").val(newHeading);
	
	/*
	boatMarker.setMap(null);
	var curPos = new google.maps.LatLng(currentLatitude,currentLongitude);
	boatMarker = new google.maps.Marker({
		position: curPos,
		map: map,
		icon: {
			path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
			anchor: new google.maps.Point(0,4),
			scale: 4,
			strokeColor: "red",
			rotation: (newHeading|0)
		}
	});
	*/
	boatMarker.setIcon (
	 {
			path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
			anchor: new google.maps.Point(0,4),
			scale: 4,
			strokeColor: "red",
			rotation: (newHeading|0)
		}
	);
	currentHeading = newHeading;
}

function updateSpeed (newSpeed) {
	$("#speed").val(newSpeed);
}

/**
 * Called from within a setInterval() function.
 */
function gpsEmulator () {

	// Return if emulation disabled
	if ($("#gps_emulator_enable").val()==0) {
		return;
	}

	var heading = $("#heading").val();
	var speed = $("#speed").val();
	var updatePeriod = $("#update_period").val()*1;

	// Given current position, heading and speed, calculate new position
	var curPos = new google.maps.LatLng(currentLatitude,currentLongitude);
	var newPos = google.maps.geometry.spherical.computeOffset(curPos, speed*updatePeriod, heading);

	updateMap(newPos.lat(), newPos.lng());

	var setPosCmd = "G "
			+ formatNMEATimestamp(new Date()) + " " 
			+ decimalDegreesToNmea(currentLatitude).toFixed(5) + " " 
			+ decimalDegreesToNmea(currentLongitude).toFixed(5) + " " 
			+ heading + " " + speed;
			
	// Send to local or remote radio depending on emulation mode
	if ($("#gps_emulator_enable").val()==1) {
		// Send positions to device on UART
		webSocket.send(setPosCmd);
	} else if  ($("#gps_emulator_enable").val()==2) {
		// Send position over air to remote radio
		// D <to-addr> <remote-cmd>
		webSocket.send("D " + $("#to_addr").val() + " " + setPosCmd);
	}
}


jQuery(function() {
	initializeMap();


	$("#heading_dec").click(function(){
		var heading = $("#heading").val() | 0;
		heading -= 10;
		heading %= 360;
		updateHeading(heading);
		//$("#heading").val(heading);
	});
	$("#heading_inc").click(function(){
		var heading = $("#heading").val() | 0;
		heading += 10;
		heading %= 360;
		updateHeading(heading);
		//$("#heading").val(heading);
	});
	$("#speed_dec").click(function(){
		var speed = $("#speed").val() * 1;
		speed -= 0.1;
		if (speed<0) speed = 0;
		$("#speed").val(speed.toFixed(1));
	});
	$("#speed_inc").click(function(){
		var speed = $("#speed").val() * 1;
		speed += 0.1;
		$("#speed").val(speed.toFixed(1));
	});
	
	$("#update_period").change(function(){
		clearInterval(gpsEmulationInterval);
		gpsEmulationInterval = setInterval(gpsEmulator, $("#update_period").val()*1000);
	});

	$("#ws_connect_btn").click(function(){
	
		// If already open, then close it.
		if (webSocket && webSocket.readyState==1) {
			webSocket.close();
			return;
		}
		
		var ws_url = $("#ws_url").val();
		logEvent("connecting to " + ws_url);
		webSocket = new WebSocket(ws_url);
		webSocket.onopen=function(){
			$("#websocket_indicator").removeClass("indicator-off");
			$("#websocket_indicator").addClass("indicator-on");
			logEvent("connection open");
		};
		webSocket.onclose=function(){
			$("#websocket_indicator").removeClass("indicator-on");
			$("#websocket_indicator").addClass("indicator-off");
			logEvent("connection closed");
		};
		webSocket.onerror=function(){
			alert('error: ' + error);
		};
		webSocket.onmessage=function(event){
			logEvent(event.data,"incoming");
			var p = event.data.split(" ");
			if (p[0] === 'g' && $("#gps_emulator_enable").val()==0) {
				updateMap(nmeaToDecimalDegrees(p[3]),nmeaToDecimalDegrees(p[4]));
				updateHeading(p[5]);
				updateSpeed(p[6]);
			}
			packetListeners.forEach(function(listener){
				listener(event.data);
			});
		};

	});

	$("#radio_cmd_send_btn").click(function(){
		sendCommand($("#radio_cmd").val());
	});
	$("#freq_set_btn").click(function(){
		var fMHz = $("#frequency").val();
		var frfReg = ((fMHz*1000000/61)|0)&0xffffff;
		var frfMsb = frfReg>>16;
		var frfMid = (frfReg>>8)&0xff;
		var frfLsb = frfReg&0xff;
		// Write to FRF register to set center frequency
		sendCommand("W 7 " 
			+ frfMsb.toString(16) + " " 
			+ frfMid.toString(16) + " " 
			+ frfLsb.toString(16)
		);
	});
	$("#radio_bps").change(function(){
		var bps = $("#radio_bps").val();
		// bitrate_reg = FXO(32MHz)/bps
		var bitrate_reg = ((32000000/bps)|0)&0xffff;
		var bitrate_msb = bitrate_reg>>8;
		var bitrate_lsb = bitrate_reg&0xff;
		sendCommand("W 3 "
			+ bitrate_msb.toString(16) + " "
			+ bitrate_lsb.toString(16)
		);
	});

	$("#radio_cmd").keydown(function(e){
		if (e.which == 13) {
			sendCommand($("#radio_cmd").val());
		}
	});
	$("#btn_reset_board").click(function(){
		sendCommand("Q");
	});

	$(".cmd_btn").click(function(){
		sendCommand($(this).data("cmd"));
	});

	gpsEmulationInterval = setInterval (gpsEmulator,1000);
	
	ota.init();
});