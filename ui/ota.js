function ota_init() {
	var i,j;
	var pageSize = 64;
	var numPages = 256;
	var numCols = 8;
	var numRows = numPages / numCols;
	$("#crcTable").empty();
	for (j = 0; j < numRows; j++) {
		var rowBaseAddr = j*numCols*pageSize;
		var rowEl = $("<tr class='row-'" + j + "'></tr>");
		rowEl.append("<th>" + rowBaseAddr.toString(16) + "</th>");
		for (i = 0; i < numCols; i++) {
			var pageAddr = (j*numCols+i)*pageSize;
			rowEl.append("<td class='ota-flash-page-" + (j*numCols+i) + "'>" + (j*numCols+i) + "</td>");
		}
		$("#crcTable").append(rowEl);
	}
	
	$("#btn_ota_query_crc").click(function(){
		pageCounter = 1;
		ota_query_crc(0x40);
	});
	
	addPacketListener(function(packet){
		var p = packet.split(" ");
		if (p.length<4) {
			return;
		}
		var payload = p[3];
		if (payload.indexOf("13")===0) {
			var flashPageAddrHex = payload.substring(2,6);
			var crc32hex = payload.substring(6,14);
			var flashPageAddr = parseInt(flashPageAddrHex,16);
			var flashPage = flashPageAddr/64;
			pageCrc[flashPage]=crc32hex;
			$(".ota-flash-page-" + flashPage).html(crc32hex).addClass("green");
			console.log ("page=" + flashPageAddrHex  + " crc=" + crc32hex);
		}
	});
}

var pageCounter = 0;
var ota_crc_query_interval;
var pageCrc = [];

function ota_query_crc (node) {
	var i=0;
	ota_crc_query_interval = setInterval(function(){
		if ( ! pageCrc[pageCounter] ) {
			ota_query_flash_page_crc(0x40, (pageCounter++)*64);
		}
		if (pageCounter===256) {
			clearInterval(ota_crc_query_interval);
		}
	},400);
}
function ota_query_flash_page_crc (node, pageAddr) {
	var nodeHex = node.toString(16);
	var pageAddrHex = (0x10000 + pageAddr).toString(16).substr(-4); 
	sendCommand("T " + nodeHex + " 92" + pageAddrHex);
}