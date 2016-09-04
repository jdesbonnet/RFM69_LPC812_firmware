var ota = (function(){

var pageCounter = 0;
var pageCrc = [];
var ota_retry_timeout;
var retry_counter = 0;

return {
	init: function() {
		ota_init();
	},
	query_flash_page_crc: function(node,pageAddr){
		ota_query_flash_page_crc(node,pageAddr);
	},
	increment_retry_counter: function() {
		retry_counter++;
	}
};

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
			rowEl.append("<td class='ota-flash-page-" + (j*numCols+i) + "'>" + pageAddr.toString(16) + "</td>");
		}
		$("#crcTable").append(rowEl);
	}
	
	$("#btn_ota_query_crc").click(function(){
		var otaNode = parseInt($("#ota_node").val());
		ota_query_crc(otaNode);
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
			if (retry_counter>0) {
				$(".ota-flash-page-" + flashPage).addClass("retry");
			}
			console.log ("page=" + flashPageAddrHex  + " crc=" + crc32hex);
			
			// Kill retry timeout
			if (ota_retry_timeout) {
				clearInterval(ota_retry_timeout);
				retry_counter = 0;
			}
			
			// Get next page
			if (pageCounter < 255) {
				pageCounter++;
				var pageAddr = pageCounter*64;
				ota_query_flash_page_crc  (0x40, pageAddr);
				var retryCmd = "ota.query_flash_page_crc(0x40," + pageAddr +");"
					+ "ota.increment_retry_counter();"
					+ "console.log(\"retry on "
					+ pageAddr.toString(16) + "\");";
				ota_retry_timeout = setInterval(retryCmd,1000);
			}
		}
	});
}



function ota_query_crc (node) {
	pageCounter = 0;
	ota_query_flash_page_crc(node, 0);
}

function ota_query_flash_page_crc (node, pageAddr) {
	var nodeHex = node.toString(16);
	var pageAddrHex = (0x10000 + pageAddr).toString(16).substr(-4);
	var radioCmd = "T " + nodeHex + " 92" + pageAddrHex;
	sendCommand(radioCmd);
}

}());