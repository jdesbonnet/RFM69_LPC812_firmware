var pageCounter = 0;
var ota_crc_query_interval;
var pageCrc = [];
var ota_retry_timeout;

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
			
			// Kill retry timeout
			if (ota_retry_timeout) {
				clearInterval(ota_retry_timeout);
			}
			
			// Get next page
			if (pageCounter < 256) {
				var pageAddr = pageCounter*64;
				ota_query_flash_page_crc  (0x40, pageAddr);
				var retryCmd = "ota_query_flash_page_crc(0x40," + pageAddr +");"
					+ "console.log(\"retry on "
					+ pageAddr + "\");";
				//console.log("retryCmd=" + retryCmd);
				ota_retry_timeout = setInterval(retryCmd,1000);
				pageCounter++;
			}
		}
	});
}



function ota_query_crc (node) {	
	ota_query_flash_page_crc(node, pageCounter*64);
}

function ota_query_flash_page_crc (node, pageAddr) {
	var nodeHex = node.toString(16);
	var pageAddrHex = (0x10000 + pageAddr).toString(16).substr(-4); 
	sendCommand("T " + nodeHex + " 92" + pageAddrHex);
}