var ota = (function(){

var pageCounter = 0;
var ota_retry_timeout;
var retry_counter = 0;
var curFwPageCrc = [];
var newFwPageCrc = [];
var newFwPageHex = [];

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
		ota_query_flash_crc(otaNode);
	});
	
	$("#btn_ota_parse_fw").click(function(){
		ota_parse_fw($("#firmware").val());
	});
	$("#btn_ota_program").click(function(){
	alert('b');
		ota_program();
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
			curFwPageCrc[flashPage]=crc32hex;
			var cellEl = $(".ota-flash-page-" + flashPage);
			cellEl.html(crc32hex);
			if (newFwPageCrc[flashPage]) {
				if (newFwPageCrc[flashPage] === curFwPageCrc[flashPage]) {
					cellEl.addClass("green");
				} else {
					cellEl.addClass("red");
				}
			}
			
			
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
			if (pageCounter < newFwPageCrc.length) {
				pageCounter++;
				var pageAddr = pageCounter*64;
				ota_query_flash_page_crc  (0x44, pageAddr);
				var retryCmd = "ota.query_flash_page_crc(0x44," + pageAddr +");"
					+ "ota.increment_retry_counter();"
					+ "console.log(\"retry on "
					+ pageAddr.toString(16) + "\");";
				ota_retry_timeout = setInterval(retryCmd,1000);
			}
		}
	});
}



function ota_query_flash_crc (node) {
	pageCounter = 0;
	ota_query_flash_page_crc(node, 0);
}

function ota_query_flash_page_crc (node, pageAddr) {
	var nodeHex = node.toString(16);
	var pageAddrHex = (0x10000 + pageAddr).toString(16).substr(-4);
	var radioCmd = "T " + nodeHex + " 92" + pageAddrHex;
	sendCommand(radioCmd);
}

function ota_parse_fw(fwHexDump) {
	flashPages = fwHexDump.match(/[^\r\n]+/g);
	flashPages.forEach(function(flashPage){
		var p = flashPage.split(" ");
		var pageAddr = parseInt(p[0],16);
		var page = pageAddr/64;
		newFwPageCrc[page] = p[2].toUpperCase();
		newFwPageHex[page] = p[1];
		console.log("newFwPageCrc[" + page + "]=" + newFwPageCrc[page]);
	});
}

function ota_program () {
	var programCommands = [];
	var i;
	// ignore page 0
	for (i = 1; i < newFwPageCrc.length; i++) {
		if (newFwPageCrc[i] !== curFwPageCrc[i]) {
			var cmd0 = "T 44 11" 
			+ hex16(i*64)
			+ newFwPageHex[i].substring(0,64);
			var cmd1 = "T 44 11"
			+ hex16(i*64 + 32)
			+ newFwPageHex[i].substring(64,128);
			console.log("program: " + cmd0);
			console.log("program: " + cmd1);
			programCommands.push(cmd0);
			programCommands.push(cmd1);
		}
	}
	
	i = 1;
	var ii = setInterval(function(){
		sendCommand(programCommands[i++]);
		if (i == programCommands.length) {
			clearInterval(ii);
		}
	},1000);
	
}

function hex16 (n) {
	return (0x10000 + n).toString(16).substr(-4);
}

}());