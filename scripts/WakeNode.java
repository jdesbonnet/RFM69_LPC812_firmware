import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.util.Date;
import java.util.ArrayList;
import java.text.SimpleDateFormat;

public class WakeNode {

	private static SimpleDateFormat df = new SimpleDateFormat("yyyyMMdd-HHmmss");

	public static void main (String[] arg) throws Exception {


		int[] frameCount = new int[256];

		File uartDevice = new File(arg[0]);

		BufferedReader in = new BufferedReader(new FileReader(uartDevice));
		FileWriter out = new FileWriter(uartDevice);


		String line;

		int i = 0;
		int node;  


		while ( (line = in.readLine()) != null) {

			Date timestamp = new Date();

			System.out.println ("rx: " + line);

			if (line.startsWith("p FF ")) {
				node = Integer.parseInt(line.substring(2,4),16);
				frameCount[node]++;
			} else {
				continue;
			}

			if (line.startsWith("p FF 44 7A")) {
				i++;
				//String cmd = "T 44 53";
				String cmd = "F 44";
				System.out.println ("tx: " + cmd);
				out.write(cmd + "\n");
				out.flush();


				String eventCountHex = line.substring(12,14);
				String batteryHex = line.substring(14,16);
				String temperatureHex = line.substring(16,20);
				int eventCount = Integer.parseInt(eventCountHex,16);
				int battery = Integer.parseInt(batteryHex,16);
				int temperature = Integer.parseInt(temperatureHex,16);
				float temperatureC = ((float)temperature)/16;
				float batteryV = ((float)battery)/10;
	
				System.out.println ( df.format(timestamp) + " " + eventCount + " " + 
					+ temperatureC + " " + batteryV);
			}

			if (line.startsWith("p FF 46 7A")) {
				String cmd = "F 46";
				System.out.println ("tx: " + cmd);
				out.write(cmd + "\n");
				out.flush();
			}

                        if (line.startsWith("p FF 41 7A")) {
                                //String cmd = "F 41";
                                //System.out.println ("tx: " + cmd);
                                //out.write(cmd + "\n");
                                //out.flush();
                        }

			for (i = 0; i < 256; i++) {
				if (frameCount[i]>0) {
					System.out.println ("node[" + String.format("%x",node) + "]=" + frameCount[node]);
				}
			}


		}
	}
}

