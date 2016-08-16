import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.util.ArrayList;

public class WakeNode {

	public static void main (String[] arg) throws Exception {


		String node = "46";

		File uartDevice = new File(arg[0]);

		BufferedReader in = new BufferedReader(new FileReader(uartDevice));
		FileWriter out = new FileWriter(uartDevice);

		String line;

		while ( (line = in.readLine()) != null) {
			System.out.println ("rx: " + line);
			if (line.startsWith("p FF " + node + " 7A")) {
				// Remote "M 0" command
				//String cmd = "T " + node + " 444D2030";
				String cmd = "F " + node;
				System.out.println ("tx: " + cmd);
				out.write(cmd + "\n");
				out.flush();
			}

		}
	}
}

