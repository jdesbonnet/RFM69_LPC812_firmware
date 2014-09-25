import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
public class QueryNode {

	private static String poke (int addr, int value) {
		String addrHex = String.format("%08x", addr);
		String valueHex = String.format("%08x", value);
		return "T FF 3E" + addrHex + valueHex;
	}

	private static String peek (int addr) {
		String addrHex = String.format("%08x", addr);
		return "T FF 3C" + addrHex;
	}

	public static void main (String[] arg) throws Exception {


		BufferedReader in = new BufferedReader(new FileReader(arg[0]));

		FileWriter out = new FileWriter(arg[0]);

		//String remoteCmd0 = "T FF 594E08";


		//String remoteCmd0 = "T FF 3C4000800C08"; // Read MCU memory 0x4000800C
		//String remoteCmd0 = "T FF 3C0c800040"; // Read MCU memory 0x4000800C
		//String remoteCmd0 = "T FF 3C04000240"; // Read MCU memory 0x40020004 GPREG0
		String remoteCmd0 = "T FF 3C00010010"; // Read MCU memory 0x10000100 flags

		//String remoteCmd1 = "T FF 583E01"; // Read temp stored in AesKey1
		//String remoteCmd1 = "T FF 3E0c80004011223344"; // Read MCU memory 0x4000800C
		//String remoteCmd1 = "T FF 3E0400024055227744"; // Read MCU memory 0x4000800C
		String remoteCmd1 = "T FF 3E0001001002040000"; // Read MCU memory 0x4000800C

		//String remoteCmd1 = "T FF 581101"; // Read PALevel
		//String remoteCmd1 = "T FF 59119F"; // Write low power to PALevel
		//String remoteCmd1 = "T FF 4456"; // Remote 'V' command
		//String remoteCmd1 = "T FF 444620343032"; // Remote 'F 402' (low power poll)
		//String remoteCmd1 = "T FF 44462032"; // Remote 'F 2' (low power poll)
		String line;

		int i = 0; 

		while ( (line = in.readLine()) != null) {

			System.out.print (" < ");
			System.out.println (line);

			String[] p = line.split(" ");

			if (p.length < 4 ) {
				continue;
			}

			String fromAddr = p[2];
			String msgType = p[3];
			if (msgType.startsWith("7A")) {
				// Send query to read Temp1, Temp2
				if (i%2 == 0) {
					out.write (remoteCmd0 + "\r");
					System.out.println (" > " + remoteCmd0);
				} else {
					out.write (remoteCmd1 + "\r");
					System.out.println (" > " + remoteCmd1);
				}

				out.flush();
				i++;

			}


		}
	}
}

