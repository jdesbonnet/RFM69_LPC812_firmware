import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.util.ArrayList;

public class QueryNode {

	private static String poke (int addr, int value) {
		String addrHex = String.format("%08x", Integer.reverseBytes(addr));
		String valueHex = String.format("%08x", Integer.reverseBytes(value));
		return "T FF 3E" + addrHex + valueHex;
	}

	private static String peek (int addr) {
		String addrHex = String.format("%08x", Integer.reverseBytes(addr));
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

		ArrayList<String> commands = new ArrayList<String>();

		commands.add(peek(0x10000100));
		commands.add(peek(0x10000100));
		commands.add(peek(0x10000100));
		commands.add(peek(0x10000104));
		commands.add(peek(0x10000108));
		commands.add(peek(0x1000010C));

		String line;

		int i = 0; 

		while ( (line = in.readLine()) != null) {

			System.out.print (" < ");
			System.out.println (line);
			//System.out.flush();

			String[] p = line.split(" ");

			if (p.length < 4 ) {
				continue;
			}

			String fromAddr = p[2];
			String msgType = p[3];
			if (msgType.startsWith("7A") && i < commands.size() ) {
				out.write (commands.get(i) + "\r");
				out.flush();
				System.out.println (" > " + commands.get(i));
				i++;
			}


		}
	}
}

