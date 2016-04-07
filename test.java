
import java.lang.Float;
import java.io.*;
import java.util.*;

class test {
	static class line {
		line() {

		}
		float ox, oy, vx, vy;
	}

	static ArrayList<mapping.Vec> shape = new ArrayList<mapping.Vec>();
	//static ArrayList<float> angles = new ArrayList<float>();
	static ArrayList<line> lines = new ArrayList<line>();
	static {
		shape.add(new mapping.Vec(2, 2));
		shape.add(new mapping.Vec(-2, 2));
		shape.add(new mapping.Vec(-2, 4));
		shape.add(new mapping.Vec(-4, 4));
		shape.add(new mapping.Vec(-4, -4));
		shape.add(new mapping.Vec(-4, -3));
		shape.add(new mapping.Vec(-2, -3));
		shape.add(new mapping.Vec(-2, -4));
		shape.add(new mapping.Vec(4, -4));
		shape.add(new mapping.Vec(4, -2));
		shape.add(new mapping.Vec(2, -2));

		for (int i = 0; i < shape.size(); ++i) {
			line l = new line();
			mapping.Vec v1, v2;
			if (i == 0) {
				v1 = shape.get(shape.size()-1);
			} else {
				v1 = shape.get(i-1);
			}
			v2 = shape.get(i);
			//
			l.ox = 20*v1.x;
			l.oy = 20*v1.y;
			l.vx = 20*(v2.x - v1.x);
			l.vy = 20*(v2.y - v1.y);
			lines.add(l);
		}
	}

	public static float x = 0;
	public static float y = 0;
	public static float theta = 0;

	public static float cast(float angle) {
		float bestDist = 100000;
		float vx = (float)Math.cos(theta + angle) * 10000;
		float vy = (float)Math.sin(theta + angle) * 10000;
		for (line l: lines) {
			float denom = vx*l.vy - vy*l.vx;
			float dx = l.ox - x;
			float dy = l.oy - y;
			float t = (dx*l.vy - dy*l.vx) / denom;
			float u = (dx*vy - dy*vx) / denom;
			boolean doesIntersect = ((denom != 0) && (u > 0 && u < 1) && (t > 0 && t < 1));
			if (doesIntersect) {
				bestDist = Math.min(bestDist, 10000*t);
			}
		}
		return bestDist;
	}

	public static float[] sweep(float[] angles) {
		float[] distances = new float[angles.length];
		for (int i = 0; i < angles.length; ++i) {
			distances[i] = cast(angles[i]);
		}
		return distances;
	}


	public static void testSweep(String filename, float[] angles, float[] distances) {
		try {
			Scanner s = new Scanner(new File(filename));
			int i = 0;
			while (s.hasNextLine()) {
				String line = s.nextLine();
				Scanner ln = new Scanner(line);
				ln.useDelimiter(",");
				angles[i] = Float.parseFloat(ln.next());
				distances[i] = Float.parseFloat(ln.next()) + 8.0f;
				++i;
			}
		} catch (FileNotFoundException e) {
			System.out.println("Couldn't open " + filename);
		}
	}

	static float orig = 400.0f;
	static float scale = 0.3f;

	public static void testOutput(float[] angles, float[] distances, mapping m, String filename) {
		ArrayList<String> cmd = 
			new ArrayList<String>(Arrays.asList("convert", 
				"-size", "800x800", "xc:white", "-fill", "red", "-stroke", "black"));
		for (int i = 0; i < 180; ++i) {
			int x = (int)((float)Math.cos(angles[i] + m.getAngle()) * distances[i] + m.getPosition().x);
			int y = (int)((float)Math.sin(angles[i] + m.getAngle()) * distances[i] + m.getPosition().y);
			cmd.add("-draw");
			cmd.add("circle " + (orig+scale*x-1) + "," + 
				(orig+scale*y-1) + " " + (orig+scale*x+1) + "," + (orig+scale*y+1));
		}
		//
		cmd.add("-stroke");
		cmd.add("black");
		cmd.add("-strokewidth");
		cmd.add("2");
		for (mapping.MapSegment seg: m.lastSegments) {
			cmd.add("-draw");
			cmd.add("line " + (orig+scale*seg.origin.x) + "," + (orig+scale*seg.origin.y) + " " +
				(orig+scale*seg.origin.x + scale*seg.vec.x) + "," + 
				(orig+scale*seg.origin.y + scale*seg.vec.y));
		}
		//
		cmd.add("-stroke");
		cmd.add("blue");
		cmd.add("-strokewidth");
		cmd.add("2");
		for (mapping.MapSegment seg: m.getSegments()) {
			cmd.add("-draw");
			cmd.add("line " + (orig+scale*seg.origin.x) + "," + (orig+scale*seg.origin.y) + " " +
				(orig+scale*seg.origin.x + scale*seg.vec.x) + "," + 
				(orig+scale*seg.origin.y + scale*seg.vec.y));
		}
		//
		cmd.add("-stroke");
		cmd.add("black");
		cmd.add("-fill");
		cmd.add("black");
		cmd.add("-draw");
		cmd.add("circle " + 
			(m.getPosition().x*scale-3+orig) + "," + (m.getPosition().y*scale-3+orig) + " " +
			(m.getPosition().x*scale+3+orig) + "," + (m.getPosition().y*scale+3+orig));
		cmd.add("-stroke");
		cmd.add("black");
		cmd.add("-draw");
		cmd.add("line " + (orig+m.getPosition().x*scale) + "," + (orig+m.getPosition().y*scale) 
			+ " " + 
			(m.getPosition().x*scale + orig + scale*(float)Math.cos(m.getAngle()) * 100) + "," +
			(m.getPosition().y*scale + orig + scale*(float)Math.sin(m.getAngle()) * 100));
		cmd.add(filename);
		try {
			Runtime r = Runtime.getRuntime();
			Process p = r.exec(cmd.toArray(new String[0]));
			p.waitFor();
			System.out.println("Output");
		} catch (Exception e) {
			System.out.println("Couldn't output: ");
		}
	}


	public static void main(String[] args) {
		// // Init
		// int count = 50;
		// float[] angles = new float[count];
		// for (int i = 0; i < count; ++i) {
		// 	float angle = (i/50.0f) * ((float)Math.PI*2.0f);
		// 	angles[i] = angle;
		// }

		// // Sweep
		// float[] distances = sweep(angles);
		float[] angles = new float[180];
		float[] distances = new float[180]; 

		// Print
		// String out = "convert -size 200x200 xc:white -fill black -stroke black ";
		// for (int i = 0; i < count; ++i) {
		// 	int tx = (int)(100 + x + (float)Math.cos(angles[i])*distances[i]);
		// 	int ty = (int)(100 + y + (float)Math.sin(angles[i])*distances[i]);
		// 	out = out + "-draw \"circle " + (tx-1) + "," + (ty-1) + " "
		// 		+ (tx+1) + "," + (ty+1) + "\" ";
		// }
		// out = out + "test.png";
		//System.out.println(out);


		// Mapping
		mapping m = new mapping();

		m.init();

		testSweep("data/log-INITIAL-0.csv", angles, distances);
		m.initialScan(angles, distances);
		testOutput(angles, distances, m, "test1.png");

		testSweep("data/log-ROTATION-1.csv", angles, distances);
		m.updateRot(angles, distances, -0.5f);
		testOutput(angles, distances, m, "test2.png");

		testSweep("data/log-ROTATION-2.csv", angles, distances);
		m.updateRot(angles, distances, -1);
		testOutput(angles, distances, m, "test3.png");

		testSweep("data/log-LINEAR-3.csv", angles, distances);
		m.updateLin(angles, distances, 1.0f);
		testOutput(angles, distances, m, "test4.png");

		testSweep("data/log-ROTATION-4.csv", angles, distances);
		m.updateRot(angles, distances, -1);
		testOutput(angles, distances, m, "test5.png");

		testSweep("data/log-LINEAR-5.csv", angles, distances);
		m.updateLin(angles, distances, 1);
		testOutput(angles, distances, m, "test6.png");

		testSweep("data/log-LINEAR-6.csv", angles, distances);
		m.updateLin(angles, distances, 1);
		testOutput(angles, distances, m, "test7.png");

		System.out.println("Test");
	}
}
