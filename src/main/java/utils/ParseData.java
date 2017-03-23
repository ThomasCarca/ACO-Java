package utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;

/**
 * Parsing util aiming to generate a graph from an input file.
 *
 * @author Boulbes Thomas
 * @version 1.0
 * @since 23/03/2017
 */
public class ParseData {

	// This is the style of nodes and edge on the GUI.
	static String styleSheet = "node {text-size: 20px; text-color: #CC0000; text-style: bold; size: 15px; fill-color: #F45628;}"
			+ "node.explored {fill-color: #146DE4;text-color: #146DE4;}"
			+ "edge {text-size: 10px; text-color: #007E8C; text-style: bold;fill-color: #000000; size: 1.5px;}"
			+ "edge.explored {fill-color: #FD1685; size: 2.5px;}" + "edge.best {fill-color: #75AB08; size: 5px;}";

	/**
	 * This method is used to create a 2D array from an input file
	 *
	 * @param filename
	 *            This is the path of the file.
	 * @return int[][] This is generated 2D array.
	 */
	@SuppressWarnings("resource")
	public static int[][] create2DIntMatrixFromFile(String filename) throws Exception {
		int[][] matrix = null;
		BufferedReader buffer = new BufferedReader(new FileReader(filename));
		String line;
		int row = 0;
		int size = 0;

		while ((line = buffer.readLine()) != null) {
			//You may have to modify the "," according to the input file's form.
			String[] vals = line.trim().split(",");
			if (matrix == null) {
				size = vals.length;
				matrix = new int[size][size];
			}
			for (int col = 0; col < size; col++) {
				matrix[row][col] = Integer.parseInt(vals[col]);
			}
			row++;
		}
		return matrix;
	}

	/**
	 * This method is used to Generate a graph from a 2D array.
	 *
	 * @param matrix
	 *            This is the 2D array.
	 * @return Graph
	 *            This is returned graph.
	 */
	public static Graph createGraphFrom2DIntMatrix(int[][] matrix) {
		Graph g = new SingleGraph("Graph");
		g.addAttribute("ui.stylesheet", styleSheet);
		for (int idNode = 0; idNode < matrix.length; idNode++) {
			g.addNode("" + idNode);
		}
		for (int i = 0; i < matrix.length; i++) {
			for (int j = 0; j < matrix.length; j++) {
				if (i != j) {
					g.addEdge(i + "-" + j, "" + i, "" + j, true).addAttribute("length", matrix[i][j]);
				}
			}
		}
		for (Node n : g) {
			n.addAttribute("label", n.getId());
		}

		for (Edge e : g.getEachEdge()) {
			e.addAttribute("label", "" + (int) e.getNumber("length"));
		}
		return g;
	}

	/**
	 * This method is used to print a 2D array in the console.
	 *
	 * @param matrix
	 *            This is the 2D array.
	 */
	public static void printMatrix(int[][] matrix) {
		String str = "";
		int size = matrix.length;

		if (matrix != null) {
			for (int row = 0; row < size; row++) {
				str += " ";
				for (int col = 0; col < size; col++) {
					str += String.format("%2d", matrix[row][col]);
					if (col < size - 1) {
						str += " | ";
					}
				}
				if (row < size - 1) {
					str += "\n";
					for (int col = 0; col < size; col++) {
						for (int i = 0; i < 4; i++) {
							str += "-";
						}
						if (col < size - 1) {
							str += "+";
						}
					}
					str += "\n";
				} else {
					str += "\n";
				}
			}
		}

		System.out.println(str);
	}

	/**
	 * This method is used to generate a Graph according to the user's input
	 *
	 * @return Graph
	 *            This is the generated graph.
	 */
	public static Graph generate() throws Exception {
		String filename = "";
		JFileChooser chooser = new JFileChooser();
		chooser.setCurrentDirectory(new File("."));
		FileNameExtensionFilter filter = new FileNameExtensionFilter("Text files", "txt");
		chooser.setFileFilter(filter);
		int returnVal = chooser.showOpenDialog(null);
		if (returnVal == JFileChooser.APPROVE_OPTION) {
			filename = chooser.getSelectedFile().getAbsolutePath();
		}
		return createGraphFrom2DIntMatrix(create2DIntMatrixFromFile(filename));
	}

}