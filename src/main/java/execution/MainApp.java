package execution;

import org.graphstream.graph.Graph;

import algorithms.ACO;
import utils.ParseData;

/**
 * ACO Algorithm aiming to find the shortest path between all nodes of a graph.
 *
 * @author Boulbes Thomas
 * @version 1.0
 * @since 23/03/2017
 */
public class MainApp {

	public static void main(String[] args) {
		try {

			System.setProperty("org.graphstream.ui.renderer", "org.graphstream.ui.j2dviewer.J2DGraphRenderer");
			ACO aco = new ACO();
			Graph g = ParseData.generate();
			aco.setGraph(g);
			aco.setSource(g.getNode(0));
			aco.init();
			g.display(true);
			final long startTime = System.currentTimeMillis();
			aco.compute();
			final long endTime = System.currentTimeMillis();
			g.display(true);

			System.out.println("Total execution time: " + (endTime - startTime) / 1000 + "s");
		} catch (Exception e) {
		}
	}

}
