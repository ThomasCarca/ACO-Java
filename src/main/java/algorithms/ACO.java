package algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;

/**
 * ACO Algorithm aiming to find the shortest path between all nodes of a graph.
 *
 * @author Boulbes Thomas
 * @version 1.0
 * @since 23/03/2017
 */
public class ACO {

	private int totalAnts, nbIterations = 10;
	private double p, alpha, beta, q;
	private Node source;
	private Graph graph;
	private long time = 1000;

	/**
	 * This method is used to get the input values from the user and to
	 * initiliaze the graph.
	 */
	public void init() {

		Exception exception = null;
		Integer result = null;
		JTextField totalAntsField = new JTextField("" + graph.getNodeCount(), 5);
		JTextField nbIterationsField = new JTextField("50", 5);
		JTextField alphaField = new JTextField("1", 5);
		JTextField betaField = new JTextField("5", 5);
		JTextField qField = new JTextField("1", 5);
		JTextField pField = new JTextField("0.01", 5);
		JTextField timeField = new JTextField("0", 5);

		JPanel myPanel = new JPanel();
		myPanel.setLayout(new BoxLayout(myPanel, BoxLayout.Y_AXIS));
		myPanel.add(new JLabel("Number of ants:"));
		myPanel.add(totalAntsField);
		myPanel.add(new JLabel("Number of iterations:"));
		myPanel.add(nbIterationsField);
		myPanel.add(new JLabel("alpha:"));
		myPanel.add(alphaField);
		myPanel.add(new JLabel("beta:"));
		myPanel.add(betaField);
		myPanel.add(new JLabel("Q:"));
		myPanel.add(qField);
		myPanel.add(new JLabel("p:"));
		myPanel.add(pField);
		myPanel.add(new JLabel("Time between iterations (ms):"));
		myPanel.add(timeField);

		do {
			try {
				exception = null;
				result = JOptionPane.showConfirmDialog(null, myPanel, "Please enter values",
						JOptionPane.OK_CANCEL_OPTION);
				this.totalAnts = Integer.parseInt(totalAntsField.getText());
				this.nbIterations = Integer.parseInt(nbIterationsField.getText());
				this.beta = Double.parseDouble(betaField.getText());
				this.alpha = Double.parseDouble(alphaField.getText());
				this.q = Double.parseDouble(qField.getText());
				this.p = Double.parseDouble(pField.getText());
				this.time = Integer.parseInt(timeField.getText());
				setAllNodesUnexplored();
				setNodeExplored(source, true);
				initEdges();
			} catch (Exception e) {
				exception = e;
				e.printStackTrace();
			}
		} while (exception != null && result != JOptionPane.CANCEL_OPTION);
	}

	/**
	 * This method is used to compute the ACO algorithm on the graph.
	 */
	public void compute() {
		Node colony = source;
		List<Edge> bestPath = new ArrayList<Edge>();
		List<List<Edge>> paths = new ArrayList<List<Edge>>();
		//Number of iterations
		for (int tmp = 0; tmp < nbIterations; tmp++) {
			//All ants travel across the graph one by one
			for (int ant = 0; ant < totalAnts; ant++) {
				List<Edge> path = new ArrayList<Edge>();
				//Do this while the ant has not traveled across all nodes
				while (getUnexploredNodes().size() != 0) {
					try {
						Thread.sleep(time);
					} catch (InterruptedException ex) {
						Thread.currentThread().interrupt();
					}
					Edge chosenEdge = chooseEdge(getAvailableEdges());
					setEdgeExplored(chosenEdge, true);
					setNodeExplored(chosenEdge.getTargetNode(), true);
					setSource(chosenEdge.getTargetNode());
					path.add(chosenEdge);
					for (Edge edge : source.getEachLeavingEdge()) {
						setProbability(edge);
					}
					//If the ant has traveled across all nodes
					if (getUnexploredNodes().size() == 0) {
						try {
							Thread.sleep(time);
						} catch (InterruptedException ex) {
							Thread.currentThread().interrupt();
						}
						//Reset the source edge to the colony
						chosenEdge = getEdge(source, colony);
						path.add(chosenEdge);
						setEdgeExplored(chosenEdge, true);
						try {
							Thread.sleep(time);
						} catch (InterruptedException ex) {
							Thread.currentThread().interrupt();
						}
						//store the ant's path
						paths.add(path);
					}
				}
				//Once an ant is done exploring, reset the graph for the next ant
				setAllEdgesUnexplored();
				setAllNodesUnexplored();
				setNodeExplored(colony, true);
				setSource(colony);
			}
			//Retrieve the best path
			if (fitness(bestPath(paths)) < fitness(bestPath)) {
				bestPath = bestPath(paths);
			}
			//Add all pheromones droped by the ants
			addPheromone(paths);
			System.out.println(" - Iteration n°" + tmp + " - ");
			System.out.println("Length of best path is : " + fitness(bestPath));
			System.out.println();
		}
		//Display the final best path that has been traveled across
		updateFinalPath(bestPath);
		System.out.println("Best path : " + bestPath);
		System.out.println("With a length of " + fitness(bestPath));
	}

	/**
	 * This method is used to set the final path class attribute to "best" in
	 * order to display it in green on the GUI.
	 * 
	 * @param path
	 *            This is the best path found during the algorithm computation.
	 */
	public void updateFinalPath(List<Edge> path) {
		for (Edge edge : path) {
			graph.getEdge(edge.getId()).setAttribute("ui.class", "best");
		}
	}

	/**
	 * This method is used to retrieve the best path from a list of paths.
	 * 
	 * @param paths
	 *            This is a list of paths.
	 * @return List<Edge> This returns the best path found.
	 */
	public List<Edge> bestPath(List<List<Edge>> paths) {
		List<Edge> currentBestPath = paths.get(new Random().nextInt(paths.size()));
		Double currentBestWeight = fitness(currentBestPath);
		for (List<Edge> path : paths) {
			Double weight = fitness(path);
			if (weight < currentBestWeight) {
				currentBestWeight = weight;
				currentBestPath = path;
			}
		}
		return currentBestPath;
	}

	/**
	 * This method is used to display the pheromones of each edge in the
	 * terminal.
	 */
	public void displayPheromones() {
		for (Edge edge : graph.getEachEdge()) {
			System.out.println("Edge " + edge.toString() + " has " + edge.getAttribute("pheromone") + " pheromones.");
		}
	}

	/**
	 * This method is used to add pheromones to each edge of the graph according
	 * to each path the ants have traveled across.
	 * 
	 * @param paths
	 *            This is the list of path each ant have traveled across.
	 */
	public void addPheromone(List<List<Edge>> paths) {
		double sumPheromones = 0;
		Map<List<Edge>, Double> weights = new HashMap<List<Edge>, Double>();
		for (List<Edge> path : paths) {
			weights.put(path, pheromoneAmount(path));
		}
		for (Edge edge : graph.getEachEdge()) {
			sumPheromones = 0;
			for (List<Edge> path : paths) {
				if (path.contains(edge)) {
					sumPheromones += weights.get(path);
				}
			}
			double currentPheromone = edge.getAttribute("pheromone");
			edge.setAttribute("pheromone", (1 - p) * currentPheromone + sumPheromones);
		}
	}

	/**
	 * This method is used to calculate the some of each edge's length in a
	 * path.
	 * 
	 * @param path
	 *            This is a list of edges.
	 * @param double
	 *            This is the fitness of the path.
	 */
	public double fitness(List<Edge> path) {
		if (path.isEmpty()) {
			return Integer.MAX_VALUE;
		} else {
			double sum = 0;
			for (Edge edge : path) {
				sum += (Integer) edge.getAttribute("length");
			}
			return sum;
		}
	}

	/**
	 * This method is used to calculate the pheromone amount that should be
	 * droped on an edge according to a path.
	 * 
	 * @param path
	 *            This is a list of edge.
	 * @return double This returns q divided by the fitness of the path where q
	 *         is an user's input.
	 */
	public double pheromoneAmount(List<Edge> path) {
		return q / fitness(path);
	}

	/**
	 * This method is used to set the probability on an edge.
	 * 
	 * @param edge
	 *            This is the edge we want to set the probability to.
	 * @return double This returns the probability that has just been set to the
	 *         edge.
	 */
	public double setProbability(Edge edge) {
		Node start = edge.getSourceNode();
		double p;
		p = (Math.pow(edge.getAttribute("pheromone"), alpha)
				* Math.pow((double) 1 / (Integer) edge.getAttribute("length"), beta));
		p /= sumUnexploredNodesFrom(start);
		edge.setAttribute("probability", p);
		return p;
	}

	/**
	 * This method is used in order to calculate the probability of an edge.
	 *
	 * @param start
	 *            This is the starting node.
	 */
	public double sumUnexploredNodesFrom(Node start) {
		double sum = 0;
		List<Node> unexploredNodes = getUnexploredNodes();
		for (Node node : unexploredNodes) {
			sum += Math.pow(getEdgePheromone(start, node), alpha)
					* Math.pow((double) 1 / getEdgeWeight(start, node), beta);
		}
		return sum;
	}

	/**
	 * This method is used to set a node class attribute to "explored" in order
	 * to display it in pink in the GUI. It also sets the node explored
	 * attribute to true or false according to the b param.
	 * 
	 * @param node
	 *            This is the node to modify.
	 * @param b
	 *            This is the boolean that is set to the node explored
	 *            attribute.
	 */
	public void setNodeExplored(Node node, boolean b) {
		node.setAttribute("explored", b);
		if (b)
			node.addAttribute("ui.class", "explored");
		else
			node.removeAttribute("ui.class");
	}

	/**
	 * This method is used to set an edge class attribute to "explored" in order
	 * to display it in blue in the GUI. It also sets the edge explored
	 * attribute to true or false according to the b param.
	 * 
	 * @param edge
	 *            This is the edge to modify.
	 * @param b
	 *            This is the boolean that is set to the edge explored
	 *            attribute.
	 */
	public void setEdgeExplored(Edge edge, boolean b) {
		edge.setAttribute("explored", b);
		if (b)
			edge.addAttribute("ui.class", "explored");
		else
			edge.removeAttribute("ui.class");
	}

	/**
	 * This method is used to initialize all edges of the graph.
	 */
	public void initEdges() {
		for (Edge edge : graph.getEachEdge()) {
			edge.setAttribute("pheromone", 0.01);
			edge.setAttribute("explored", false);
		}
		for (Edge edge : source.getEachLeavingEdge()) {
			setProbability(edge);
		}
	}

	/**
	 * This method is used to set all nodes to unexplored.
	 */
	public void setAllNodesUnexplored() {
		for (Node node : graph) {
			setNodeExplored(node, false);
		}
	}

	/**
	 * This method is used to set all edges to unexplored.
	 */
	public void setAllEdgesUnexplored() {
		for (Edge edge : graph.getEachEdge()) {
			setEdgeExplored(edge, false);
		}
	}

	/**
	 * This method is used to retrieve an edge from its starting node and ending
	 * node.
	 * 
	 * @param start
	 *            This is the starting node.
	 * @param end
	 *            This is the ending node.
	 */
	public Edge getEdge(Node start, Node end) {
		return graph.getEdge(start.getId() + "-" + end.getId());
	}

	/**
	 * This method is used to retrieve an edge length attribute from its
	 * starting node and ending node.
	 *
	 * @param start
	 *            This is the starting node.
	 * @param end
	 *            This is the ending node.
	 */
	public int getEdgeWeight(Node start, Node end) {
		return getEdge(start, end).getAttribute("length");
	}

	/**
	 * This method is used to retrieve an edge pheromone attribute from its
	 * starting node and ending node.
	 *
	 * @param start
	 *            This is the starting node.
	 * @param end
	 *            This is the ending node.
	 */
	public double getEdgePheromone(Node start, Node end) {
		return getEdge(start, end).getAttribute("pheromone");
	}

	/**
	 * This method is used to retrieve all unexplored nodes of the graph.
	 * 
	 * @return List<Node> This is list of unexplored nodes.
	 */
	public List<Node> getUnexploredNodes() {
		return ((Collection<Node>) graph.getNodeSet()).stream().filter(node -> !isNodeExplored(node))
				.collect(Collectors.toList());
	}

	/**
	 * This method is used to retrieve all explored edges of the graph.
	 * 
	 * @return List<Edge> This is list of explored edges.
	 */
	public List<Edge> getExploredEdges() {
		return ((Collection<Edge>) graph.getEdgeSet()).stream().filter(edge -> isEdgeExplored(edge))
				.collect(Collectors.toList());
	}

	/**
	 * This method is used to retrieve all available edges from the source node.
	 * 
	 * @return List<Edge> This is list of available edges.
	 */
	public List<Edge> getAvailableEdges() {
		List<Node> unexploredNodes = getUnexploredNodes();
		List<Edge> availableEdges = new ArrayList<Edge>();
		for (Node node : unexploredNodes) {
			availableEdges.add(getEdge(source, node));
		}
		return availableEdges;
	}

	/**
	 * This method is used to ask if a node is explored.
	 * 
	 * @return boolean This tells if the node is explored or not.
	 */
	public boolean isNodeExplored(Node node) {
		return node.getAttribute("explored");
	}

	/**
	 * This method is used to ask if an edge is explored.
	 * 
	 * @return boolean This tells if the edge is explored or not.
	 */
	public boolean isEdgeExplored(Edge edge) {
		return edge.getAttribute("explored");
	}

	/**
	 * This method is used to choose randomly an edge between a list of edges.
	 * the higher the edge's probability is, the higher the chance to choose it
	 * is.
	 * 
	 * @return Edge This is the edge that has been chosen according to the
	 *         roulette.
	 */
	public Edge chooseEdge(List<Edge> edges) {
		Edge[] roulette = new Edge[edges.size()];
		double[] probabilityArray = new double[edges.size()];
		int acc = edges.size() - 1;
		double r = Math.random();
		probabilityArray[acc] = 1;
		for (Edge edge : edges) {
			roulette[acc] = edge;
			if (acc != 0) {
				probabilityArray[acc - 1] = probabilityArray[acc] - (Double) edge.getAttribute("probability");
			} else {
				probabilityArray[acc] = (Double) edge.getAttribute("probability");
			}
			acc--;
		}
		acc = 0;
		while (r > probabilityArray[acc]) {
			acc++;
		}
		return roulette[acc];
	}
	
	/*
	 * GETTERS AND SETTERS
	 */

	public int getAnts() {
		return totalAnts;
	}

	public void setAnts(int totalAnts) {
		this.totalAnts = totalAnts;
	}

	public double getAlpha() {
		return alpha;
	}

	public void setAlpha(double alpha) {
		this.alpha = alpha;
	}

	public double getBeta() {
		return beta;
	}

	public void setBeta(double beta) {
		this.beta = beta;
	}

	public double getP() {
		return p;
	}

	public void setP(double p) {
		this.p = p;
	}

	public double getQ() {
		return q;
	}

	public void setQ(double q) {
		this.q = q;
	}

	public Node getSource() {
		return source;
	}

	public void setSource(Node source) {
		this.source = source;
	}

	public Graph getGraph() {
		return graph;
	}

	public void setGraph(Graph graph) {
		this.graph = graph;
	}

	public double getTime() {
		return time;
	}

	public void setTime(long time) {
		this.time = time;
	}

	public int getNbIterations() {
		return nbIterations;
	}

	public void setNbIterations(int nbIterations) {
		this.nbIterations = nbIterations;
	}

}
