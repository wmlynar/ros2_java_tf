package org.ros2.java.tf;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.ros2.java.tf.dataobjects.LatestTime;
import org.ros2.java.tf.dataobjects.Path;
import org.ros2.java.tf.dataobjects.StringPair;
import org.ros2.java.tf.dataobjects.TransformMatrix;
import org.ros2.java.tf.internal.TransformBuffer;

public class TfManager {

	public final static int LATEST_SAME_TIME = 0;
	public final static int LATEST_INDIVIDUALLY = 1;
	public final static int SPECIFIC = 2;

	// for checking if TfManager needs to be reset because of transform back in
	// time (looped logs)
	private final static double SECONDS_BACK_IN_TIME_TO_RESET = 5;

	public String staticTransformKeyword = "static";

	// structures to access transform buffers (transforms sorted over time)
	private final ArrayList<TransformBuffer> transformBufferList = new ArrayList<>();
	private final HashMap<StringPair, TransformBuffer> transformBufferMap = new HashMap<>();

	// cache of the calculated path along transform buffer indexes
	private final HashMap<StringPair, Path> pathCache = new HashMap<>();

	// graph used to compute path between coordinate frames using the transforms
	private final Graph<String, Integer> graph = new SimpleDirectedGraph<>(Integer.class);

	// mutex used for synchronisation of all public methods
	private final Object mutex = new Object();

	// objects used for avoiding to allocate memory
	private final StringPair stringPair = new StringPair("", "");
	private final Matrix4d mat = new Matrix4d();
	private final TransformMatrix trans = new TransformMatrix();

	// lates available transform time
	private double lastTime = Double.NEGATIVE_INFINITY;

	// temporary object used to avoid memory allocation
	private final LatestTime latestTime = new LatestTime();

	/**
	 * Add single transform.
	 */
	public void add(geometry_msgs.msg.TransformStamped transformStamped) {
		Transform transform = new Transform(transformStamped);
		add(transform);
	}

	/**
	 * Add list of transforms in tfMessage.
	 */
	public void add(tf2_msgs.msg.TFMessage tfMessage) {
		TransformList transformList = new TransformList(tfMessage);
		add(transformList);
	}

	/**
	 * Add single transform.
	 */
	public void add(Transform transform) {
		synchronized (mutex) {
			if (!isQuaternionCorrect(transform.rotation)) {
				System.out.println("Skipping transform with wrong quaternion " + transform.rotation.toString());
				return;
			}

			checkIfShouldReset(transform.time);

			Matrix4d matrix = new Matrix4d(transform.rotation, transform.translation, 1.0);

			TransformBuffer tb = getTransformBuffer(transform);
			tb.addTransform(transform.time, matrix);
		}
	}

	/**
	 * Add transform list.
	 */
	public void add(TransformList transformList) {
		// synchronized, bcause adding all transforms need to an atomic operation
		// (for example drif cancelation)
		synchronized (mutex) {
			for (Transform transform : transformList.transforms) {
				add(transform);
			}
		}
	}

	/**
	 * Get the time of the latest available transform chain, returns
	 * Double.NEGATIVE_INFINITY if there is no path or the transform is not
	 * available.
	 */
	public double getLatestCommonTime(String from, String to) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null) {
				return Double.NEGATIVE_INFINITY;
			}
			if (getLatestCommonTime(path.indexes, latestTime)) {
				return latestTime.time;
			} else {
				return Double.NEGATIVE_INFINITY;
			}
		}
	}

	/**
	 * Obtain transform at specific time.
	 */
	public boolean lookupTransform(String from, String to, double time, Matrix4d mat) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			return multiplyMatricesInPath(path.indexes, path.inverted, time, mat);
		}
	}

	/**
	 * Obtain transform at specific time.
	 */
	public boolean lookupTransform(String from, String to, double time, Transform transform) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			if (multiplyMatricesInPath(path.indexes, path.inverted, time, mat)) {
				transform.time = time;
				transform.parentFrameId = from;
				transform.childFrameId = to;
				transform.set(mat);
				return true;
			} else {
				return false;
			}
		}
	}

	/**
	 * Obtain latest available transform chain.
	 */
	public boolean lookupTransform(String from, String to, Matrix4d mat) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			if (!getLatestCommonTime(path.indexes, latestTime)) {
				return false;
			}
			return multiplyMatricesInPath(path.indexes, path.inverted, latestTime.time, mat);
		}
	}

	public boolean lookupTransform(String from, String to, Transform transform) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			if (!getLatestCommonTime(path.indexes, latestTime)) {
				return false;
			}

			if (multiplyMatricesInPath(path.indexes, path.inverted, latestTime.time, mat)) {
				transform.time = latestTime.time;
				transform.parentFrameId = from;
				transform.childFrameId = to;
				transform.set(mat);
				return true;
			} else {
				return false;
			}
		}
	}

	/**
	 * Get the transform composed from individually latest transforms. WARNING! use
	 * with care, it can cause strange effects when used incorrectly. Especially
	 * NEVER publish computations based on this again to TfManager, only publish for
	 * example as odometry or pose.
	 */
	public boolean lookupTransformIndividuallyLatest(String from, String to, Matrix4d mat) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			if (multiplyMatricesInPathIndividuallyLatest(path.indexes, path.inverted, trans)) {
				mat.set(trans.matrix);
				return true;
			} else {
				return false;
			}
		}
	}

	/**
	 * Get the transform composed from individually latest transforms. WARNING! use
	 * with care, it can cause strange effects when used incorrectly. Especially
	 * NEVER publish computations based on this again to TfManager, only publish for
	 * example as odometry or pose.
	 */
	public boolean lookupTransformIndividuallyLatest(String from, String to, Transform transform) {
		synchronized (mutex) {
			Path path = getPath(from, to);
			if (path == null || path.indexes.length == 0) {
				return false;
			}
			if (multiplyMatricesInPathIndividuallyLatest(path.indexes, path.inverted, trans)) {
				transform.time = trans.time;
				transform.parentFrameId = from;
				transform.childFrameId = to;
				transform.set(trans.matrix);
				return true;
			} else {
				return false;
			}
		}
	}

	private void checkIfShouldReset(double time) {
		if (time + SECONDS_BACK_IN_TIME_TO_RESET < lastTime) {
			for (TransformBuffer tb : transformBufferList) {
				tb.reset(time);
			}

		}
		lastTime = time;
	}

	private TransformBuffer getTransformBuffer(Transform transform) {
		boolean isStaticTransform = transform.childFrameId.contains(staticTransformKeyword);
		stringPair.set(transform.parentFrameId, transform.childFrameId);
		TransformBuffer tb = transformBufferMap.get(stringPair);
		if (tb == null) {
			tb = createTransformBuffer(transform.parentFrameId, transform.childFrameId, isStaticTransform);
		} else if (tb.isStaticTransform != isStaticTransform) {
			throw new IllegalArgumentException("Transform cannot change status of isStaticTransform");
		}
		return tb;
	}

	private Path getPath(String from, String to) {
		stringPair.set(from, to);
		Path path = pathCache.get(stringPair);
		if (path == null) {
			// path not in cache
			path = computePathAndAddToCache(from, to);
		}
		return path;
	}

	private TransformBuffer createTransformBuffer(String from, String to, boolean isStaticTransform) {
		int transformBufferIndexInList = transformBufferList.size();

		TransformBuffer tb = new TransformBuffer(isStaticTransform);
		StringPair sp = new StringPair(from, to);
		transformBufferMap.put(sp, tb);
		transformBufferList.add(tb);

		// add graph nodes
		if (!graph.containsVertex(from)) {
			graph.addVertex(from);
		}
		if (!graph.containsVertex(to)) {
			graph.addVertex(to);
		}
		// add graph edge
		if (!graph.containsEdge(from, to)) {
			graph.addEdge(from, to, transformBufferIndexInList);
		}
		return tb;
	}

	private boolean multiplyMatricesInPath(int[] path, boolean inverted, double time, Matrix4d matrix) {
		if (!transformBufferList.get(path[0]).getTransform(time, matrix)) {
			return false;
		}
		for (int i = 1; i < path.length; i++) {
			if (!transformBufferList.get(path[i]).getTransform(time, mat)) {
				return false;
			}
			matrix.mul(mat);
		}
		if (inverted) {
			matrix.invert();
		}
		return true;
	}

	private boolean multiplyMatricesInPathIndividuallyLatest(int[] path, boolean inverted, TransformMatrix transform) {
		if (!transformBufferList.get(path[0]).getTransformLatest(transform)) {
			return false;
		}
		for (int i = 1; i < path.length; i++) {
			if (!transformBufferList.get(path[i]).getTransformLatest(trans)) {
				return false;
			}
			transform.matrix.mul(trans.matrix);
			if (trans.time > transform.time) {
				trans.time = transform.time;
			}
		}
		if (inverted) {
			transform.matrix.invert();
		}
		return true;
	}

	private boolean getLatestCommonTime(int[] path, LatestTime latestTime) {
		latestTime.time = Double.POSITIVE_INFINITY;
		for (int i = 0; i < path.length; i++) {
			if (!transformBufferList.get(path[i]).updateLatestTime(latestTime)) {
				return false;
			}
		}
		return true;
	}

	private Path computePathAndAddToCache(String from, String to) {
		try {
			Path path;
			GraphPath<String, Integer> graphPath = DijkstraShortestPath.findPathBetween(graph, from, to);
			if (graphPath == null) {
				graphPath = DijkstraShortestPath.findPathBetween(graph, to, from);
				if (graphPath == null) {
					return null;
				}
				path = new Path();
				path.inverted = true;
			} else {
				path = new Path();
			}
			path.indexes = graphPathToArray(graphPath);
			pathCache.put(stringPair, path);
			return path;
		} catch (IllegalArgumentException e) {
			// Dijkstra search did not find path
			return null;
		}
	}

	private int[] graphPathToArray(GraphPath<String, Integer> path) {
		return toIntArray(path.getEdgeList());
	}

	private int[] toIntArray(List<Integer> list) {
		int[] ret = new int[list.size()];
		for (int i = 0; i < ret.length; i++) {
			ret[i] = list.get(i);
		}
		return ret;
	}

	private boolean isQuaternionCorrect(Quat4d q) {
		double sum = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		return sum > 0.5;
	}

}
