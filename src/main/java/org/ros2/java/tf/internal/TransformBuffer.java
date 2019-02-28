package org.ros2.java.tf.internal;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros2.java.tf.dataobjects.LatestTime;
import org.ros2.java.tf.dataobjects.TransformMatrix;

/**
 * Circular buffer of transforms between two coordinate frames sorted in time.
 * Holds <code>CAPACITY_SIZE</code> number of latest transform updates.
 *
 */
public class TransformBuffer {

	private final static double DOUBLE_EPSILON = 1e-8;

	private final static int CAPACITY_SIZE = 1 << 10;
	private final static int CAPACITY_MASK = (1 << 10) - 1;

	private final TransformMatrix[] circularBuffer;

	private int pos = 0;
	private double firstTime = Double.NEGATIVE_INFINITY;

	/**
	 * Public only for reading, do not modify outside of this class.
	 */
	public boolean isStaticTransform = false;

	/**
	 * When transform is static than it is valid for all time periods.
	 */
	public TransformBuffer(boolean isStaticTransform) {
		this.isStaticTransform = isStaticTransform;
		if (isStaticTransform) {
			circularBuffer = new TransformMatrix[1];
			circularBuffer[0] = new TransformMatrix();
		} else {
			circularBuffer = new TransformMatrix[CAPACITY_SIZE];
			for (int i = 0; i < CAPACITY_SIZE; i++) {
				circularBuffer[i] = new TransformMatrix();
			}
		}
	}

	/**
	 * When <code>TransformBuffer</code> is static it will update the transform.
	 * When <code>TransformBuffer</code> is dynamic it will add the transform to the
	 * end of the list, sorted by time and it will not accept transform before the
	 * latest received.
	 */
	public void addTransform(double time, Matrix4d matrix) {
		if (isStaticTransform) {
			circularBuffer[0].time = time;
			circularBuffer[0].matrix.set(matrix);
			circularBuffer[0].exists = true;
			return;
		}
		if (time <= firstTime) {
			return;
		}
		firstTime = time;
		circularBuffer[pos].time = time;
		circularBuffer[pos].matrix.set(matrix);
		circularBuffer[pos].exists = true;
		pos = (pos + 1) & CAPACITY_MASK;
	}

	/**
	 * Get transform at specific time. When transform is static it will always
	 * return the latest transform. When transform is dynamic it will return
	 * interpolated transform between two transforms before and next in time.
	 */
	public boolean getTransform(double time, Matrix4d matrix) {
		if (isStaticTransform) {
			matrix.set(circularBuffer[0].matrix);
			return circularBuffer[0].exists;
		}
		if (time > firstTime) {
			return false;
		}

		int floorIndex = findFloorIndex(time);
		if (floorIndex < 0) {
			return false;
		}
		int ceilIndex = (floorIndex + 1) & CAPACITY_MASK;

		// if the time is exact with ceil, return exact ceil transform
		TransformMatrix ceil = circularBuffer[ceilIndex];
		if (Math.abs(ceil.time - time) < DOUBLE_EPSILON) {
			matrix.set(ceil.matrix);
			return true;
		}

		// interpolate between floor and ceil transforms
		TransformMatrix floor = circularBuffer[floorIndex];
		if (!floor.exists) {
			// missing floor transform
			return false;
		}

		double alpha = (time - floor.time) / (ceil.time - floor.time);
		interpolate(floor.matrix, ceil.matrix, alpha, matrix);

		return true;
	}

	/**
	 * Get latest available transform.
	 */
	public boolean getTransformLatest(Matrix4d matrix) {
		if (isStaticTransform) {
			circularBuffer[0].matrix.set(matrix);
			return circularBuffer[0].exists;
		}
		int index = (pos - 1 + CAPACITY_SIZE) & CAPACITY_MASK;
		if (!circularBuffer[index].exists) {
			return false;
		}
		matrix.set(circularBuffer[index].matrix);
		return true;

	}

	/**
	 * Get latest available transform.
	 */
	public boolean getTransformLatest(TransformMatrix transform) {
		if (isStaticTransform) {
			circularBuffer[0].matrix.set(transform.matrix);
			return circularBuffer[0].exists;
		}
		int index = (pos - 1 + CAPACITY_SIZE) & CAPACITY_MASK;
		if (!circularBuffer[index].exists) {
			return false;
		}
		TransformMatrix t = circularBuffer[index];
		transform.matrix.set(t.matrix);
		transform.time = t.time;
		return true;
	}

	public boolean updateLatestTime(LatestTime latestTime) {
		if (isStaticTransform) {
			return true;
		}
		if (firstTime == Double.NEGATIVE_INFINITY) {
			return false;
		}
		if (firstTime < latestTime.time) {
			latestTime.time = firstTime;
		}
		return true;
	}

	/**
	 * Reset transform buffer to initial state.
	 *
	 * @param time
	 */
	public void reset(double time) {
		boolean found = false;
		for (int i = 0; i < circularBuffer.length; i++) {
			if (circularBuffer[i].exists) {
				circularBuffer[i].time = time;
				found = true;
			}
			if (found) {
				firstTime = time;
			} else {
				firstTime = Double.NEGATIVE_INFINITY;
			}

		}
	}

	/**
	 * Returns an index of the oldest value younger than time. WARNING: returns -1
	 * when time is younger than the youngest available transform
	 */
	private int findFloorIndex(double time) {
		int start = pos - 1 + 2 * CAPACITY_SIZE; // make sure pos is always positive
		int end = pos - CAPACITY_SIZE - 1 + 2 * CAPACITY_SIZE;

		for (int i = start; i > end; i--) {
			int index = i & CAPACITY_MASK;
			TransformMatrix t = circularBuffer[index];
			if (!t.exists || t.time < time) {
				return index;
			}
		}
		return -1;
	}

	// temporary variables used for avoiding memory allocation
	private final Vector3d vec1 = new Vector3d();
	private final Quat4d quat1 = new Quat4d();
	private final Vector3d vec2 = new Vector3d();
	private final Quat4d quat2 = new Quat4d();
	private final Vector3d vec3 = new Vector3d();
	private final Quat4d quat3 = new Quat4d();

	private void interpolate(Matrix4d m1, Matrix4d m2, double alpha, Matrix4d matrix) {
		// interpolate translation
		m1.get(vec1);
		m2.get(vec2);
		vec3.interpolate(vec1, vec2, alpha);
		// interpolate rotation
		m1.get(quat1);
		m2.get(quat2);
		quat3.interpolate(quat1, quat2, alpha);
		// compose matrix of rotation, translation and scale of 1
		matrix.set(quat3, vec3, 1);
	}

}
