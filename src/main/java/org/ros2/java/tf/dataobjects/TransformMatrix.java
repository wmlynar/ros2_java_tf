package org.ros2.java.tf.dataobjects;

import javax.vecmath.Matrix4d;

public class TransformMatrix {

	public boolean exists = false;
	public double time = Double.NaN;
	public Matrix4d matrix = new Matrix4d();

}
