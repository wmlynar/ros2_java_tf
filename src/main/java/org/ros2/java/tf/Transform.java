package org.ros2.java.tf;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class Transform {

	public double time;
	public String parentFrameId;
	public String childFrameId;

	public Vector3d translation = new Vector3d();
	public Quat4d rotation = new Quat4d();

	public Transform() {
	}

	public Transform(Transform transform) {
		set(transform);
	}

	public Transform(double time, String parentFrameId, String childFrameId) {
		this.time = time;
		this.parentFrameId = parentFrameId;
		this.childFrameId = childFrameId;
	}

	public Transform(geometry_msgs.msg.TransformStamped transform) {
		set(transform);
	}

	public Transform(double time, String parentFrameId, String childFrameId, Matrix4d matrix) {
		set(time, parentFrameId, childFrameId, matrix);
	}

	public Matrix4d asMatrix() {
		Matrix4d mat = new Matrix4d();
		get(mat);
		return mat;
	}

	public void set(Transform t) {
		time = t.time;
		parentFrameId = t.parentFrameId;
		childFrameId = t.childFrameId;
		translation = t.translation;
		rotation = t.rotation;
	}

	public void set(double time, String parentFrameId, String childFrameId, Matrix4d matrix) {
		this.time = time;
		this.parentFrameId = parentFrameId;
		this.childFrameId = childFrameId;
		// WARNING! always do quat.get(matrix)
		// NEVER do mat.set(quat) as it may return NaN when quat is not normalized
		matrix.get(this.translation);
		this.rotation.set(matrix);
		// VERY IMPORTANT! without it the multiplication of matrices accumulates errors
		this.rotation.normalize();
	}

	public void set(Matrix4d matrix) {
		// WARNING! always do quat.get(matrix)
		// NEVER do mat.set(quat) as it may return NaN when quat is not normalized
		matrix.get(this.translation);
		this.rotation.set(matrix);
		// VERY IMPORTANT! without it the multiplication of matrices accumulates errors
		this.rotation.normalize();
	}

	public void get(Matrix4d matrix) {
		matrix.set(this.rotation);
		matrix.m03 = this.translation.x;
		matrix.m13 = this.translation.y;
		matrix.m23 = this.translation.z;
	}

	public void setIdentity() {
		rotation.set(0, 0, 0, 1);
		translation.set(0, 0, 0);
	}

	public void set(geometry_msgs.msg.TransformStamped rosTransform) {
		time = toSeconds(rosTransform.getHeader().getStamp());
		parentFrameId = rosTransform.getHeader().getFrameId();
		childFrameId = rosTransform.getChildFrameId();
		geometry_msgs.msg.Transform t = rosTransform.getTransform();
		geometry_msgs.msg.Vector3 vec = t.getTranslation();
		geometry_msgs.msg.Quaternion quat = t.getRotation();
		translation.set(vec.getX(), vec.getY(), vec.getZ());
		rotation.set(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	}

	public void get(geometry_msgs.msg.TransformStamped rosTransform) {
		rosTransform.getHeader().setStamp(toStamp(time));
		rosTransform.getHeader().setFrameId(parentFrameId);
		rosTransform.setChildFrameId(childFrameId);
		geometry_msgs.msg.Transform t = rosTransform.getTransform();
		geometry_msgs.msg.Vector3 vec = t.getTranslation();
		geometry_msgs.msg.Quaternion quat = t.getRotation();
		vec.setX(translation.x);
		vec.setY(translation.y);
		vec.setZ(translation.z);
		quat.setX(rotation.x);
		quat.setY(rotation.y);
		quat.setZ(rotation.z);
		quat.setW(rotation.w);
	}

	public void get(tf2_msgs.msg.TFMessage rosTfMessage) {
		geometry_msgs.msg.TransformStamped transform = new geometry_msgs.msg.TransformStamped();
		get(transform);
		rosTfMessage.getTransforms().clear();
		rosTfMessage.getTransforms().add(transform);
	}
	
	private double toSeconds(builtin_interfaces.msg.Time stamp) {
		return 1.0 * stamp.getSec() + 1e-6 * stamp.getNanosec();
	}

	private builtin_interfaces.msg.Time toStamp(double time) {
		builtin_interfaces.msg.Time stamp = new builtin_interfaces.msg.Time();
		int secs = (int)time;
		int nsecs = (int) ((time - secs) * 1000000000);
		stamp.setSec(secs);
		stamp.setNanosec(nsecs);
		return stamp;
	}

}
