package org.ros2.java.tf;

import java.util.ArrayList;
import java.util.List;

public class TransformList {

	public ArrayList<Transform> transforms = new ArrayList<>();

	public TransformList() {
	}

	public TransformList(tf2_msgs.msg.TFMessage tfMessage) {
		set(tfMessage);
	}

	public void set(tf2_msgs.msg.TFMessage tfMessage) {
		List<geometry_msgs.msg.TransformStamped> rosTransforms = tfMessage.getTransforms();
		for (geometry_msgs.msg.TransformStamped rosTransform : rosTransforms) {
			transforms.add(new Transform(rosTransform));
		}
	}

	public void get(tf2_msgs.msg.TFMessage tfMessage) {
		tfMessage.getTransforms().clear();
		for (Transform t : transforms) {
			geometry_msgs.msg.TransformStamped transform = new geometry_msgs.msg.TransformStamped();
			t.get(transform);
			tfMessage.getTransforms().add(transform);
		}
	}
}
