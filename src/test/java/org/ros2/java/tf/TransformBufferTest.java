package org.ros2.java.tf;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Matrix4d;

import org.junit.Ignore;
import org.junit.Test;
import org.ros2.java.tf.internal.TransformBuffer;

public class TransformBufferTest {

	@Test
	public void test() {
		boolean exists;
		TransformBuffer tb = new TransformBuffer(false);

		// compute matrices
		Matrix4d mat1 = new Matrix4d();
		mat1.setIdentity();
		mat1.m03 = 1;

		Matrix4d mat2 = new Matrix4d();
		mat2.setIdentity();
		mat2.m03 = 2;

		// add transforms
		tb.addTransform(1, mat1);
		tb.addTransform(2, mat2);

		// test getting proper matrices
		Matrix4d res = new Matrix4d();

		exists = tb.getTransform(0.9, res);
		assertFalse(exists);

		exists = tb.getTransform(1.0, res);
		assertTrue(exists);
		assertEquals(1.0, res.m03, 1e-6);

		exists = tb.getTransform(1.5, res);
		assertTrue(exists);
		assertEquals(1.5, res.m03, 1e-6);

		exists = tb.getTransform(2.0, res);
		assertTrue(exists);
		assertEquals(2.0, res.m03, 1e-6);

		exists = tb.getTransform(2.1, res);
		assertFalse(exists);

		exists = tb.getTransformLatest(res);
		assertTrue(exists);
		assertEquals(2.0, res.m03, 1e-6);
	}

	@Test
	@Ignore
	public void testPerformance() {
		TransformBuffer tb = new TransformBuffer(false);
		Matrix4d mat1 = new Matrix4d();
		Matrix4d mat2 = new Matrix4d();
		for(int i=0; i<2000; i++) {
			tb.addTransform(i, mat1);
		}
		
		for(int i=0; i<1000; i++) {
			tb.getTransform(0.1, mat2);
		}
		long t1 = System.nanoTime();
		for(int i=0; i<1000; i++) {
			tb.getTransform(0.1, mat2);
		}
		long t2 = System.nanoTime();
		System.out.println("Average execution time: " + (t2-t1)*1e-12);
	}
}
