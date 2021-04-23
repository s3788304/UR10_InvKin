using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LA 
{
	//linear algebra

	public static float Dist(float[] a, float[] b)
	{
		float r = 0;
		for (int i = 0; i < a.Length; i++)
			r += (a[i] - b[i]) * (a[i] - b[i]);
		return Mathf.Sqrt(r);
	}

	public static float[] Sub(float[] a, float[] b)
	{
		float[] re = new float[a.Length];
		for (int i = 0; i < a.Length; i++)
			re[i] = a[i] - b[i];
		return re;
	}

	public static float Mag(float[] a)
	{
		float r = 0;
		for (int i = 0; i < a.Length; i++)
			r += a[i] * a[i];
		r = Mathf.Sqrt(r);
		return r;
	}

	public static float[] Normalize(float[] a)
	{
		float r = Mag(a);
		float[] re = new float[a.Length];
		for (int i = 0; i < a.Length; i++)
			re[i] = a[i] / r;
		return re;
	}

	public static float Dot(float[] a, float[] b)
	{
		float re = 0;
		for (int i = 0; i < a.Length; i++)
			re += a[i] * b[i];
		return re;
	}

	public static float[] Mul(float[] a, float s)
	{
		float[] re = new float[a.Length];
		for (int i = 0; i < a.Length; i++)
			re[i] = a[i] * s;
		return re;
	}

	public static float[] Cross(float[] b, float[] c)
	{
		float[] re = new float[b.Length];
		re[0] = b[1] * c[2] - b[2] * c[1];
		re[1] = b[2] * c[0] - b[0] * c[2];
		re[2] = b[0] * c[1] - b[1] * c[0];
		return re;
	}
}
