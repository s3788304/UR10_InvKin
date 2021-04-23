using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Robot 
{
    private static readonly float PI = Mathf.PI;
	private static readonly float HP = 0.5f * Mathf.PI;
	private readonly float[] a;
	private readonly float[] d;
	private readonly float l2;
	private readonly float ad2;

	public Robot(float[] a, float[] d) 
	{ // a,d: DH parameters, corresponding to the geometry of your robot
		this.a=a;
		this.d=d;
		l2 = Mathf.Sqrt((float)(a[2] * a[2] + d[3] * d[3]));
		ad2 = Mathf.Atan2(a[2], d[3]);
	}
	
	public float[] Inverse(float[] v, Double A4_deg, float[][] baseP, float[][] tool)
	{
		float[][] Pos = Matrix(v[0], v[1], v[2], v[3], v[4], v[5]);
		float[][] goal;
		if(baseP==null)
			goal=Pos;
		else
			goal= Mul34(baseP, Pos); // in WORLD frame baseP*POS(in baseP)= GOAL
		float[][] T6;
		if (tool == null) {
			T6 = goal;
		} else {
			float[][] intool = Inverse34(tool);
			T6 = Mul34(goal, intool); // T6*TOOL = GOAL= baseP*POS(in baseP)
		}
		float[] inreDeg = Inverse(T6, A4_deg);// (T6, thetaDeg[3]);
		return inreDeg;
	}
	
	public float[][] Forward(float[] degs) {  
		float[] ts= Deg_rad(degs);
		float[] c=new float[6];
		float[] s=new float[6];
		for(int i=0;i<6;i++){
			c[i]= Mathf.Cos(ts[i]);
			s[i]= Mathf.Sin(ts[i]);
		}
		float[][] m123=new float[3][];
		m123[0] = new float[] { c[0] * (c[1] * c[2] - s[1] * s[2]),     s[0],   c[0] * (c[1] * s[2] + s[1] * c[2]),      c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] };
		m123[1] = new float[] { s[0]*  (c[1] * c[2] - s[1] * s[2]),   -c[0],    s[0] * (c[1] * s[2] + s[1] * c[2]),      s[0]* (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0]};
		m123[2] = new float[] { s[1] * c[2] +c[1] * s[2],                0,    s[1] * s[2] - c[1] * c[2],                 a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] +d[0]};
		float[][] m456=new float[3][];
		m456[0] = new float[] {c[3]*c[4]*c[5]-s[3]*s[5],   -c[3]*c[4]*s[5]-s[3]*c[5]     , c[3] * s[4],     c[3]*s[4]*d[5] };
		m456[1] = new float[] {s[3]*c[4]*c[5]+c[3]*s[5],   -s[3]*c[4]*s[5]+c[3]*c[5]     , s[3] * s[4],     s[3]*s[4]*d[5] };
		m456[2] = new float[] { -s[4]*c[5],                  s[4]*s[5],                     c[4],             c[4]*d[5]+d[3]};
		float[][] arr = Mul34(m123, m456);
		return arr;
	}
	public float[][][] ForwardSequence(float[] degs) {
		float[] ts= Deg_rad(degs);
		float[] c=new float[6];
		float[] s=new float[6];
		for(int i=0;i<6;i++){
			c[i]= Mathf.Cos(ts[i]);
			s[i]= Mathf.Sin(ts[i]);
		}
        float[][] a0=new float[3][]; 
        a0[0]=new float[]{c[0]  , 0   ,   s[0]  ,  a[0]*c[0]};
        a0[1]=new float[]{s[0] ,  0   ,  -c[0]  ,   a[0]*s[0]};
        a0[2]=new float[]{0     ,  1    ,   0      , d[0]};
        float[][] a1=new float[3][]; 
        a1[0]=new float[]{c[1] ,  -s[1]  ,  0  ,   a[1]*c[1]};
        a1[1]=new float[]{s[1] ,  c[1]   ,  0  ,   a[1]*s[1]};
        a1[2]=new float[]{0     ,  0    ,   1      , 0};
        float[][] a2=new float[3][]; 
        a2[0]=new float[]{c[2],    0 ,     s[2] ,    a[2]*c[2]};
        a2[1]=new float[]{s[2],    0 ,    -c[2],     a[2]*s[2]};
        a2[2]=new float[]{0      ,  1    ,   0      ,     0};
        float[][] a3=new float[3][]; 
        a3[0]=new float[]{c[3] ,    0   ,  -s[3] ,     0};
        a3[1]=new float[]{s[3]  ,   0   ,   c[3] ,     0};
        a3[2]=new float[]{0     ,  - 1    ,    0   ,   d[3]};
        float[][] a4=new float[3][]; 
        a4[0]=new float[]{c[4]  ,   0  ,    s[4]   ,    0};
        a4[1]=new float[]{s[4]  ,   0  ,   -c[4]  ,     0};
        a4[2]=new float[]{0      ,   1     ,   0     ,   0};
        float[][] a5=new float[3][]; 
        a5[0]=new float[]{c[5] ,   -s[5] ,     0  ,    0};
        a5[1]=new float[]{s[5] ,    c[5] ,    0   ,  0};
        a5[2]=new float[]{0       ,   0     ,    1   ,  d[5]};
        float[][][] M=new float[6][][];
		M[0] = a0;
		M[1] = Mul34(M[0],  a1);
		M[2] = Mul34(M[1],  a2);
		M[3] = Mul34(M[2],  a3);
		M[4] = Mul34(M[3],  a4);
		M[5] = Mul34(M[4],  a5);
		return M;
	}
	public float[] Forward(float[]  degs, float[][] baseP, float[][] tool) {
		float[][] T6 = Forward(degs);
		float[][] goal;
		if (tool == null) {
			goal = T6;
		} else {
			goal = Mul34(T6, tool);
		}
		float[][] pos;
		if (baseP == null) {
			pos = goal;
		} else {
			float[][] inbaseP = Inverse34(baseP);
			pos = Mul34(inbaseP, goal);
		}
		float[] ass = Robot.ABC(pos);
		return new float[] { pos[0][3], pos[1][3], pos[2][3], ass[0], ass[1], ass[2] };
	}

	public float[] Inverse(float[][] T6, Double A4_deg) 
	{
		float[] theta=new float[9]; 
		float[] center= Mul34(T6, new float[]{0,0, -d[5]});
		theta[0] = Mathf.Atan2(center[1], center[0]); // or -atan2     choice one possibility

		float ll = Mathf.Sqrt(center[0] * center[0] + center[1] * center[1]);
		float[] p1 = { a[0] * center[0] / ll, a[0] * center[1] / ll, d[0] };
		float l3 =LA.Dist(center, p1);
		float l1 = a[1];
		float beta = Mathf.Acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3));
		float ttl = Mathf.Sqrt((center[0] - p1[0]) * (center[0] - p1[0]) + (center[1] - p1[1]) * (center[1] - p1[1]));
		if (p1[0] * (center[0] - p1[0]) < 0) // opposite side
			ttl = -ttl;
		float al = Mathf.Atan2(center[2] - p1[2], ttl);
		theta[1] =beta+al; // choice one possibility
		float gama = Mathf.Acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2));
		theta[2] = gama - ad2 - HP;

		float[][] arr = new float[4][];
		float[] c = new float[3];
		float[] s=new float[3];
		for(int i=0;i<3;i++){
			c[i]= Mathf.Cos(theta[i]);
			s[i]= Mathf.Sin(theta[i]);
		}
		arr[0] = new float[] { c[0] * (c[1] * c[2] - s[1] * s[2]),     s[0],   c[0] * (c[1] * s[2] + s[1] * c[2]),      c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] };
		arr[1] = new float[] { s[0]*  (c[1] * c[2] - s[1] * s[2]),   -c[0],    s[0] * (c[1] * s[2] + s[1] * c[2]),      s[0]* (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0]};
		arr[2] = new float[] { s[1] * c[2] +c[1] * s[2],                0,    s[1] * s[2] - c[1] * c[2],                 a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] +d[0]};
		float[][] in123= Inverse34(arr);
		float[][] mr = Mul34(in123, T6);
		float c5 = mr[2][2];
		if (Mathf.Abs(c5 - 1) < 0.000001) { //Singularity
			float A4= (float)(-PI*A4_deg/180f); //see deg_rad
			float c4 = Mathf.Cos(A4);
			float s4 = Mathf.Sin(A4);
			float s6 = c4 * mr[1][0] - s4 * mr[0][0];
			float c6;
			if (Mathf.Abs(c4) > Mathf.Abs(s4))
				c6 = (mr[0][0] + s4 * s6) / c4;
			else
				c6 = (mr[1][0] - c4 * s6) / s4;
			theta[3] = A4;
			theta[4] = 0;
			theta[5] =   Mathf.Atan2(s6, c6);
			if (Mathf.Abs(c6) > 1 || Mathf.Abs(s6) > 1)
				Debug.LogError("Error Line 164");
		} else {
			float ang = Mathf.Atan2(mr[1][2], mr[0][2]);
			theta[3] = ang;
			theta[4] = Mathf.Acos(c5); // *********
			theta[5] = Mathf.Atan2(mr[2][1], -mr[2][0]);
		}
		float[] inreDeg = Rad_deg(theta);
		return inreDeg;
	}

	private float[] Deg_rad(float[] ds) {
		float[] rd = new float[6];
		for (int i = 0; i < 6; i++)
			rd[i] = ds[i] * PI / 180;
		rd[2] -= HP;
		rd[5] += PI;
		for (int i = 0; i < 6; i++)
			rd[i] = -rd[i];
		return rd;
	}

	private float[] Rad_deg(float[] ds) {
		float[] rd = new float[6];
		for (int i = 0; i < 6; i++)
			rd[i] = -ds[i];
		rd[2] += HP;
		rd[5] -= PI;
		for (int i = 0; i < 6; i++)
			rd[i] = rd[i] * 180 / PI;
		return rd;
	}
	
	public static float[][] Mul34(float[][] a, float[][] b) { // multiple two 3*4 matrices
		float[][] re = new float[3][] {
			new float[4],
			new float[4],
			new float[4]
		};
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				float b3j = (j == 3 ? 1 : 0);
				re[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b3j;
			}
		}
		return re;
	}

	public static float[] Mul34(float[][] a, float[] b) { // multiple a 3*4 Matrix with a vector
		float[] re = new float[3];
		for (int i = 0; i < 3; i++)
			re[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2] + a[i][3];
		return re;
	}
	
	public static float[][] Matrix(float x, float y, float z, float aDeg, float bDeg, float cDeg) {
		float a = -aDeg * PI / 180;
		float b = -bDeg * PI / 180;
		float c = -cDeg * PI / 180;
		float ca = Mathf.Cos(a);
		float sa = Mathf.Sin(a);
		float cb = Mathf.Cos(b);
		float sb = Mathf.Sin(b);
		float cc = Mathf.Cos(c);
		float sc = Mathf.Sin(c);
		float[][] tt = new float[3][];
		tt[0] = new float[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc, x };
		tt[1] = new float[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc, y };
		tt[2] = new float[] { sb, -cb * sc, cb * cc, z };
		return tt;
	}
	
	public static float[][] Matrix(float aDeg, float bDeg, float cDeg) {
		float a = -aDeg * PI / 180;
		float b = -bDeg * PI / 180;
		float c = -cDeg * PI / 180;
		float ca = Mathf.Cos(a);
		float sa = Mathf.Sin(a);
		float cb = Mathf.Cos(b);
		float sb = Mathf.Sin(b);
		float cc = Mathf.Cos(c);
		float sc = Mathf.Sin(c);
		float[][] tt = new float[3][];
		tt[0] = new float[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc };
		tt[1] = new float[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc};
		tt[2] = new float[] { sb, -cb * sc, cb * cc };
		return tt;
	}
	
	public static float[] ABCby3Point(float[] _dx, float[] _dxy) {//_dx:  x-axis    _dxy: a vector  on XY plane,     
		float[] dx= LA.Normalize(_dx);
		float[] tt= LA.Mul(dx, LA.Dot(_dxy, dx));
		float[] _dy= LA.Sub(_dxy, tt);
		float[] dy= LA.Normalize(_dy);
		float[] dz= LA.Cross(dx, dy);
				
		float cacb=dx[0];
		float sacb=-dx[1];
		float sb=dx[2];
		float cbsc=-dy[2];
		float cbcc= dz[2];
	
		float cb = Mathf.Sqrt(1 - sb * sb); // + -  similar to ABC()
		float a = Mathf.Atan2(sacb, cacb) * -180 /PI;
		float b = Mathf.Atan2(sb, cb) * -180 / PI;
		float c = Mathf.Atan2(cbsc, cbcc) * -180 / PI;
		return new float[] { a, b, c };
	}

	public static float[] ABC(float[][] m) { //Euler angles from 3*3 Matrix
		float sb = m[2][0];
		float cb = Mathf.Sqrt(1 - sb * sb); // + -
		float ca = m[0][0];
		float sa = -m[1][0];
		float cc = m[2][2];
		float sc = -m[2][1];
		float a = Mathf.Atan2(sa, ca) * -180 /PI;
		float b = Mathf.Atan2(sb, cb) * -180 / PI;
		float c = Mathf.Atan2(sc, cc) * -180 / PI;
		return new float[] { a, b, c };
	}
	public static float[] FlipABC( float[] abc){
		return FlipABC(abc[0], abc[1], abc[2]);
	}
	public static float[] FlipABC( float a, float b, float c){
		float na= a>0? (a-180):(a+180);
		float nb= b>0?  (180-b):(-180-b);
		float nc= c>0? (c-180):(c+180);
		return new float[]{na, nb, nc};
	}
	public static float[][] Inverse34(float[][] m) { // det=1,  row 3 is {0,0,0,1}
		float[][] v = new float[3][]{
			new float[4],
			new float[4],
			new float[4]
		};
		v[0][0] = -m[1][2] * m[2][1] + m[1][1] * m[2][2];
		v[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
		v[0][2] = -m[0][2] * m[1][1] + m[0][1] * m[1][2];
		v[0][3] = m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] - m[0][1]
				* m[1][2] * m[2][3];
		v[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
		v[1][1] = -m[0][2] * m[2][0] + m[0][0] * m[2][2];
		v[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
		v[1][3] = m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] + m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] + m[0][0]
				* m[1][2] * m[2][3];
		v[2][0] = -m[1][1] * m[2][0] + m[1][0] * m[2][1];
		v[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
		v[2][2] = -m[0][1] * m[1][0] + m[0][0] * m[1][1];
		v[2][3] = m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] - m[0][0]
				* m[1][1] * m[2][3];
		return v;
	}
}
