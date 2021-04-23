using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class IKManager : MonoBehaviour
{
    public GameObject component1;
    public GameObject endEffector1;


    private GameObject component;
    private GameObject endEffector;

    public GameObject[] armAxes;
    
    //Robot Class
    private Robot kr;

    //IK Variables
    float[][] tool;
    float[][] baseP;
    float[] XYZABC;

    //Output Angles
    float[] degs;
    void Start()
    {
        float[] a = { 0, -0.612f, -0.572f, 0, 0, 0 };     //here are the parameters of UR10 Robot from unity
        float[] d = { 0.128f, 0, 0, 0.103f, 0.116f, 0.3035f };
        //float[] d = { 0.128f, 0, 0, 0.103f, 0.116f, 92.2f };

        //float[] a = { 0, -0.612f, -0.5723f, 0, 0, 0 };     //here are the parameters of UR10 Robot in m
        //float[] d = { 0.1273f, 0, 0, 0.163941f, 0.1157f, 0.0922f };

        //float[] a = { 0, -612f, -572.3f, 0, 0, 0 };     //here are the parameters of UR10 Robot in mm
        //float[] d = { 127.3f, 0, 0, 163.941f, 115.7f, 92.2f };

        //Copy of components to rotate
        component = component1;
        endEffector = endEffector1;

        //Rotate along X to bring to normal axis
       
        //endEffector.transform.localRotation = Quaternion.AngleAxis(90f, Vector3.left);

        kr = new Robot(a, d);

        //tool = Robot.Matrix(endEffector.transform.TransformPoint(Vector3.zero).x, endEffector.transform.TransformPoint(Vector3.zero).z, -endEffector.transform.TransformPoint(Vector3.zero).y, endEffector.transform.eulerAngles.x, endEffector.transform.eulerAngles.y, endEffector.transform.eulerAngles.z);
        //tool = Robot.Matrix(endEffector.transform.position.x, endEffector.transform.position.z, -endEffector.transform.position.y, endEffector.transform.eulerAngles.x, endEffector.transform.eulerAngles.z, -endEffector.transform.eulerAngles.y);
        //tool = Robot.Matrix(0f, 0.4675004f, -1.428001f, 90f, -180f, 0f); //in m
        tool = null;
        


       //baseP = Robot.Matrix(transform.position.x, transform.position.y, transform.position.z, transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, transform.rotation.eulerAngles.z);
        //baseP = Robot.Matrix(898.094f, -1265.699f, 245.752f, 161.956f, -11f, 22f);
        baseP = null;

        //float[] tempPos = { component.transform.position.x, component.transform.position.z, -component.transform.position.y, component.transform.rotation.eulerAngles.x, component.transform.rotation.eulerAngles.z, -component.transform.rotation.eulerAngles.y};
        //float[] tempPos = { 97.16f, -161.65f, 577.30f, -7.7f, 6.19f, 168.76f };
        //XYZABC = tempPos;
    }

    // Update is called once per frame
    void Update()
    {
        float[] tempPos = { component.transform.position.x, component.transform.position.y, component.transform.position.z, component.transform.rotation.eulerAngles.x, component.transform.rotation.eulerAngles.y, component.transform.rotation.eulerAngles.z };
        //float[] tempPos = { 0.4f, 0.4f, 0f, 0f, 0f, 0f};
        XYZABC = tempPos;
        

        degs = kr.Inverse(XYZABC, 0.0f , baseP, tool);    //A1-A6, joint angles

        for (int j = 0; j < 6; j++)
            Debug.Log(" " + degs[j] + ", ");
        Debug.Log(" \n");

        RotateAxis();
    }

    //Rotate arm axes
    void RotateAxis()
    {
        armAxes[0].transform.localRotation = Quaternion.AngleAxis(degs[0] + 95.791f, armAxes[0].GetComponent<Axis>().rotationAxisY);
        armAxes[1].transform.localRotation = Quaternion.AngleAxis(degs[1] + 178.417f, armAxes[1].GetComponent<Axis>().rotationAxis);
        armAxes[2].transform.localRotation = Quaternion.AngleAxis(degs[2] - 65.174f, armAxes[2].GetComponent<Axis>().rotationAxis);
        armAxes[3].transform.localRotation = Quaternion.AngleAxis(degs[3] - 147.849f, armAxes[3].GetComponent<Axis>().rotationAxis);
        armAxes[4].transform.localRotation = Quaternion.AngleAxis(degs[4] + 74.351f, armAxes[4].GetComponent<Axis>().rotationAxisY);
    }

    void RotateAxisGlobal()
    {
        armAxes[0].transform.rotation = Quaternion.AngleAxis(degs[0], armAxes[0].GetComponent<Axis>().rotationAxisY);
        armAxes[1].transform.rotation = Quaternion.AngleAxis(degs[1], armAxes[1].GetComponent<Axis>().rotationAxis);
        armAxes[2].transform.rotation = Quaternion.AngleAxis(degs[2], armAxes[2].GetComponent<Axis>().rotationAxis);
        armAxes[3].transform.rotation = Quaternion.AngleAxis(degs[3], armAxes[3].GetComponent<Axis>().rotationAxis);
        armAxes[4].transform.rotation = Quaternion.AngleAxis(degs[4], armAxes[4].GetComponent<Axis>().rotationAxisY);
    }
}
