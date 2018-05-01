using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine;


public class UNOConnect : MonoBehaviour {
    public GameObject Endo;
    public GameObject EndoTube;
    public GameObject rotator;
    public Vector3 rotationOffset;
    public bool setOrigin = false;
    Quaternion rotOffset = Quaternion.identity;
    public float Speed = 10f;
    public float pmeterSpeed = 0.2f;
    float pmeterOld;
    float pmeterNow;
    SerialPort stream = new SerialPort("COM4", 38400);
    float[] q = new float[8];
    float accelX = 0f;
    float accelY = 0f;
    float accelZ = 0f;
    float pmeter = 0f;
    Quaternion rotate1;
    Vector3 dir;
    string[] massQ;
    string value;
    bool isUpdated = false;
    Vector3 lastAcc;
    Vector3 linAcc;
    //float lastAcc;
    //float linAcc;
    public Vector3 linearAcceleration;
    // Use this for initialization

    const float kFilteringFactor = 0.1f; //0.1f

    public Vector3 A1;
    public Vector3 A2;
    public Vector3 A2ramping; // for the low-pass filter
    public Vector3 V1;
    public Vector3 V2;
    Quaternion origin;

    public int SpeedFactor = 1000; //this factor is for increasing acceleration to move in unity world

    void resetAll()
    {
        Debug.Log("Reset all");
        A2 = Vector3.zero;
        V1 = Vector3.zero;
        V2 = Vector3.zero;
        A2ramping = Vector3.zero;
        origin = rotator.transform.localRotation;
    }

    void Start () {
        origin = Endo.transform.localRotation;
        resetAll();
        stream.Open();
    }

    Vector3 ramping(Vector3 A)
    {
        A2ramping = A * kFilteringFactor + A2ramping * (1.0f - kFilteringFactor);
        return A - A2ramping;
    }

    void getAcceleration(float deltaTime)
    {
        A1 = A2;
        A2 = ramping(dir) * SpeedFactor;

        V2 = V1 + (A2 - A1) * deltaTime;

        V1 = V2;
    }
    // Update is called once per frame
    void Update() {
        if(setOrigin)
        {
            rotOffset = Endo.transform.rotation;
            setOrigin = false;
        }
        if (Input.GetKeyDown(KeyCode.Alpha1)){
            setOrigin = true;
        }
        //    if (!isUpdated)
        //    {
        //    StartCoroutine(GetInfo());
        //}
        //isUpdated = true;
        //stream.WriteLine("s");
        //yield return new WaitForSeconds(0.1f);
        value = stream.ReadLine();
        if (value != null)
        {
            
            massQ = value.Split(',');
            q[0] = float.Parse(massQ[0]);
            q[1] = float.Parse(massQ[1]);
            q[2] = float.Parse(massQ[2]);
            q[3] = float.Parse(massQ[3]);
            q[4] = float.Parse(massQ[4]);
            q[5] = float.Parse(massQ[5]);
            q[6] = float.Parse(massQ[6]);
            q[7] = float.Parse(massQ[7]);
            //rotate1 = new Quaternion(-q[0], -q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], q[1], q[2], -q[3]);
            //Quaternion rotate2 = origin * rotate1;

            //rotate1 = new Quaternion(q[0], q[1], q[2], q[3]);
            //rotate1 = new Quaternion(q[0], q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], q[1], -q[2], q[3]);
            //rotate1 = new Quaternion(q[0], q[1], -q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], -q[1], q[2], q[3]);
            //rotate1 = new Quaternion(q[0], -q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], -q[1], -q[2], q[3]);
            //rotate1 = new Quaternion(q[0], -q[1], -q[2], -q[3]);
            //rotate1 = new Quaternion(-q[0], q[1], q[2], q[3]);
            //rotate1 = new Quaternion(-q[0], q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(-q[0], q[1], -q[2], q[3]);
            //1 rotate1 = new Quaternion(-q[0], q[1], -q[2], -q[3]);
            //rotate1 = new Quaternion(-q[0], -q[1], q[2], q[3]);
            //rotate1 = new Quaternion(-q[0], -q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(-q[0], -q[1], -q[2], q[3]);
            //rotate1 = new Quaternion(-q[0], -q[1], -q[2], -q[3]);

            //rotate1 = new Quaternion(q[0], q[1], q[2], -q[3]);
            //rotate1 = new Quaternion(q[0], q[2], q[1], -q[3]);
            //rotate1 = new Quaternion(q[0], q[3], q[2], -q[1]);
            //rotate1 = new Quaternion(q[0], q[2], q[3], -q[1]);
            //rotate1 = new Quaternion(q[0], q[1], q[3], -q[2]); //working


            rotate1 = new Quaternion(q[0], q[1], q[3], -q[2]);
            //rotate1 = new Quaternion(q[3], q[0], q[1], q[2]);

            //rotate1 = new Quaternion(q[0], q[3], q[1], -q[2]);
            accelX = q[4];
            accelY = q[5];
            accelZ = q[6];
            accelZ += 9.8f;
            pmeter = 0f;
            pmeterNow = q[7];
            //Debug.Log(q[7]);
            if (pmeterNow != pmeterOld)
            {
                pmeter = pmeterNow - pmeterOld;
                pmeterOld = pmeterNow;
            }
            else
            {
                pmeter = 0f;
            }
            //Debug.Log(pmeter);


            //Endo.transform.rotation = Quaternion.Lerp(Endo.transform.rotation, rotate1, Time.deltaTime * Speed) * Quaternion.Inverse(rotOffset);
            Endo.transform.localRotation = Quaternion.Lerp(Endo.transform.localRotation, rotate1, Time.deltaTime * Speed);
            Endo.transform.parent.transform.eulerAngles = rotationOffset;
            //Endo.transform.rotation = rotate1* Quaternion.Inverse(rotOffset);

            //Endo.transform.rotation = Quaternion.Inverse(rotate1);
            //Endo.transform.localRotation = rotate1 * origin;
            //Endo.transform.localRotation = Quaternion.Lerp(Endo.transform.rotation, rotate1, Time.deltaTime * Speed);
            EndoTube.transform.Translate(Vector3.up * pmeter * pmeterSpeed);
            //EndoTube.transform.Translate(Vector3.up * pmeter * pmeterSpeed);
            //Endo.transform.Translate(0, (-1 * accelZ * Speed * Time.deltaTime), 0 );
            //Endo.transform.position = (Vector3.forward * accelX * Time.deltaTime);

            //Endo.transform.Translate(0, 0, (-1 * accelX * Speed * Time.deltaTime));
            //dir = Vector3.zero;
            //dir.z = accelY;
            //dir.z = accelX;
            //dir.x = -accelY;
            //dir.y = -accelZ;
            //linAcc = dir - lastAcc * Time.deltaTime;
            //lastAcc = linAcc;
            //Debug.Log("LinAcc" + linAcc);
            //linAcc = (dir.z - lastAcc) * Speed * Time.deltaTime;
            //linAcc = dir - lastAcc * Time.deltaTime;
            //if (linAcc.sqrMagnitude > 1)
            //linAcc.Normalize();
            //lastAcc = linAcc * Time.deltaTime;
            //linAcc = linAcc * Time.deltaTime;

            //if (linAcc < lastAcc)
            //Endo.transform.Translate(0, 0, (-1 * linAcc * Time.deltaTime));
            //Endo.transform.position += (Vector3.forward * lastAcc * Time.deltaTime);
            //Endo.transform.position += (Vector3.forward * linAcc * Speed * Time.deltaTime);
            //dir.z = accelZ;
            //if (dir.sqrMagnitude > 1.1f)
            //    dir.Normalize();
            //dir *= Time.deltaTime;
            //Endo.transform.Translate(linAcc * Speed * Time.deltaTime);

            //getAcceleration(Time.deltaTime);
            //float distance = -1f;
            //Vector3 newPos = Endo.transform.position;
            //Endo.transform.Translate(Vector3.forward * Time.deltaTime * V2.z * distance);
        }
        //yield return new WaitForSeconds(0.1f);
        //Debug.Log(value);
        //Debug.Log(q[4]);
        //isUpdated = false;
    }

    //IEnumerator GetInfo()
    //{
    //    isUpdated = true;
    //    stream.WriteLine("s");
    //    //yield return new WaitForSeconds(0.1f);
    //    value = stream.ReadLine();
    //    if (value != null)
    //    {
    //        massQ = value.Split(',');
    //        q[0] = float.Parse(massQ[0]);
    //        q[1] = float.Parse(massQ[1]);
    //        q[2] = float.Parse(massQ[2]);
    //        q[3] = float.Parse(massQ[3]);
    //        q[4] = float.Parse(massQ[4]);
    //        q[5] = float.Parse(massQ[5]);
    //        q[6] = float.Parse(massQ[6]);
    //        rotate1 = new Quaternion(q[0], q[1], q[2], q[3]);
    //        accelX = q[4];
    //        accelY = q[5];
    //        accelZ = q[6];
    //        accelZ += 9.8f;
    //        Endo.transform.rotation = rotate1;
    //        //Endo.transform.position = (Vector3.forward * accelX * Time.deltaTime);
    //        //Endo.transform.position += (Vector3.forward * accelX * Time.deltaTime);
    //        //Endo.transform.Translate(0, 0, (-1 * accelZ * Speed * Time.deltaTime));
    //        //dir = Vector3.zero;
    //        //dir.x = -accelY;
    //        //dir.y = accelX;
    //        //dir.z = accelZ;
    //        //if (dir.sqrMagnitude > 1)
    //        //    dir.Normalize();
    //        //dir *= Time.deltaTime;
    //        //Endo.transform.Translate(dir * Speed);
    //    }
    //    yield return new WaitForSeconds(0.1f);
    //    Debug.Log(value);
    //    isUpdated = false;
    //}

    //IEnumerator GetAccel()
    //{
    //    isUpdated = true;
    //    value = stream.ReadLine();
    //    if (value != null)
    //    {
    //        accelX = float.Parse(value);
    //        Endo.transform.position += (Vector3.forward * accelX * Time.deltaTime);
    //    }
    //    yield return new WaitForSeconds(0.05f);
    //    isUpdated = false;
    //}
}
