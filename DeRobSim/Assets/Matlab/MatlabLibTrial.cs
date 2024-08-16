using UnityEngine;
using System.Collections;
using System;
using MathWorks.MATLAB.NET.Arrays; // import from MWArray.dll
using mycos_lib;                   // import your custom Matlab Library

public class myMatlab : MonoBehaviour {

    void Start () {
        mycos_lib.Mycos g = new mycos_lib.Mycos();  // Generate an object with your function contained within the library
        Debug.Log("Hello From mycustomLib");
        Debug.Log(g.mycos(1,95).GetValue(0));       // Call the function
    }
}