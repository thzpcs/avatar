using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//makes the texture of the object the script is applied to

public class webcamScript : MonoBehaviour {

	// Use this for initialization
	void Start () {
        WebCamTexture webcamTexture = new WebCamTexture();
        Renderer renderer = GetComponent<Renderer>();
        renderer.material.mainTexture = webcamTexture;
        webcamTexture.Play();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
