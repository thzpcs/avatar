using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Threading;
using UnityEngine;

//A simple local HTTP server used to communicate the VR headset facing information.
//https://www.codeproject.com/Tips/485182/Create-a-local-server-in-Csharp tutorial followed for help with creating the server (licensed under CPOL)

public class Server : MonoBehaviour {

	public static double xdir;
	public static double ydir;
	public static double zdir;
    static HttpListener _httpListener = new HttpListener();

    // Use this for initialization
    void Start () {
        Vector3 forward = transform.forward;
		xdir = forward.x;
		ydir = forward.y;
		zdir = forward.z;
		print("Starting server...");
        _httpListener.Prefixes.Add("http://127.0.0.1:5000/");
        _httpListener.Start();
        print("Server started sucessfully.");
		Thread _responseThread = new Thread(ServerResponseListener);
        _responseThread.Start();
    }

	void ServerResponseListener()
    {
        while (true)
        {
			string text = Server.xdir.ToString()+ " " + Server.ydir.ToString() + " " + Server.zdir.ToString();
			HttpListenerContext context = _httpListener.GetContext();
            byte[] _responseArray = System.Text.Encoding.UTF8.GetBytes(text);
            context.Response.OutputStream.Write(_responseArray, 0, _responseArray.Length);
            context.Response.Close();
            print("Respone given to a request.");
        }
    }

    // Update is called once per frame
    void Update () {
		xdir = transform.eulerAngles.x;
		ydir = transform.eulerAngles.y;
		zdir = transform.eulerAngles.z;
	}
}
