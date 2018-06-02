using AOT;
using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;


public class imagineARController : MonoBehaviour
{
	public Texture2D image;
	public TextAsset database;

	public bool debugImage = false;


	[DllImport("imagineARPlugin")]
	private static extern void OpenWebcam(out int w, out int h);

	[DllImport("imagineARPlugin")]
	private static extern void CloseWebcam();

	[DllImport ("imagineARPlugin")]
	private static extern void SetWebcamTexture(IntPtr texture, int w, int h);

	[DllImport("imagineARPlugin")]
	private static extern void DebugShowTexture();

	[DllImport("imagineARPlugin")]
	private static extern IntPtr GetRenderEventFunc();

	[DllImport("imagineARPlugin")]
	private static extern int BuildImageTargetDatabase (Color32[] img, int width, int height, string imgName);

	[DllImport("imagineARPlugin")]
	private static extern int InitImageTarget (string imgName);

	[DllImport("imagineARPlugin")]
	private static extern void Train();

	[DllImport("imagineARPlugin", CallingConvention = CallingConvention.Cdecl)]
	static extern void RegisterImageTargetFoundCallback(imageTargetFoundCallback cb);
	delegate void imageTargetFoundCallback();

	[DllImport("imagineARPlugin", CallingConvention = CallingConvention.Cdecl)]
	static extern void RegisterImageTargetLostCallback(imageTargetLostCallback cb);
	delegate void imageTargetLostCallback();

	[DllImport("imagineARPlugin", CallingConvention = CallingConvention.Cdecl)]
	static extern void RegisterImageTargetTrackedCallback(imageTargetTrackedCallback cb);
	delegate void imageTargetTrackedCallback(IntPtr tMat);


	private static imagineARController _instance;
	public static imagineARController Instance{
		get{
			if (_instance == null) {
				GameObject.FindObjectOfType<imagineARController> ();
			}

			return _instance;
		}
	}

	void Start()
	{
		InitWebcam ();
	}

	WebCamTexture webcamTexture;


	private void InitWebcam()
	{
		int width = 0, height = 0;
		OpenWebcam(out width, out height);

		Debug.Log("Initialize Webcam with dimensions[" + width + "," + height + "]" );

		Texture2D tex = new Texture2D (width, height, TextureFormat.RGBA32, false);
		tex.filterMode = FilterMode.Point;
		tex.Apply ();

		ARCamera.Instance.videoBackground.material.mainTexture = tex;


		// Pass texture pointer to the plugin
		SetWebcamTexture (tex.GetNativeTexturePtr(), tex.width, tex.height);
		// Register Callbacks
		RegisterImageTargetFoundCallback   ( OnImageTargetFound );
		RegisterImageTargetLostCallback    ( OnImageTargetLost );
		RegisterImageTargetTrackedCallback ( OnImageTargetTracked );



		//Initialize Imagetarget
		Initialize();
		Train ();

		updatePlugin = true;
		StartCoroutine ("UpdateRoutine");
	}

	bool updatePlugin = false;

//	void Update(){
//		if (updatePlugin) {
//			GL.IssuePluginEvent (GetRenderEventFunc (), 1);
//			if(debugImage)
//				DebugShowTexture ();
//		}
//	}

	IEnumerator UpdateRoutine(){
		while (true) {
			yield return new WaitForEndOfFrame ();
			GL.IssuePluginEvent (GetRenderEventFunc (), 1);
				if(debugImage)
					DebugShowTexture ();
		}
	}
		
	void OnDisable(){
		CloseWebcam ();
	}

	public unsafe void Build(){
		byte[] buffer = new byte[1024];
		int size = 0;
		int result = -1;

		//Pin Memory
		fixed (byte* d = buffer) {

			result = BuildImageTargetDatabase (
				image.GetPixels32 (), 
				image.width, 
				image.height, 
				image.name 
			);
		}

		byte[] data = new byte[size];
		Array.Copy (buffer, data, size);

		Debug.Log ("Received data[" + data.Length + "] with bytes = " + size + " ");
		string directory = "Assets/ImageTargets/";
		string fileName = image.name + ".txt";

		if (!Directory.Exists (directory)) {
			Directory.CreateDirectory (directory);
		}

		string path = directory + fileName;
		File.WriteAllBytes (path, data);
	}

	public unsafe void Initialize(){
		byte[] data = database.bytes;

		Debug.Log ("Init " + database.name);

		//Pin Memory
		fixed (byte* d = data) {
			InitImageTarget (database.name);
		}
	}

	public void TrainTracker(){
		Train ();
	}

	[MonoPInvokeCallback(typeof(imageTargetTrackedCallback))]
	static void OnImageTargetTracked(IntPtr tMat)
	{
		float[] values = new float[16];

		Marshal.Copy (tMat, values, 0, 16);
			
		Matrix4x4 transformationM;

		transformationM.m00 = values [0];
		transformationM.m01 = values [4];
		transformationM.m02 = values [8];
		transformationM.m03 = values [12];

		transformationM.m10 = values [1];
		transformationM.m11 = values [5];
		transformationM.m12 = values [9];
		transformationM.m13 = values [13];

		transformationM.m20 = values [2];
		transformationM.m21 = values [6];
		transformationM.m22 = values [10];
		transformationM.m23 = values [14];

		transformationM.m30 = values [3];
		transformationM.m31 = values [7];
		transformationM.m32 = values [11];
		transformationM.m33 = values [15];

		Debug.Log(transformationM.ToString());
		ARCamera.Instance.SetImageTargetTransform (transformationM);
	}

	public static bool found = false;

	[MonoPInvokeCallback(typeof(imageTargetFoundCallback))]
	static void OnImageTargetFound(){
		Debug.LogWarning ("Found");
		found = true;
	}

	[MonoPInvokeCallback(typeof(imageTargetLostCallback))]
	static void OnImageTargetLost(){
		Debug.LogWarning ("Lost");
		found = false;
	}
}
