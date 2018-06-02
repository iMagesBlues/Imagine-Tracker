using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;


public class imagineARController : MonoBehaviour
{
	public Texture2D image;
	public TextAsset database;


	[DllImport("imagineARPlugin")]
	private static extern void OpenWebcam(out int w, out int h);

	[DllImport("imagineARPlugin")]
	private static extern void CloseWebcam();

	[DllImport ("imagineARPlugin")]
	private static extern void SetWebcamTexture(System.IntPtr texture, int w, int h);

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

		GetComponent<Renderer>().material.mainTexture = tex;


		// Pass texture pointer to the plugin
		SetWebcamTexture (tex.GetNativeTexturePtr(), tex.width, tex.height);

		//Initialize Imagetarget
		Initialize();
		Train ();

		StartCoroutine("CallPluginAtEndOfFrames");

	}
		
		
	private IEnumerator CallPluginAtEndOfFrames()
	{
		while (true) {
			yield return new WaitForEndOfFrame();

			GL.IssuePluginEvent (GetRenderEventFunc (), 1);
			//DebugShowTexture ();
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
}
