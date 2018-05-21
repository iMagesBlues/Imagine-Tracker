using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;


public class imagineARController : MonoBehaviour
{

	[DllImport("imagineARPlugin")]
	private static extern void OpenWebcam(out int w, out int h);

	[DllImport("imagineARPlugin")]
	private static extern void CloseWebcam();

	[DllImport ("imagineARPlugin")]
	private static extern void SetTextureFromUnity(System.IntPtr texture, int w, int h);

	[DllImport("imagineARPlugin")]
	private static extern void DebugShowTexture();

	[DllImport("imagineARPlugin")]
	private static extern IntPtr GetRenderEventFunc();

	void Start()
	{
		InitWebcam ();
	}

	WebCamTexture webcamTexture;
	public Texture2D panda;
	private void InitWebcam(){
//		webcamTexture = new WebCamTexture();
//		webcamTexture.Play();


		//get webcamtexture dimensions;
		int width = 0, height = 0;
		OpenWebcam(out width, out height);

		Debug.Log("Initialize Webcam with dimensions[" + width + "," + height + "]" );

		Texture2D tex = new Texture2D (width, height, TextureFormat.RGBA32, false);
		tex.filterMode = FilterMode.Point;
		tex.Apply ();

		GetComponent<Renderer>().material.mainTexture = tex;


		// Pass texture pointer to the plugin
		SetTextureFromUnity (tex.GetNativeTexturePtr(), tex.width, tex.height);

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
}
