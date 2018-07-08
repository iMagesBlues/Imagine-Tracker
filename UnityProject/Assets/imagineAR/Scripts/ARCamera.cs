using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARCamera : MonoBehaviour {

	private static ARCamera instance;

	public static ARCamera Instance{
		get{
			if (instance == null)
				instance = GameObject.FindObjectOfType<ARCamera> ();

			return instance;
		}
	}

	public Renderer videoBackground;

	Matrix4x4 invertYM, invertZM;
	Matrix4x4 targetTransform;

	public Vector3 ypos, yeul, ysca = Vector3.one;
	public Vector3 zpos, zeul, zsca = Vector3.one;
	public float mulx = -1, muly = -1, mulz = 1;


	public ImageTarget trackedImageTarget;

	//TODO: try rotating x by 90 and offset x y z by sin cos angles
	public void Start(){
		invertYM = Matrix4x4.TRS (Vector3.zero, Quaternion.Euler(0, 0, 0), new Vector3 (1, 1, 1));
		Debug.Log ("invertYM " + invertYM.ToString ());
		invertZM = Matrix4x4.TRS (Vector3.zero, Quaternion.Euler(0, 0, 0), new Vector3 (1, 1, 1));
		Debug.Log ("invertZM " + invertZM.ToString ());
	}

	public Vector3 ExtractTranslationFromMatrix (Matrix4x4 matrix)
	{
		Vector3 translate;
		translate.x = mulx * matrix.m03;
		translate.y = muly * matrix.m13;
		translate.z = mulz * matrix.m23;
		return translate;
	}
		
	public Quaternion ExtractRotationFromMatrix (Matrix4x4 matrix)
	{
		Vector3 forward;
		forward.x = matrix.m02;
		forward.y = matrix.m12;
		forward.z = matrix.m22;

		Vector3 upwards;
		upwards.x = matrix.m01;
		upwards.y = matrix.m11;
		upwards.z = matrix.m21;

		Quaternion rot = Quaternion.LookRotation (forward, upwards);

		//flip right handed to left handed to right handed (back cam - unmirrored x)
		//make sure to use mulx = 1 
		rot.y *= -1;
		rot.w *= -1;

		////flip right handed to left handed to right handed (front cam - mirrored x)
		//make sure to use mulx = -1 
		//rot.x *= -1;
		//rot.y *= -1;

		rot = Quaternion.Inverse (rot);

		return rot * Quaternion.Euler(-90, 0, 0);
	}
		
	public Vector3 ExtractScaleFromMatrix (Matrix4x4 matrix)
	{
		Vector3 scale;
		scale.x = new Vector4 (matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;// * zsca.x;
		scale.y = new Vector4 (matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;// * zsca.y;
		scale.z = new Vector4 (matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;// * zsca.z;
		return scale;
	}
		
		
	public void SetImageTargetTransform (Matrix4x4 transforMationMatrix)
	{
		targetTransform = transforMationMatrix;
	}

	void Update(){
		if (imagineARController.found) {
			trackedImageTarget.gameObject.SetActive (true);

			Matrix4x4 yM = Matrix4x4.TRS (ypos, Quaternion.Euler (yeul), ysca);
			Matrix4x4 zM = Matrix4x4.TRS (zpos, Quaternion.Euler (zeul), zsca);

			//Debug.LogWarning ("yM = \n" + yM);
			//Debug.LogWarning ("zM = \n" + zM);


			Matrix4x4 matrix = this.transform.localToWorldMatrix * zM * targetTransform * yM;

			trackedImageTarget.transform.localPosition = ExtractTranslationFromMatrix (matrix);
			trackedImageTarget.transform.localRotation = ExtractRotationFromMatrix (matrix);
			trackedImageTarget.transform.localScale = ExtractScaleFromMatrix (matrix);


				
		} 
		else {
			trackedImageTarget.gameObject.SetActive (false);

		}
	}
		
}
