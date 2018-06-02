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

	public ImageTarget trackedImageTarget;


	public void Start(){
		invertYM = Matrix4x4.TRS (Vector3.zero, Quaternion.identity, new Vector3 (1, -1, 1));
		Debug.Log ("invertYM " + invertYM.ToString ());
		invertZM = Matrix4x4.TRS (Vector3.zero, Quaternion.identity, new Vector3 (1, 1, -1));
		Debug.Log ("invertZM " + invertZM.ToString ());
	}

	public Vector3 ExtractTranslationFromMatrix (Matrix4x4 matrix)
	{
		Vector3 translate;
		translate.x = matrix.m03;
		translate.y = matrix.m13;
		translate.z = matrix.m23;
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

		return Quaternion.LookRotation (forward, upwards);
	}
		
	public Vector3 ExtractScaleFromMatrix (Matrix4x4 matrix)
	{
		Vector3 scale;
		scale.x = new Vector4 (matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
		scale.y = new Vector4 (matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
		scale.z = new Vector4 (matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
		return scale;
	}
		
		
	public void SetImageTargetTransform (Matrix4x4 transforMationMatrix)
	{
		targetTransform = transforMationMatrix;
	}

	void Update(){
		if (imagineARController.found) {
			trackedImageTarget.gameObject.SetActive (true);

			//Matrix4x4 matrix = this.transform.localToWorldMatrix * invertYM * targetTransform * invertZM;
			Matrix4x4 matrix = this.transform.localToWorldMatrix * targetTransform;


			trackedImageTarget.transform.localPosition = ExtractTranslationFromMatrix (matrix);
			trackedImageTarget.transform.localRotation = ExtractRotationFromMatrix (matrix);
			trackedImageTarget.transform.localScale = ExtractScaleFromMatrix (matrix);

			Vector3 eul = trackedImageTarget.transform.eulerAngles;
			eul.x -= 90;
			trackedImageTarget.transform.eulerAngles = eul;
				
		} 
		else {
			trackedImageTarget.gameObject.SetActive (false);

		}
	}
		
}
