using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(imagineARController))]
public class imagineARControllerEditor : Editor {

	imagineARController _target;

	public void OnEnable(){
		_target = (imagineARController)target;
	}

	public override void OnInspectorGUI(){
		DrawDefaultInspector ();

		if (GUILayout.Button ("Build")) {
			_target.Build ();
		}
		if (GUILayout.Button ("Initialize")) {
			_target.Initialize ();
		}
		if (GUILayout.Button ("Train")) {
			_target.TrainTracker ();
		}

	}
}
