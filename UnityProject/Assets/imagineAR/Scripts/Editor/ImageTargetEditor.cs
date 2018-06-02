using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ImageTarget))]
public class ImageTargetEditor : Editor {
	ImageTarget _target;

	public void OnEnable(){
		_target = (ImageTarget)target;
	}

	public override void OnInspectorGUI(){
		DrawDefaultInspector ();

		if (GUILayout.Button ("Create Mesh")) {
			if (_target.GetComponent<MeshFilter> () == null) {
				_target.CreateMesh ();
			}
		}
	}
}
