using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilterPos
{
    private Vector3 A, H, Q, R, P, x;

    public KalmanFilterPos(Vector3 A, Vector3 H, Vector3 Q, Vector3 R, Vector3 initial_P, Vector3 initial_x)
    {
        this.A = A;
        this.H = H;
        this.Q = Q;
        this.R = R;
        this.P = initial_P;
        this.x = initial_x;
    }

    public Vector3 Output(Vector3 input)
    {
        // time update - prediction
        x = Mul(A, x);//A * x;
        P = Mul( Mul(A, P), A) + Q;// A * P * A + Q;

        // measurement update - correction
        Vector3 K = Div(Mul(P, H),  Mul(Mul(H, P), H) + R);//P * H / (H * P * H + R);
        x = Mul(x + K, (input - Mul(H,x)));
        //P = (1 - K * H) * P;
        P = Mul(One - Mul(K, H), P);
        return x;
    }

    Vector3 Mul(Vector3 a, Vector3 b)
    {
        return (Vector3.Scale(a, b));
    }
    Vector3 Div(Vector3 a, Vector3 b)
    {
        return new Vector3(a.x / b.x, a.y / b.y, a.x / b.x);
    }
    Vector3 One{
        get{
            return Vector3.one;
        }
    }
}

public class ImageTarget : MonoBehaviour {

	public Texture2D texture;
	public TextAsset database;
	public float width, height;
    KalmanFilterPos kalmanPos;
    KalmanFilterPos kalmanRot;


    // Use this for initialization
    void Start () {
        //CreateMesh ();
        kalmanPos = new KalmanFilterPos(One, One, .125f*One, One, 0.1f*One, transform.localPosition);
        kalmanPos = new KalmanFilterPos(One, One, .125f * One, One, 0.1f * One, transform.localEulerAngles);

    }

    // Update is called once per frame
    void Update () {
        transform.localPosition = kalmanPos.Output(transform.localPosition);
        transform.localEulerAngles = kalmanPos.Output(transform.localEulerAngles);

    }

    Vector3 One
    {
        get
        {
            return Vector3.one;
        }
    }

    public void CreateMesh(){

		// You can change that line to provide another MeshFilter
		MeshFilter filter = gameObject.AddComponent< MeshFilter >();
		Mesh mesh = filter.mesh;
		mesh.Clear();

		int resX = 2; // 2 minimum
		int resZ = 2;

		float _height = (float)height / 100;
		float _width = (float)width / 100;


		#region Vertices		
		Vector3[] vertices = new Vector3[ resX * resZ ];
		for(int z = 0; z < resZ; z++)
		{
			// [ -height / 2, height / 2 ]
			float zPos = ((float)z / (resZ - 1) - .5f) * _height;
			for(int x = 0; x < resX; x++)
			{
				// [ -width / 2, width / 2 ]
				float xPos = ((float)x / (resX - 1) - .5f) * _width;
				vertices[ x + z * resX ] = new Vector3( xPos, 0f, zPos );
			}
		}
		#endregion

		#region Normales
		Vector3[] normales = new Vector3[ vertices.Length ];
		for( int n = 0; n < normales.Length; n++ )
			normales[n] = Vector3.up;
		#endregion

		#region UVs		
		Vector2[] uvs = new Vector2[ vertices.Length ];
		for(int v = 0; v < resZ; v++)
		{
			for(int u = 0; u < resX; u++)
			{
				uvs[ u + v * resX ] = new Vector2( (float)u / (resX - 1), (float)v / (resZ - 1) );
			}
		}
		#endregion

		#region Triangles
		int nbFaces = (resX - 1) * (resZ - 1);
		int[] triangles = new int[ nbFaces * 6 ];
		int t = 0;
		for(int face = 0; face < nbFaces; face++ )
		{
			// Retrieve lower left corner from face ind
			int i = face % (resX - 1) + (face / (resZ - 1) * resX);

			triangles[t++] = i + resX;
			triangles[t++] = i + 1;
			triangles[t++] = i;

			triangles[t++] = i + resX;	
			triangles[t++] = i + resX + 1;
			triangles[t++] = i + 1; 
		}
		#endregion

		mesh.vertices = vertices;
		mesh.normals = normales;
		mesh.uv = uvs;
		mesh.triangles = triangles;

		mesh.RecalculateBounds();
		;


		MeshRenderer renderer = GetComponent<MeshRenderer> ();
		if (renderer == null) {
			renderer = gameObject.AddComponent<MeshRenderer> ();
			renderer.material.mainTexture = texture;
			renderer.material.shader = Shader.Find ("Unlit/Texture");
		}
	}
}
