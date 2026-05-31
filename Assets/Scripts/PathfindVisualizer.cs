using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class PathfindVisualizer : MonoBehaviour
{
    private static readonly Color WALL_COLOR = new Color(0.29f, 0.29f, 0.42f, 0.5f);
    private static readonly Color FRONTIER_COLOR = new Color(0.25f, 0.57f, 0.42f, 0.7f);
    private static readonly Color VISITED_COLOR = new Color(0.18f, 0.42f, 0.31f, 0.5f);
    private static readonly Color PATH_COLOR = new Color(1f, 0.82f, 0.4f, 1f);
    private static readonly Color START_MARKER_COLOR = new Color(0.02f, 0.84f, 0.63f, 1f);
    private static readonly Color GOAL_MARKER_COLOR = new Color(0.94f, 0.28f, 0.44f, 1f);
    private static readonly Color SLICE_PLANE_COLOR = new Color(1f, 0.82f, 0.4f, 0.08f);
    private static readonly Color BORDER_COLOR = new Color(0.2f, 0.2f, 0.33f, 0.3f);
    private static readonly Color SLICE_EDGE_COLOR = new Color(1f, 0.82f, 0.4f, 0.4f);

    private SVOBuilder.SVOData svoData;
    private int gs;

    private Mesh cubeMesh;
    private Mesh sphereMesh;

    private Material wallMat;
    private Material frontierMat;
    private Material visitedMat;
    private Material startMat;
    private Material goalMat;

    private Matrix4x4[] wallMatrices;
    private Matrix4x4[] frontierMatrices;
    private Matrix4x4[] visitedMatrices;

    private int wallCount;
    private int frontierCount;
    private int visitedCount;

    private GameObject startMarkerObj;
    private GameObject goalMarkerObj;
    private GameObject sliceObj;
    private LineRenderer sliceEdge;
    private LineRenderer pathLineObj;

    private int zSlice;
    private bool showSlice = true;

    private const int MAX_INSTANCES_PER_DRAW = 1023;

    public int ZSlice => zSlice;
    public bool ShowSlice => showSlice;
    public int FrontierCount => frontierCount;
    public int VisitedCount => visitedCount;

    public void Init(SVOBuilder.SVOData data)
    {
        svoData = data;
        gs = data.gridSize;
        zSlice = gs / 2;

        CreateMeshes();
        CreateMaterials();
        CreateWallInstances();
        CreateDynamicBuffers();
        CreateMarkers();
        CreateSlicePlane();
        CreateBorderFrame();
    }

    private void CreateMeshes()
    {
        var cubeGO = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeMesh = cubeGO.GetComponent<MeshFilter>().sharedMesh;
        Object.Destroy(cubeGO);

        var sphereGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphereMesh = sphereGO.GetComponent<MeshFilter>().sharedMesh;
        Object.Destroy(sphereGO);
    }

    private Material CreateTransparentMaterial(Color color, int renderQueue = 3000)
    {
        var mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        mat.enableInstancing = true;
        mat.SetFloat("_Glossiness", 0.1f);
        mat.SetFloat("_Metallic", 0f);
        if (color.a < 1f)
        {
            mat.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
            mat.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
            mat.SetInt("_ZWrite", 0);
            mat.DisableKeyword("_ALPHATEST_ON");
            mat.EnableKeyword("_ALPHABLEND_ON");
            mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            mat.renderQueue = renderQueue;
        }
        return mat;
    }

    private void CreateMaterials()
    {
        wallMat = CreateTransparentMaterial(WALL_COLOR, 2900);
        frontierMat = CreateTransparentMaterial(FRONTIER_COLOR, 3000);
        visitedMat = CreateTransparentMaterial(VISITED_COLOR, 2950);
        startMat = CreateTransparentMaterial(START_MARKER_COLOR, 3100);
        goalMat = CreateTransparentMaterial(GOAL_MARKER_COLOR, 3100);
    }

    private void CreateWallInstances()
    {
        var positions = new List<Matrix4x4>();
        for (int z = 0; z < gs; z++)
        {
            for (int y = 0; y < gs; y++)
            {
                for (int x = 0; x < gs; x++)
                {
                    int vi = x + y * gs + z * gs * gs;
                    if (svoData.voxelToLeaf[vi] == -1)
                    {
                        positions.Add(Matrix4x4.TRS(
                            new Vector3(x + 0.5f, y + 0.5f, z + 0.5f),
                            Quaternion.identity,
                            Vector3.one
                        ));
                    }
                }
            }
        }
        wallCount = positions.Count;
        wallMatrices = positions.ToArray();
    }

    private void CreateDynamicBuffers()
    {
        int maxNodes = svoData.leafCount;
        frontierMatrices = new Matrix4x4[maxNodes];
        visitedMatrices = new Matrix4x4[maxNodes];
        frontierCount = 0;
        visitedCount = 0;
    }

    private void CreateMarkers()
    {
        startMarkerObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Object.Destroy(startMarkerObj.GetComponent<Collider>());
        startMarkerObj.transform.localScale = Vector3.one * 0.5f;
        startMarkerObj.GetComponent<Renderer>().material = startMat;
        startMarkerObj.SetActive(false);

        goalMarkerObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Object.Destroy(goalMarkerObj.GetComponent<Collider>());
        goalMarkerObj.transform.localScale = Vector3.one * 0.5f;
        goalMarkerObj.GetComponent<Renderer>().material = goalMat;
        goalMarkerObj.SetActive(false);
    }

    private void CreateSlicePlane()
    {
        sliceObj = GameObject.CreatePrimitive(PrimitiveType.Quad);
        Object.Destroy(sliceObj.GetComponent<Collider>());
        sliceObj.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        sliceObj.transform.localScale = new Vector3(gs, gs, 1f);
        sliceObj.GetComponent<Renderer>().material = CreateTransparentMaterial(SLICE_PLANE_COLOR, 2800);
        UpdateSlicePosition();
        sliceObj.SetActive(showSlice);

        var edgeGO = new GameObject("SliceEdge");
        sliceEdge = edgeGO.AddComponent<LineRenderer>();
        sliceEdge.positionCount = 5;
        UpdateSliceEdge();
        sliceEdge.startWidth = 0.05f;
        sliceEdge.endWidth = 0.05f;
        sliceEdge.material = new Material(Shader.Find("Unlit/Color"));
        sliceEdge.material.color = SLICE_EDGE_COLOR;
        sliceEdge.gameObject.SetActive(showSlice);
    }

    private void UpdateSlicePosition()
    {
        if (sliceObj != null)
            sliceObj.transform.position = new Vector3(gs / 2f, zSlice + 0.5f, gs / 2f);
        UpdateSliceEdge();
    }

    private void UpdateSliceEdge()
    {
        if (sliceEdge == null) return;
        float y = zSlice + 0.51f;
        sliceEdge.SetPositions(new Vector3[]
        {
            new Vector3(0, y, 0),
            new Vector3(gs, y, 0),
            new Vector3(gs, y, gs),
            new Vector3(0, y, gs),
            new Vector3(0, y, 0),
        });
    }

    private void CreateBorderFrame()
    {
        var border = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Object.Destroy(border.GetComponent<Collider>());
        border.transform.position = new Vector3(gs / 2f, gs / 2f, gs / 2f);
        border.transform.localScale = new Vector3(gs, gs, gs);
        border.GetComponent<Renderer>().material = CreateTransparentMaterial(BORDER_COLOR, 2500);

        var wireGO = new GameObject("Wireframe");
        var lr = wireGO.AddComponent<LineRenderer>();
        float h = gs;
        Vector3 c = new Vector3(0, 0, 0);
        lr.positionCount = 24;
        lr.startWidth = 0.03f;
        lr.endWidth = 0.03f;
        lr.material = new Material(Shader.Find("Unlit/Color"));
        lr.material.color = new Color(0.2f, 0.2f, 0.33f, 0.6f);

        Vector3[] corners = new Vector3[]
        {
            new Vector3(0,0,0), new Vector3(gs,0,0), new Vector3(gs,0,gs), new Vector3(0,0,gs),
            new Vector3(0,gs,0), new Vector3(gs,gs,0), new Vector3(gs,gs,gs), new Vector3(0,gs,gs),
        };

        var lines = new Vector3[]
        {
            corners[0], corners[1], corners[1], corners[2], corners[2], corners[3], corners[3], corners[0],
            corners[4], corners[5], corners[5], corners[6], corners[6], corners[7], corners[7], corners[4],
            corners[0], corners[4], corners[1], corners[5], corners[2], corners[6], corners[3], corners[7],
        };
        lr.SetPositions(lines);
    }

    private void LateUpdate()
    {
        RenderInstanced(wallMatrices, wallCount, wallMat, Vector3.one);
        RenderInstanced(frontierMatrices, frontierCount, frontierMat, Vector3.one * 0.9f);
        RenderInstanced(visitedMatrices, visitedCount, visitedMat, Vector3.one * 0.9f);
    }

    private void RenderInstanced(Matrix4x4[] matrices, int count, Material mat, Vector3 scale)
    {
        if (count == 0) return;
        int offset = 0;
        while (offset < count)
        {
            int batch = Mathf.Min(MAX_INSTANCES_PER_DRAW, count - offset);
            var batchMatrices = new Matrix4x4[batch];
            for (int i = 0; i < batch; i++)
            {
                var origPos = matrices[offset + i].GetColumn(3);
                batchMatrices[i] = Matrix4x4.TRS(origPos, Quaternion.identity, scale);
            }
            Graphics.DrawMeshInstanced(cubeMesh, 0, mat, batchMatrices, batch);
            offset += batch;
        }
    }

    public void UpdateVisual(PathNode[] pathData, List<Vector3> waypoints, int startIdx, int goalIdx)
    {
        frontierCount = 0;
        visitedCount = 0;

        if (pathData != null)
        {
            for (int i = 0; i < svoData.leafCount; i++)
            {
                var ln = svoData.leafNodes[i];
                var state = pathData[i].state;
                Vector3 pos = new Vector3(ln.leafX + 0.5f, ln.leafY + 0.5f, ln.leafZ + 0.5f);

                if (state == 2)
                {
                    if (visitedCount < visitedMatrices.Length)
                        visitedMatrices[visitedCount++] = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one);
                }
                else if (state == 1)
                {
                    if (frontierCount < frontierMatrices.Length)
                        frontierMatrices[frontierCount++] = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one);
                }
            }
        }

        UpdatePathLine(waypoints);
        UpdateMarkers(startIdx, goalIdx);
    }

    private void UpdateMarkers(int startIdx, int goalIdx)
    {
        if (startIdx >= 0 && startIdx < svoData.leafCount)
        {
            var sn = svoData.leafNodes[startIdx];
            startMarkerObj.transform.position = new Vector3(sn.leafX + 0.5f, sn.leafY + 0.5f, sn.leafZ + 0.5f);
            startMarkerObj.SetActive(true);
        }
        else
        {
            startMarkerObj.SetActive(false);
        }

        if (goalIdx >= 0 && goalIdx < svoData.leafCount)
        {
            var gn = svoData.leafNodes[goalIdx];
            goalMarkerObj.transform.position = new Vector3(gn.leafX + 0.5f, gn.leafY + 0.5f, gn.leafZ + 0.5f);
            goalMarkerObj.SetActive(true);
        }
        else
        {
            goalMarkerObj.SetActive(false);
        }
    }

    private void UpdatePathLine(List<Vector3> waypoints)
    {
        if (pathLineObj != null)
        {
            Object.Destroy(pathLineObj.gameObject);
            pathLineObj = null;
        }

        if (waypoints == null || waypoints.Count < 2) return;

        var go = new GameObject("PathLine");
        pathLineObj = go.AddComponent<LineRenderer>();
        pathLineObj.positionCount = waypoints.Count;
        pathLineObj.startWidth = 0.15f;
        pathLineObj.endWidth = 0.15f;
        pathLineObj.material = new Material(Shader.Find("Unlit/Color"));
        pathLineObj.material.color = PATH_COLOR;

        for (int i = 0; i < waypoints.Count; i++)
        {
            pathLineObj.SetPosition(i, waypoints[i]);
        }
    }

    public void SetZSlice(int z)
    {
        zSlice = z;
        UpdateSlicePosition();
    }

    public void SetShowSlice(bool show)
    {
        showSlice = show;
        if (sliceObj != null) sliceObj.SetActive(show);
        if (sliceEdge != null) sliceEdge.gameObject.SetActive(show);
    }

    public int PickVoxel(Ray ray)
    {
        var plane = new Plane(Vector3.up, new Vector3(0, zSlice + 0.5f, 0));
        float enter;
        if (!plane.Raycast(ray, out enter)) return -1;

        Vector3 hit = ray.GetPoint(enter);
        int vx = Mathf.FloorToInt(hit.x);
        int vy = Mathf.FloorToInt(hit.y);
        int vz = Mathf.FloorToInt(hit.z);

        if (vx < 0 || vx >= gs || vy < 0 || vy >= gs || vz < 0 || vz >= gs) return -1;
        if (vy != zSlice) return -1;

        int vi = vx + vy * gs + vz * gs * gs;
        return svoData.voxelToLeaf[vi];
    }

    private void OnDestroy()
    {
        if (wallMat != null) Object.Destroy(wallMat);
        if (frontierMat != null) Object.Destroy(frontierMat);
        if (visitedMat != null) Object.Destroy(visitedMat);
        if (startMat != null) Object.Destroy(startMat);
        if (goalMat != null) Object.Destroy(goalMat);
    }
}