using System.Collections.Generic;
using UnityEngine;

public class PathfindDemo : MonoBehaviour
{
    private const int WALL_LAYER = 6;

    public ComputeShader pathfindShader;

    public int gridSize = 16;
    public int itersPerFrame = 8;

    private SVOBuilder svoBuilder;
    private SVOBuilder.SVOData svoData;
    private PathfindEngine engine;
    private PathfindVisualizer viz;

    private int startLeafIdx = -1;
    private int goalLeafIdx = -1;
    private string clickMode = "start";
    private bool placeMode = false;
    private bool running = false;
    private bool pathFound = false;
    private List<Vector3> currentWaypoints = null;
    private PathNode[] lastResult = null;

    private Camera cam;
    private Vector3 camTarget;
    private float camDistance;
    private float camTheta;
    private float camPhi;
    private bool isDragging;

    private void Awake()
    {
        cam = Camera.main;
        if (cam == null)
        {
            var camObj = new GameObject("Main Camera");
            cam = camObj.AddComponent<Camera>();
            camObj.AddComponent<AudioListener>();
            camObj.tag = "MainCamera";
        }

        if (pathfindShader == null)
        {
            pathfindShader = Resources.Load<ComputeShader>("PathfindCompute");
        }

        if (pathfindShader == null)
        {
            Debug.LogError("PathfindCompute compute shader not assigned! Please assign it in the Inspector or place it in a Resources folder.");
            enabled = false;
            return;
        }

        camTarget = new Vector3(gridSize / 2f, gridSize / 2f, gridSize / 2f);
        camDistance = gridSize * 2.5f;
        camTheta = Mathf.PI / 4f;
        camPhi = Mathf.PI / 3f;
        UpdateCameraPosition();

        BuildScene();
    }

    private void UpdateCameraPosition()
    {
        float x = camTarget.x + camDistance * Mathf.Sin(camPhi) * Mathf.Cos(camTheta);
        float y = camTarget.y + camDistance * Mathf.Cos(camPhi);
        float z = camTarget.z + camDistance * Mathf.Sin(camPhi) * Mathf.Sin(camTheta);
        cam.transform.position = new Vector3(x, y, z);
        cam.transform.LookAt(camTarget);
    }

    private void BuildScene()
    {
        byte[] grid = BuildGrid();

        svoBuilder = new SVOBuilder(gridSize);
        for (int z = 0; z < gridSize; z++)
            for (int y = 0; y < gridSize; y++)
                for (int x = 0; x < gridSize; x++)
                    svoBuilder.SetVoxel(x, y, z, grid[x + y * gridSize + z * gridSize * gridSize] != 0);

        svoData = svoBuilder.Build();
        Debug.Log($"SVO built: {svoData.leafCount} leaf nodes, {svoData.nodes.Count} total nodes");

        if (svoData.leafCount == 0)
        {
            Debug.LogWarning("No traversable cells found. Place BoxColliders in the scene to create walls.");
            enabled = false;
            return;
        }

        engine = new PathfindEngine();
        engine.Init(pathfindShader, svoData, 1 << WALL_LAYER);

        viz = gameObject.AddComponent<PathfindVisualizer>();
        viz.Init(svoData);

        startLeafIdx = FindEmptyLeaf(1, 1, 1, gridSize - 2, gridSize - 2, 1, 1, 1, 1);
        goalLeafIdx = FindEmptyLeaf(gridSize - 2, gridSize - 2, gridSize - 2, 1, 1, 1, -1, -1, -1);

        if (startLeafIdx < 0)
            startLeafIdx = FindEmptyLeaf(0, 0, 0, gridSize - 1, gridSize - 1, gridSize - 1, 1, 1, 1);
        if (goalLeafIdx < 0)
            goalLeafIdx = FindEmptyLeaf(gridSize - 1, gridSize - 1, gridSize - 1, 0, 0, 0, -1, -1, -1);

        if (startLeafIdx < 0 || goalLeafIdx < 0)
        {
            Debug.LogWarning("Could not find start/goal positions.");
            enabled = false;
            return;
        }

        engine.SetStart(startLeafIdx);
        engine.SetGoal(goalLeafIdx);
        engine.Reset();

        viz.UpdateVisual(null, null, startLeafIdx, goalLeafIdx);
    }

    private byte[] BuildGrid()
    {
        int gs = gridSize;
        byte[] g = new byte[gs * gs * gs];

        var colliders = FindObjectsOfType<BoxCollider>();
        foreach (var col in colliders)
        {
            if (col.transform == transform) continue;
            col.gameObject.layer = WALL_LAYER;
            Bounds b = col.bounds;
            int minX = Mathf.Clamp(Mathf.RoundToInt(b.min.x - 0.5f), 0, gs - 1);
            int minY = Mathf.Clamp(Mathf.RoundToInt(b.min.y - 0.5f), 0, gs - 1);
            int minZ = Mathf.Clamp(Mathf.RoundToInt(b.min.z - 0.5f), 0, gs - 1);
            int maxX = Mathf.Clamp(Mathf.RoundToInt(b.max.x - 0.5f), 0, gs - 1);
            int maxY = Mathf.Clamp(Mathf.RoundToInt(b.max.y - 0.5f), 0, gs - 1);
            int maxZ = Mathf.Clamp(Mathf.RoundToInt(b.max.z - 0.5f), 0, gs - 1);

            for (int z = minZ; z <= maxZ; z++)
                for (int y = minY; y <= maxY; y++)
                    for (int x = minX; x <= maxX; x++)
                        g[x + y * gs + z * gs * gs] = 1;
        }

        return g;
    }

    private int FindEmptyLeaf(int x0, int y0, int z0, int x1, int y1, int z1, int dx, int dy, int dz)
    {
        int gs = gridSize;
        int zs = dz > 0 ? z0 : z1;
        int ze = dz > 0 ? z1 : z0;
        int ys = dy > 0 ? y0 : y1;
        int ye = dy > 0 ? y1 : y0;
        int xs = dx > 0 ? x0 : x1;
        int xe = dx > 0 ? x1 : x0;

        for (int z = zs; z <= ze; z++)
        {
            for (int y = ys; y <= ye; y++)
            {
                for (int x = xs; x <= xe; x++)
                {
                    if (x < 0 || x >= gs || y < 0 || y >= gs || z < 0 || z >= gs) continue;
                    int vi = x + y * gs + z * gs * gs;
                    if (svoData.voxelToLeaf[vi] >= 0)
                        return svoData.voxelToLeaf[vi];
                }
            }
        }
        return -1;
    }

    private bool pathReadbackRequested;

    private void Update()
    {
        HandleCameraInput();
        HandleInput();

        if (engine.UpdateReadback())
        {
            if (engine.GoalReached)
            {
                pathFound = true;
                running = false;
                if (!pathReadbackRequested)
                {
                    engine.RequestPathReadback();
                    pathReadbackRequested = true;
                }
            }
            else if (running && !pathFound)
            {
                engine.DispatchWavefront(itersPerFrame);
            }
        }

        PathNode[] data = engine.GetPathData();
        if (data != null)
        {
            lastResult = data;
            List<Vector3> waypoints = null;
            if (pathFound)
            {
                List<int> leafPath = engine.ReconstructPath(data);
                if (leafPath != null)
                {
                    waypoints = engine.ComputeWaypoints(leafPath);
                    if (waypoints != null)
                        waypoints = engine.SmoothPath(waypoints);
                }
                if (waypoints != null) currentWaypoints = waypoints;
            }
            viz.UpdateVisual(data, currentWaypoints, startLeafIdx, goalLeafIdx);
        }
        else if (lastResult != null)
        {
            viz.UpdateVisual(lastResult, currentWaypoints, startLeafIdx, goalLeafIdx);
        }

        if (running && !pathFound && !engine.IsBusy)
        {
            engine.DispatchWavefront(itersPerFrame);
        }
    }

    private void HandleCameraInput()
    {
        if (placeMode) return;

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
        {
            camDistance *= 1f - scroll * 2f;
            camDistance = Mathf.Clamp(camDistance, 2f, gridSize * 10f);
            UpdateCameraPosition();
        }

        if (Input.GetMouseButtonDown(0) || Input.GetMouseButtonDown(2))
        {
            if (!placeMode && Input.mousePosition.x > 230f)
            {
                isDragging = true;
            }
        }

        if (Input.GetMouseButtonUp(0) || Input.GetMouseButtonUp(2))
        {
            isDragging = false;
        }

        if (isDragging)
        {
            float dx = Input.GetAxis("Mouse X") * 0.01f;
            float dy = Input.GetAxis("Mouse Y") * 0.01f;
            camTheta -= dx * 3f;
            camPhi -= dy * 3f;
            camPhi = Mathf.Clamp(camPhi, 0.1f, Mathf.PI - 0.1f);
            UpdateCameraPosition();
        }
    }

    private void HandleInput()
    {
        if (placeMode && Input.GetMouseButtonDown(0))
        {
            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            int leafIdx = viz.PickVoxel(ray);
            if (leafIdx >= 0)
            {
                if (clickMode == "start")
                {
                    startLeafIdx = leafIdx;
                    engine.SetStart(leafIdx);
                }
                else
                {
                    goalLeafIdx = leafIdx;
                    engine.SetGoal(leafIdx);
                }
                ResetPathfinding();
            }
        }

        if (Input.GetMouseButtonDown(1) && placeMode)
        {
            clickMode = clickMode == "start" ? "goal" : "start";
        }
    }

    private void ResetPathfinding()
    {
        pathFound = false;
        running = false;
        currentWaypoints = null;
        lastResult = null;
        pathReadbackRequested = false;
        engine.Reset();
        viz.UpdateVisual(null, null, startLeafIdx, goalLeafIdx);
    }

    private void OnGUI()
    {
        int x = 10, y = 10, w = 230, h = 340;
        GUI.Box(new Rect(x, y, w, h), "");

        GUILayout.BeginArea(new Rect(x + 10, y + 10, w - 20, h - 20));

        GUILayout.Label("<b>SVO GPU Pathfinding</b>", new GUIStyle(GUI.skin.label) { richText = true, fontSize = 14 });
        GUILayout.Space(5);

        GUILayout.Label($"Grid: {gridSize}x{gridSize}x{gridSize}");
        GUILayout.Label($"Leaf nodes: {svoData.leafCount}");

        if (lastResult != null)
        {
            int fc = 0, vc = 0;
            for (int i = 0; i < svoData.leafCount; i++)
            {
                if (lastResult[i].state == 2) vc++;
                else if (lastResult[i].state == 1) fc++;
            }
            GUILayout.Label($"Frontier: {fc}");
            GUILayout.Label($"Visited: {vc}");
        }

        GUILayout.Label($"Iteration: {engine.Iteration}");

        string status = pathFound ? $"PATH FOUND ({(currentWaypoints != null ? currentWaypoints.Count : 0)} waypoints)" :
                        running ? "Running..." : "Idle";
        GUILayout.Label($"Status: {status}");

        GUILayout.Space(5);

        var sn = startLeafIdx >= 0 ? svoData.leafNodes[startLeafIdx] : null;
        var gn = goalLeafIdx >= 0 ? svoData.leafNodes[goalLeafIdx] : null;
        GUILayout.Label($"Start: {(sn != null ? $"({sn.leafX},{sn.leafY},{sn.leafZ})" : "not set")}");
        GUILayout.Label($"Goal: {(gn != null ? $"({gn.leafX},{gn.leafY},{gn.leafZ})" : "not set")}");

        GUILayout.Space(10);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button(running ? "Pause" : "Run", GUILayout.Width(70)))
        {
            if (running)
            {
                running = false;
            }
            else
            {
                if (startLeafIdx >= 0 && goalLeafIdx >= 0)
                {
                    pathFound = false;
                    currentWaypoints = null;
                    lastResult = null;
                    engine.Reset();
                    running = true;
                }
            }
        }

        if (GUILayout.Button("Step", GUILayout.Width(50)))
        {
            if (startLeafIdx >= 0 && goalLeafIdx >= 0 && !pathFound && !engine.IsBusy)
            {
                if (engine.Iteration == 0) engine.Reset();
                engine.DispatchWavefront(1);
            }
        }

        if (GUILayout.Button("Reset", GUILayout.Width(50)))
        {
            ResetPathfinding();
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(5);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button(placeMode ? "Orbit" : "Place", GUILayout.Width(80)))
        {
            placeMode = !placeMode;
            if (placeMode) clickMode = "start";
            isDragging = false;
        }
        GUILayout.Label(placeMode ? $"Place {clickMode}" : "Orbit");
        GUILayout.EndHorizontal();

        if (placeMode)
        {
            GUILayout.Label("(Right-click: toggle start/goal)");
        }

        GUILayout.Space(5);
        GUILayout.Label($"Z-Slice: {viz.ZSlice}");
        float newSlice = GUILayout.HorizontalSlider(viz.ZSlice, 0, gridSize - 1);
        viz.SetZSlice(Mathf.RoundToInt(newSlice));

        bool newShowSlice = GUILayout.Toggle(viz.ShowSlice, "Show Slice");
        viz.SetShowSlice(newShowSlice);

        GUILayout.EndArea();
    }

    private void OnDestroy()
    {
        engine?.Dispose();
    }
}