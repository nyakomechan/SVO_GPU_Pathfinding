using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public struct PathNode
{
    public float cost;
    public int state;
    public int parent;
    public int padding;
}

public struct NeighborData
{
    public int n0, n1, n2, n3, n4, n5;
}

public class PathfindEngine
{
    private ComputeShader computeShader;
    private int kernelWavefront;
    private int kernelGoalDetect;
    private int kernelReset;

    private ComputeBuffer neighborBuf;
    private ComputeBuffer pathDataA;
    private ComputeBuffer pathDataB;
    private ComputeBuffer goalResultBuf;

    private int nodeCount;
    private int startIdx = -1;
    private int goalIdx = -1;
    private bool currentReadA = true;
    private int iteration;
    private bool goalReached;

    private SVOBuilder.SVOData svoData;
    private int wallLayerMask;

    private enum EngineState { Idle, Dispatched, GoalReadbackPending, PathReadbackPending }
    private EngineState state = EngineState.Idle;

    private AsyncGPUReadbackRequest goalReadbackRequest;
    private AsyncGPUReadbackRequest pathReadbackRequest;

    private static readonly int NODE_COUNT = Shader.PropertyToID("nodeCount");
    private static readonly int GOAL_INDEX = Shader.PropertyToID("goalIndex");
    private static readonly int START_INDEX = Shader.PropertyToID("startIndex");
    private static readonly int NEIGHBORS = Shader.PropertyToID("neighbors");
    private static readonly int PATH_DATA_IN = Shader.PropertyToID("pathDataIn");
    private static readonly int PATH_DATA_OUT = Shader.PropertyToID("pathDataOut");
    private static readonly int GOAL_RESULT = Shader.PropertyToID("goalResult");

    public int Iteration => iteration;
    public bool GoalReached => goalReached;
    public int LeafCount => nodeCount;
    public SVOBuilder.SVOData SVOData => svoData;
    public bool IsBusy => state != EngineState.Idle;

    public void Init(ComputeShader shader, SVOBuilder.SVOData data, int wallLayerMask = -1)
    {
        this.wallLayerMask = wallLayerMask;
        computeShader = shader;
        svoData = data;
        nodeCount = data.leafCount;

        kernelWavefront = computeShader.FindKernel("WavefrontExpand");
        kernelGoalDetect = computeShader.FindKernel("GoalDetect");
        kernelReset = computeShader.FindKernel("Reset");

        int neighborStride = System.Runtime.InteropServices.Marshal.SizeOf(typeof(NeighborData));
        neighborBuf = new ComputeBuffer(nodeCount, neighborStride);

        var neighborArr = new NeighborData[nodeCount];
        for (int i = 0; i < nodeCount; i++)
        {
            neighborArr[i] = new NeighborData
            {
                n0 = data.neighbors[i * 6 + 0],
                n1 = data.neighbors[i * 6 + 1],
                n2 = data.neighbors[i * 6 + 2],
                n3 = data.neighbors[i * 6 + 3],
                n4 = data.neighbors[i * 6 + 4],
                n5 = data.neighbors[i * 6 + 5],
            };
        }
        neighborBuf.SetData(neighborArr);

        int pathNodeStride = System.Runtime.InteropServices.Marshal.SizeOf(typeof(PathNode));
        pathDataA = new ComputeBuffer(nodeCount, pathNodeStride);
        pathDataB = new ComputeBuffer(nodeCount, pathNodeStride);

        goalResultBuf = new ComputeBuffer(1, 4);

        computeShader.SetBuffer(kernelWavefront, NEIGHBORS, neighborBuf);
        computeShader.SetInt(NODE_COUNT, nodeCount);

        state = EngineState.Idle;
    }

    public void SetStart(int idx)
    {
        startIdx = idx;
    }

    public void SetGoal(int idx)
    {
        goalIdx = idx;
    }

    public void Reset()
    {
        iteration = 0;
        goalReached = false;
        currentReadA = true;
        state = EngineState.Idle;

        computeShader.SetInt(START_INDEX, startIdx);
        computeShader.SetInt(NODE_COUNT, nodeCount);
        computeShader.SetBuffer(kernelReset, PATH_DATA_OUT, pathDataA);
        int groups = Mathf.CeilToInt((float)nodeCount / 64f);
        computeShader.Dispatch(kernelReset, groups, 1, 1);

        computeShader.SetBuffer(kernelReset, PATH_DATA_OUT, pathDataB);
        computeShader.Dispatch(kernelReset, groups, 1, 1);
    }

    public void DispatchWavefront(int count)
    {
        if (startIdx < 0 || goalIdx < 0) return;
        if (goalReached) return;
        if (state != EngineState.Idle) return;

        computeShader.SetInt(NODE_COUNT, nodeCount);
        computeShader.SetInt(GOAL_INDEX, goalIdx);

        int groups = Mathf.CeilToInt((float)nodeCount / 64f);

        for (int i = 0; i < count; i++)
        {
            ComputeBuffer readBuf = currentReadA ? pathDataA : pathDataB;
            ComputeBuffer writeBuf = currentReadA ? pathDataB : pathDataA;

            computeShader.SetBuffer(kernelWavefront, PATH_DATA_IN, readBuf);
            computeShader.SetBuffer(kernelWavefront, PATH_DATA_OUT, writeBuf);
            computeShader.Dispatch(kernelWavefront, groups, 1, 1);

            currentReadA = !currentReadA;
            iteration++;
        }

        ComputeBuffer currentBuf = currentReadA ? pathDataA : pathDataB;
        computeShader.SetBuffer(kernelGoalDetect, PATH_DATA_IN, currentBuf);
        computeShader.SetInt(GOAL_INDEX, goalIdx);
        computeShader.SetInt(NODE_COUNT, nodeCount);
        computeShader.SetBuffer(kernelGoalDetect, GOAL_RESULT, goalResultBuf);
        computeShader.Dispatch(kernelGoalDetect, 1, 1, 1);

        goalReadbackRequest = AsyncGPUReadback.Request(goalResultBuf);
        state = EngineState.GoalReadbackPending;
    }

    public bool UpdateReadback()
    {
        if (state == EngineState.GoalReadbackPending)
        {
            if (!goalReadbackRequest.done) return false;
            if (goalReadbackRequest.hasError)
            {
                state = EngineState.Idle;
                return false;
            }

            int result = goalReadbackRequest.GetData<int>()[0];
            goalReached = result > 0;
            state = EngineState.Idle;
            return true;
        }

        if (state == EngineState.PathReadbackPending)
        {
            if (!pathReadbackRequest.done) return false;
            if (pathReadbackRequest.hasError)
            {
                state = EngineState.Idle;
                return true;
            }
            state = EngineState.Idle;
            return true;
        }

        return false;
    }

    public void RequestPathReadback()
    {
        ComputeBuffer currentBuf = currentReadA ? pathDataA : pathDataB;
        pathReadbackRequest = AsyncGPUReadback.Request(currentBuf);
        state = EngineState.PathReadbackPending;
    }

    public PathNode[] GetPathData()
    {
        if (state != EngineState.Idle) return null;
        if (!pathReadbackRequest.done || pathReadbackRequest.hasError) return null;
        return pathReadbackRequest.GetData<PathNode>().ToArray();
    }

    public List<int> ReconstructPath(PathNode[] data)
    {
        if (!goalReached || data == null) return null;

        var path = new List<int>();
        int current = goalIdx;
        var visited = new HashSet<int>();

        while (current >= 0 && current < nodeCount)
        {
            if (visited.Contains(current)) break;
            visited.Add(current);
            path.Add(current);
            if (current == startIdx) break;
            int parentIdx = data[current].parent;
            if (parentIdx < 0 || parentIdx == current) break;
            current = parentIdx;
        }

        path.Reverse();
        return path;
    }

    public List<Vector3> ComputeWaypoints(List<int> path)
    {
        if (path == null || path.Count == 0) return null;

        var waypoints = new List<Vector3>();
        var leafNodes = svoData.leafNodes;

        waypoints.Add(new Vector3(
            leafNodes[path[0]].leafX + 0.5f,
            leafNodes[path[0]].leafY + 0.5f,
            leafNodes[path[0]].leafZ + 0.5f));

        for (int i = 0; i < path.Count - 1; i++)
        {
            var a = leafNodes[path[i]];
            var b = leafNodes[path[i + 1]];
            waypoints.Add(new Vector3(
                (a.leafX + b.leafX) / 2f + 0.5f,
                (a.leafY + b.leafY) / 2f + 0.5f,
                (a.leafZ + b.leafZ) / 2f + 0.5f));
        }

        waypoints.Add(new Vector3(
            leafNodes[path[path.Count - 1]].leafX + 0.5f,
            leafNodes[path[path.Count - 1]].leafY + 0.5f,
            leafNodes[path[path.Count - 1]].leafZ + 0.5f));

        return waypoints;
    }

    public List<Vector3> SmoothPath(List<Vector3> waypoints)
    {
        if (waypoints == null || waypoints.Count <= 2) return waypoints;

        var result = new List<Vector3> { waypoints[0] };
        int current = 0;

        while (current < waypoints.Count - 1)
        {
            int farthest = current + 1;
            for (int candidate = waypoints.Count - 1; candidate > current + 1; candidate--)
            {
                if (IsLineClear(waypoints[current], waypoints[candidate]))
                {
                    farthest = candidate;
                    break;
                }
            }
            result.Add(waypoints[farthest]);
            current = farthest;
        }

        return result;
    }

    private bool IsLineClear(Vector3 from, Vector3 to)
    {
        const float radius = 0.5f;

        if (Physics.CheckSphere(from, radius, wallLayerMask)) return false;
        if (Physics.CheckSphere(to, radius, wallLayerMask)) return false;

        Vector3 dir = to - from;
        float dist = dir.magnitude;
        if (dist < 0.001f) return true;

        return !Physics.SphereCast(new Ray(from, dir.normalized), radius, dist, wallLayerMask);
    }

    public void Dispose()
    {
        neighborBuf?.Dispose();
        pathDataA?.Dispose();
        pathDataB?.Dispose();
        goalResultBuf?.Dispose();
    }
}