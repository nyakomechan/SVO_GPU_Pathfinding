using System;
using System.Collections.Generic;

public class SVOBuilder
{
    public int gridSize;
    public byte[] grid;

    public SVOBuilder(int gridSize)
    {
        this.gridSize = gridSize;
        this.grid = new byte[gridSize * gridSize * gridSize];
    }

    private int Idx3(int x, int y, int z)
    {
        return x + y * gridSize + z * gridSize * gridSize;
    }

    public void SetVoxel(int x, int y, int z, bool occupied)
    {
        if (x < 0 || x >= gridSize || y < 0 || y >= gridSize || z < 0 || z >= gridSize) return;
        grid[Idx3(x, y, z)] = occupied ? (byte)1 : (byte)0;
    }

    public bool GetVoxel(int x, int y, int z)
    {
        if (x < 0 || x >= gridSize || y < 0 || y >= gridSize || z < 0 || z >= gridSize) return true;
        return grid[Idx3(x, y, z)] != 0;
    }

    public class SVONode
    {
        public int idx;
        public int childBase;
        public int parent;
        public int level;
        public bool occupied;
        public bool isLeaf;
        public int ox, oy, oz, size;
        public int leafIdx;
        public int leafX, leafY, leafZ;
    }

    public class SVOData
    {
        public List<SVONode> nodes;
        public List<SVONode> leafNodes;
        public int[] neighbors;
        public int[] voxelToLeaf;
        public float[] leafCosts;
        public int gridSize;
        public int leafCount;
    }

    private static readonly int[][] DIRS = new int[][]
    {
        new int[] { 1, 0, 0 }, new int[] { -1, 0, 0 },
        new int[] { 0, 1, 0 }, new int[] { 0, -1, 0 },
        new int[] { 0, 0, 1 }, new int[] { 0, 0, -1 },
    };

    private List<SVONode> nodes;

    public SVOData Build(float heightFactor = 0.0f)
    {
        int gs = gridSize;
        int maxLevel = (int)Math.Round(Math.Log(gs, 2));
        nodes = new List<SVONode>();

        BuildRecursive(0, 0, 0, gs, maxLevel);

        var leafNodes = new List<SVONode>();
        var voxelToLeaf = new int[gs * gs * gs];
        for (int i = 0; i < voxelToLeaf.Length; i++) voxelToLeaf[i] = -1;

        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (node.size == 1 && !node.occupied)
            {
                node.leafIdx = leafNodes.Count;
                node.leafX = node.ox;
                node.leafY = node.oy;
                node.leafZ = node.oz;
                leafNodes.Add(node);
                voxelToLeaf[Idx3(node.ox, node.oy, node.oz)] = node.leafIdx;
            }
        }

        var neighbors = new int[leafNodes.Count * 6];
        for (int i = 0; i < neighbors.Length; i++) neighbors[i] = -1;

        for (int i = 0; i < leafNodes.Count; i++)
        {
            var ln = leafNodes[i];
            for (int d = 0; d < 6; d++)
            {
                int nx = ln.leafX + DIRS[d][0];
                int ny = ln.leafY + DIRS[d][1];
                int nz = ln.leafZ + DIRS[d][2];
                if (nx >= 0 && nx < gs && ny >= 0 && ny < gs && nz >= 0 && nz < gs)
                {
                    neighbors[i * 6 + d] = voxelToLeaf[Idx3(nx, ny, nz)];
                }
            }
        }

        var leafCosts = new float[leafNodes.Count];
        for (int i = 0; i < leafNodes.Count; i++)
        {
            leafCosts[i] = 1.0f + heightFactor * leafNodes[i].leafY;
        }

        return new SVOData
        {
            nodes = nodes,
            leafNodes = leafNodes,
            neighbors = neighbors,
            voxelToLeaf = voxelToLeaf,
            leafCosts = leafCosts,
            gridSize = gs,
            leafCount = leafNodes.Count,
        };
    }

    private int BuildRecursive(int ox, int oy, int oz, int size, int level)
    {
        bool allOccupied = true;
        bool allEmpty = true;

        for (int dz = 0; dz < size && (allOccupied || allEmpty); dz++)
        {
            for (int dy = 0; dy < size && (allOccupied || allEmpty); dy++)
            {
                for (int dx = 0; dx < size && (allOccupied || allEmpty); dx++)
                {
                    if (GetVoxel(ox + dx, oy + dy, oz + dz))
                        allEmpty = false;
                    else
                        allOccupied = false;
                }
            }
        }

        bool isLeaf = (size == 1) || allOccupied;
        int nodeIdx = nodes.Count;

        var node = new SVONode
        {
            idx = nodeIdx,
            childBase = -1,
            parent = -1,
            level = level,
            occupied = allOccupied,
            isLeaf = isLeaf,
            ox = ox, oy = oy, oz = oz,
            size = size,
            leafIdx = -1,
        };
        nodes.Add(node);

        if (!isLeaf)
        {
            node.childBase = nodes.Count;
            int half = size / 2;
            var childIndices = new List<int>();

            for (int dz = 0; dz < 2; dz++)
            {
                for (int dy = 0; dy < 2; dy++)
                {
                    for (int dx = 0; dx < 2; dx++)
                    {
                        int ci = BuildRecursive(ox + dx * half, oy + dy * half, oz + dz * half, half, level - 1);
                        childIndices.Add(ci);
                    }
                }
            }

            foreach (int ci in childIndices)
            {
                nodes[ci].parent = nodeIdx;
            }
        }

        return nodeIdx;
    }
}