using System.Collections.Generic;
using System.Linq;
using System;
using Rhino.Geometry;

public class SpatialPaths
{
    public Dictionary<Guid, PathCurve> Paths { get; private set; } = new Dictionary<Guid, PathCurve>();
    public Dictionary<Guid, List<Guid>> CurveGraph { get; private set; } = new Dictionary<Guid, List<Guid>>();

    private double tolerance;

    public SpatialPaths(List<Line> lines, double tolerance = 1e-6)
    {
        this.tolerance = tolerance;

        foreach (var line in lines)
        {
            var path = new PathCurve(line, tolerance);
            Paths[path.Id] = path;
        }

        ComputeConnectivity();
        BuildGraph();
    }

    private void ComputeConnectivity()
    {
        foreach (var path in Paths.Values)
        {
            path.StartConnections.Clear();
            path.EndConnections.Clear();

            foreach (var other in Paths.Values)
            {
                if (path.Id == other.Id) continue;

                if (path.StartPoint.DistanceTo(other.StartPoint) < tolerance ||
                    path.StartPoint.DistanceTo(other.EndPoint) < tolerance)
                    path.StartConnections.Add(other.Id);

                if (path.EndPoint.DistanceTo(other.StartPoint) < tolerance ||
                    path.EndPoint.DistanceTo(other.EndPoint) < tolerance)
                    path.EndConnections.Add(other.Id);
            }
        }
    }

    private void BuildGraph()
    {
        CurveGraph.Clear();

        foreach (var path in Paths.Values)
        {
            HashSet<Guid> neighbors = new HashSet<Guid>();
            neighbors.UnionWith(path.StartConnections);
            neighbors.UnionWith(path.EndConnections);

            CurveGraph[path.Id] = neighbors.ToList();
        }
    }

    public Dictionary<string, int> GetOrientationStats()
    {
        return Paths.Values
            .GroupBy(p => p.Orientation)
            .ToDictionary(g => g.Key.ToString(), g => g.Count());
    }

    /// <summary>
    /// Finds clusters of PathCurves based on connectivity.
    /// </summary>
    /// <returns></returns>
    public List<List<Guid>> FindClusters()
    {
        var visited = new HashSet<Guid>();
        var clusters = new List<List<Guid>>();

        foreach (var curveId in Paths.Keys)
        {
            if (visited.Contains(curveId))
                continue;

            var cluster = new List<Guid>();
            var stack = new Stack<Guid>();
            stack.Push(curveId);

            while (stack.Count > 0)
            {
                var current = stack.Pop();
                if (visited.Contains(current)) continue;

                visited.Add(current);
                cluster.Add(current);

                if (CurveGraph.ContainsKey(current))
                {
                    foreach (var neighbor in CurveGraph[current])
                    {
                        if (!visited.Contains(neighbor))
                            stack.Push(neighbor);
                    }
                }
            }

            clusters.Add(cluster);
        }

        return clusters;
    }

    /// <summary>
    /// Finds clusters of PathCurves based on their Midpoint Z-coordinates, allowing for a specified tolerance.
    /// </summary>
    /// <param name="zTolerance"></param>
    /// <returns></returns>

    public List<List<Guid>> FindZLayeredClusters(double zTolerance = 1e-3)
    {
        var zGroups = GroupByZ(zTolerance);
        var allClusters = new List<List<Guid>>();

        foreach (var group in zGroups.Values)
        {
            var visited = new HashSet<Guid>();

            foreach (var curveId in group)
            {
                if (visited.Contains(curveId))
                    continue;

                var cluster = new List<Guid>();
                var stack = new Stack<Guid>();
                stack.Push(curveId);

                while (stack.Count > 0)
                {
                    var current = stack.Pop();
                    if (visited.Contains(current)) continue;

                    visited.Add(current);
                    cluster.Add(current);

                    if (CurveGraph.ContainsKey(current))
                    {
                        foreach (var neighbor in CurveGraph[current])
                        {
                            if (!visited.Contains(neighbor) && group.Contains(neighbor))
                                stack.Push(neighbor);
                        }
                    }
                }

                allClusters.Add(cluster);
            }
        }

        return allClusters;
    }

    /// <summary>
    /// Returns a list of clusters of PathCurves based on connectivity.
    /// </summary>
    /// <returns></returns>
    public List<List<Line>> GetCurveClusters()
    {
        //var idClusters = FindClusters();
        var idClusters = FindZLayeredClusters();

        return idClusters.Select(cluster => cluster.Select(id => Paths[id].Line).ToList()).ToList();
    }

    /// <summary>
    /// Groub by PathCurves based on their Midpoint Z-coordinate.
    /// </summary>
    /// <param name="zTolerance"></param>
    /// <returns></returns>
    private Dictionary<double, List<Guid>> GroupByZ(double zTolerance = 1e-3)
    {
        var zGroups = new Dictionary<double, List<Guid>>();

        foreach (var kvp in Paths)
        {
            var z = kvp.Value.MidpointZ;
            double zKey = Math.Round(z / zTolerance) * zTolerance;

            if (!zGroups.ContainsKey(zKey))
                zGroups[zKey] = new List<Guid>();

            zGroups[zKey].Add(kvp.Key);
        }

        return zGroups;
    }

    public List<(PathCurve Vertical, PathCurve Angled)> FindVerticalAngledPairs(List<Guid> clusterIds, double tolerance = 1e-6)
    {
        var pairs = new List<(PathCurve, PathCurve)>();
        var usedVerticals = new HashSet<Guid>();
        var usedAngleds = new HashSet<Guid>();

        var verticals = clusterIds
            .Select(id => Paths[id])
            .Where(p => p.Orientation == PathCurve.OrientationType.Vertical)
            .ToList();

        var angleds = clusterIds
            .Select(id => Paths[id])
            .Where(p => p.Orientation == PathCurve.OrientationType.AngledDown)
            .ToList();

        foreach (var v in verticals)
        {
            if (usedVerticals.Contains(v.Id)) continue;

            foreach (var a in angleds)
            {
                if (usedAngleds.Contains(a.Id)) continue;

                // Get all endpoints
                var endpoints = new[] { v.StartPoint, v.EndPoint, a.StartPoint, a.EndPoint };

                // Get highest Z point
                var maxZPoint = endpoints.OrderByDescending(pt => pt.Z).First();

                // Check if maxZPoint is shared between v and a
                bool vHas = IsCloseTo(v.StartPoint, maxZPoint, tolerance) || IsCloseTo(v.EndPoint, maxZPoint, tolerance);
                bool aHas = IsCloseTo(a.StartPoint, maxZPoint, tolerance) || IsCloseTo(a.EndPoint, maxZPoint, tolerance);

                if (vHas && aHas)
                {
                    pairs.Add((v, a));
                    usedVerticals.Add(v.Id);
                    usedAngleds.Add(a.Id);
                    break; // Move to next vertical
                }
            }
        }

        return pairs;
    }

    private bool IsCloseTo(Point3d a, Point3d b, double tolerance)
    {
        return a.DistanceTo(b) < tolerance;
    }

    public List<Polyline> BuildLongestChains(List<(PathCurve Vertical, PathCurve AngledDown)> pairs, double tolerance = 1e-6)
    {

        bool expectVerticalAtTail = true; // tail starts as Angled → next must be Vertical
        bool expectAngledAtHead = true;  // head starts as Vertical → next must be Angled

        var unused = new HashSet<int>(Enumerable.Range(0, pairs.Count));
        var polylines = new List<Polyline>();
        var pairPolys = pairs.Select(p => new PairPolyline(p.Vertical, p.AngledDown)).ToList();

        while (unused.Count > 0)
        {
            int startIdx = unused.First();
            unused.Remove(startIdx);

            var chainPts = new List<Point3d>(pairPolys[startIdx].Polyline);
            var used = new HashSet<int> { startIdx };

            bool extended;

            do
            {
                extended = false;
                Point3d head = chainPts.First();
                Point3d tail = chainPts.Last();

                // Get orientation at both ends
                bool headExpectAngled = true;  // since chain starts with vertical
                bool tailExpectVertical = true; // since chain ends with angled

                foreach (var i in unused.ToList())
                {
                    var pp = pairPolys[i];

                    // Tail connection: must be Vertical
                    if (expectVerticalAtTail && pp.Vertical != null &&
                        (IsCloseTo(pp.Vertical.StartPoint, tail, tolerance) || IsCloseTo(pp.Vertical.EndPoint, tail, tolerance)))
                    {
                        chainPts.AddRange(pp.Polyline.Skip(1));
                        unused.Remove(i);
                        expectVerticalAtTail = false; // next one would be angled
                        expectAngledAtHead = true;
                        extended = true;
                        break;
                    }

                    // Head connection: must be Angled
                    if (expectAngledAtHead && pp.AngledDown != null &&
                        (IsCloseTo(pp.AngledDown.StartPoint, head, tolerance) || IsCloseTo(pp.AngledDown.EndPoint, head, tolerance)))
                    {
                        var reversed = new List<Point3d>(pp.Polyline);
                        reversed.Reverse();
                        chainPts.InsertRange(0, reversed.Skip(1));
                        unused.Remove(i);
                        expectAngledAtHead = false; // next one would be vertical
                        expectVerticalAtTail = true;
                        extended = true;
                        break;
                    }
                }

            } while (extended);

            polylines.Add(new Polyline(chainPts));
        }

        return polylines;
    }

    public List<List<Polyline>> BuildClusteredChains(List<List<Guid>> clusters, double tolerance = 1e-6)
    {
        var allChains = new List<List<Polyline>>();

        foreach (var cluster in clusters)
        {
            var pairs = FindVerticalAngledPairs(cluster, tolerance);
            var chains = BuildLongestChains(pairs, tolerance);
            allChains.Add(chains);
        }

        return allChains;
    }
}

public class PairPolyline
{
    public PathCurve Vertical { get; }
    public PathCurve AngledDown { get; }
    public Polyline Polyline { get; }

    public Point3d Start => Polyline.First;
    public Point3d End => Polyline.Last;

    public PairPolyline(PathCurve v, PathCurve a)
    {
        Vertical = v;
        AngledDown = a;

        // Step 1: Find the shared point
        Point3d shared = GetSharedEndpoint(v.Line, a.Line);

        // Step 2: Determine the base of the vertical (lower Z point)
        Point3d vBase = v.StartPoint.Z < v.EndPoint.Z ? v.StartPoint : v.EndPoint;
        Point3d vTop = v.StartPoint.Z >= v.EndPoint.Z ? v.StartPoint : v.EndPoint;

        // Step 3: Determine the direction of the angled line (from shared to other point)
        Point3d aOther = shared.DistanceTo(a.StartPoint) < 1e-6 ? a.EndPoint : a.StartPoint;

        // Step 4: Build polyline starting from base of vertical → top of vertical → along angled
        var pts = new List<Point3d> { vBase, vTop, aOther };

        Polyline = new Polyline(pts);
    }

    private Point3d GetSharedEndpoint(Line l1, Line l2)
    {
        foreach (var p1 in new[] { l1.From, l1.To })
            foreach (var p2 in new[] { l2.From, l2.To })
                if (p1.DistanceTo(p2) < 1e-6)
                    return p1;

        throw new Exception("No shared endpoint found between vertical and angled lines.");
    }

    public bool StartsWithVertical => true; // enforced by definition
    public bool EndsWithAngled => true;     // enforced by order
}
