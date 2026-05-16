using System;
using System.Collections.Generic;
using Rhino;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Collections;
using System.Linq;
using Rhino.ApplicationSettings;
using Grasshopper.Kernel.Data;
using Grasshopper;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class N_Bracing : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public N_Bracing()
          : base("Find Vertical - Angled Connections", "N",
              "Reorders curve list to find curves connected to the end point of vertical curves",
              "FGAM", "Sorting")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("pathCurves", "pC", "a list of Curves", GH_ParamAccess.list);
            pManager.AddNumberParameter("Use Most Curves Weight", "WUse", "Higher values prefer vertical/angled choices that use more available curves.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Equal Direction Weight", "WDir", "Higher values prefer a balanced amount of angled curves in left, right, forward, and back directions.", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Longest Paths Weight", "WLen", "Higher values prefer longer connected print paths and fewer print segments.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Same Height Tolerance", "ZTol", "Clusters whose lowest point is within this Z tolerance are optimized together.", GH_ParamAccess.item, 0.01);
            pManager.AddNumberParameter("Connection Tolerance", "Tol", "Endpoint tolerance for clustering, chaining, and print-order search.", GH_ParamAccess.item, 0.01);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Curve Orientation", "pC", "Resorted curves", GH_ParamAccess.list);
            pManager.AddTextParameter("Curve Connectivity", "uC", "Resorted curves in pairs", GH_ParamAccess.list);
            pManager.AddCurveParameter("Cluster Curves", "cC", "Resorted curves in pairs", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Cluster Vertical Angled Pairs", "CVAP", "Resorted curves in pairs", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Connected Angled Pairs", "CAP", "Continious Path of Vertical Angled Lines", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Horizontal Longest Paths", "HLP", "Horizontal members ordered into longest connected Euler-style print paths.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Ordered Vertical Members", "V", "Ordered vertical members for Classified Member Weave.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Ordered Angled Members", "A", "Ordered angled members for Classified Member Weave.", GH_ParamAccess.tree);
            pManager.AddTextParameter("Weave Pattern", "P", "Pattern tokens for Classified Member Weave.", GH_ParamAccess.list);
            pManager.AddCurveParameter("SMT Print Order Preview", "SMT", "Preview of the intended print order before Classified Member Weave.", GH_ParamAccess.list);
            pManager.AddTextParameter("Optimization Report", "R", "Summary of weighted vertical/angled optimization and print order.", GH_ParamAccess.item);

        }

   
        
       

        public List<List<(PathCurve Vertical, PathCurve Angled)>> SortConnectedPairChains(
    List<(PathCurve Vertical, PathCurve Angled)> pairs,
    double tolerance = 1e-6)
        {
            var chains = new List<List<(PathCurve, PathCurve)>>();
            var unused = new HashSet<int>(Enumerable.Range(0, pairs.Count));

            // Build a lookup of endpoints → pair index
            var endpointMap = new Dictionary<Point3d, List<int>>(new Point3dComparer(tolerance));
            for (int i = 0; i < pairs.Count; i++)
            {
                var v = pairs[i].Vertical;
                var a = pairs[i].Angled;
                var pts = new[] { v.StartPoint, v.EndPoint, a.StartPoint, a.EndPoint };

                foreach (var pt in pts)
                {
                    if (!endpointMap.ContainsKey(pt))
                        endpointMap[pt] = new List<int>();
                    endpointMap[pt].Add(i);
                }
            }

            while (unused.Count > 0)
            {
                int seed = unused.First();
                unused.Remove(seed);

                var originalPair = pairs[seed];

                // Ensure the polyline starts at the base of the vertical
                var shared = GetSharedEndpoint(originalPair.Vertical.Line, originalPair.Angled.Line, tolerance);
                double zVStart = originalPair.Vertical.StartPoint.Z;
                double zVEnd = originalPair.Vertical.EndPoint.Z;
                double zShared = shared.Z;

                // If shared point is at the top of vertical (i.e., not the base), flip both lines
                bool sharedAtTopOfVertical = Math.Abs(zShared - Math.Max(zVStart, zVEnd)) < tolerance;
                (PathCurve, PathCurve) startingPair = originalPair;

                if (sharedAtTopOfVertical)
                {
                    var flippedVertical = new PathCurve(new Line(originalPair.Vertical.EndPoint, originalPair.Vertical.StartPoint), tolerance);
                    var flippedAngled = new PathCurve(new Line(originalPair.Angled.EndPoint, originalPair.Angled.StartPoint), tolerance);
                    startingPair = (flippedVertical, flippedAngled);
                }

                var chain = new List<(PathCurve, PathCurve)> { startingPair };
                var front = GetEndpoints(pairs[seed]);
                bool extended;

                do
                {
                    extended = false;

                    Point3d tail = front.tail;
                    Point3d head = GetEndpoints(chain[0]).head;

                    // Try to extend at the tail → match vertical start
                    if (endpointMap.TryGetValue(tail, out var tailCandidates))
                    {
                        foreach (int c in tailCandidates)
                        {
                            if (!unused.Contains(c)) continue;
                            var (v, a) = pairs[c];
                            if (IsSamePoint(v.StartPoint, tail, tolerance) || IsSamePoint(v.EndPoint, tail, tolerance))
                            {
                                chain.Add(pairs[c]);
                                unused.Remove(c);
                                front = GetEndpoints(pairs[c]);
                                extended = true;
                                break;
                            }
                        }
                    }

                    // Try to extend at the head → match angled end
                    if (!extended && endpointMap.TryGetValue(head, out var headCandidates))
                    {
                        foreach (int c in headCandidates)
                        {
                            if (!unused.Contains(c)) continue;
                            var (v, a) = pairs[c];
                            if (IsSamePoint(a.StartPoint, head, tolerance) || IsSamePoint(a.EndPoint, head, tolerance))
                            {
                                chain.Insert(0, pairs[c]);
                                unused.Remove(c);
                                extended = true;
                                break;
                            }
                        }
                    }

                } while (extended);

                chains.Add(chain);
            }

            return chains;
        }

        private (Point3d head, Point3d tail) GetEndpoints((PathCurve Vertical, PathCurve Angled) pair)
        {
            var shared = GetSharedEndpoint(pair.Vertical.Line, pair.Angled.Line, 1e-6);
            Point3d otherV = shared.DistanceTo(pair.Vertical.StartPoint) < 1e-6 ? pair.Vertical.EndPoint : pair.Vertical.StartPoint;
            Point3d otherA = shared.DistanceTo(pair.Angled.StartPoint) < 1e-6 ? pair.Angled.EndPoint : pair.Angled.StartPoint;
            return (otherV, otherA); // head = vertical base, tail = angled tip
        }

        private Point3d GetSharedEndpoint(Line l1, Line l2, double tol)
        {
            foreach (var p1 in new[] { l1.From, l1.To })
                foreach (var p2 in new[] { l2.From, l2.To })
                    if (p1.DistanceTo(p2) < tol)
                        return p1;

            throw new Exception("No shared point found");
        }

        //private bool IsSamePoint(Point3d a, Point3d b, double tol) => a.DistanceTo(b) < tol;

        class Point3dComparer : IEqualityComparer<Point3d>
        {
            private readonly double _tolerance;
            public Point3dComparer(double tol) => _tolerance = tol;

            public bool Equals(Point3d a, Point3d b) => a.DistanceTo(b) < _tolerance;

            public int GetHashCode(Point3d pt)
            {
                return (int)(pt.X / _tolerance) ^
                       (int)(pt.Y / _tolerance) ^
                       (int)(pt.Z / _tolerance);
            }
        }

        private bool IsSamePoint(Point3d a, Point3d b, double tol)
        {
            return a.DistanceTo(b) < tol;
        }


        private DataTree<Curve> ConvertToTree(List<List<Line>> clusters)
        {
            var tree = new DataTree<Curve>();
            for (int i = 0; i < clusters.Count; i++)
            {
                GH_Path path = new GH_Path(i);
                foreach (var line in clusters[i])
                {
                    tree.Add(new LineCurve(line), path); // Convert Line → Curve
                }
            }
            return tree;
        }

        public DataTree<Curve> ConvertPairChainsToTree(List<List<(PathCurve Vertical, PathCurve Angled)>> chains)
        {
            var tree = new DataTree<Curve>();

            for (int i = 0; i < chains.Count; i++)
            {
                GH_Path path = new GH_Path(i);

                foreach (var (v, a) in chains[i])
                {
                    tree.Add(new LineCurve(v.Line), path);
                    if (a != null)
                        tree.Add(new LineCurve(a.Line), path);
                }
            }

            return tree;
        }

        public List<List<(PathCurve Vertical, PathCurve Angled)>> TrimIntersectingTailAnglesOnly(
        List<List<(PathCurve Vertical, PathCurve Angled)>> chains,
        List<PathCurve> allCurves,
        double tolerance = 1e-6)
        {
            var trimmedChains = new List<List<(PathCurve, PathCurve)>>();

            foreach (var chain in chains)
            {
                if (chain.Count == 0)
                {
                    trimmedChains.Add(chain);
                    continue;
                }

                var trimmedChain = new List<(PathCurve, PathCurve)>(chain);

                var lastIndex = trimmedChain.Count - 1;
                (PathCurve Vertical, PathCurve Angled) lastPair = trimmedChain[lastIndex];
                var angledCurve = lastPair.Angled;

                bool intersects = false;

                foreach (var other in allCurves)
                {
                    if (other.Id == angledCurve.Id) continue;

                    var ccx = Rhino.Geometry.Intersect.Intersection.LineLine(
                        angledCurve.Line, other.Line, out double a, out double b, tolerance, false);

                    if (ccx)
                    {
                        intersects = true;
                        break;
                    }
                }

                if (intersects)
                {
                    // Replace the last pair with a new one that has only the vertical line
                    trimmedChain[lastIndex] = (lastPair.Vertical, null);
                }

                trimmedChains.Add(trimmedChain);
            }

            return trimmedChains;
        }

        private enum AngledDirection
        {
            Left,
            Right,
            Forward,
            Back
        }

        private sealed class OptimizationWeights
        {
            public double UseMostCurves;
            public double EqualDirection;
            public double LongestPaths;
            public double SameHeightTolerance;
            public double ConnectionTolerance;
        }

        private sealed class VerticalAngledPairCandidate
        {
            public PathCurve Vertical;
            public PathCurve Angled;
            public Line VerticalLine;
            public Line AngledLine;
            public Point3d Start;
            public Point3d Joint;
            public Point3d End;
            public AngledDirection Direction;
            public bool CulledAngledCollision;
            public bool HasAngled => !CulledAngledCollision && AngledLine.IsValid && AngledLine.Length > RhinoMath.ZeroTolerance;
            public double Length => VerticalLine.Length + (HasAngled ? AngledLine.Length : 0.0);
        }

        private sealed class VerticalAngledChainCandidate
        {
            public List<VerticalAngledPairCandidate> Pairs = new List<VerticalAngledPairCandidate>();
            public List<Line> StandaloneVerticalLines = new List<Line>();
            public double Score;
            public double Length => Pairs.Sum(pair => pair.Length) + StandaloneVerticalLines.Sum(line => line.Length);
            public int CurveCount => Pairs.Count + Pairs.Count(pair => pair.HasAngled) + StandaloneVerticalLines.Count;
            public double MinZ
            {
                get
                {
                    var zValues = Pairs
                        .Select(pair => Math.Min(pair.Start.Z, Math.Min(pair.Joint.Z, pair.End.Z)))
                        .Concat(StandaloneVerticalLines.Select(line => Math.Min(line.From.Z, line.To.Z)))
                        .ToList();

                    return zValues.Count == 0 ? double.MaxValue : zValues.Min();
                }
            }
            public Point3d Center => AveragePoint(
                Pairs.SelectMany(pair => new[] { pair.Start, pair.Joint, pair.End })
                    .Concat(StandaloneVerticalLines.SelectMany(line => new[] { line.From, line.To })));
        }

        private sealed class PrintSequenceItem
        {
            public string Kind;
            public List<Line> Lines = new List<Line>();
            public List<VerticalAngledPairCandidate> Pairs = new List<VerticalAngledPairCandidate>();
            public double MinZ;
            public Point3d Center;
            public int SupportGroup;
            public int SupportPriority;
            public int SequenceOrder;
        }

        private sealed class VerticalSupportEndpoint
        {
            public Point3d Point;
            public double BottomZ;
        }

        private sealed class HorizontalSupportMetrics
        {
            public int FullySupportedLines;
            public int SupportedEndpointCount;
            public int TerminalSupportedEndpointCount;
            public bool StartSupported;

            public int PriorityScore =>
                FullySupportedLines * 10000 +
                SupportedEndpointCount * 100 +
                TerminalSupportedEndpointCount * 10 +
                (StartSupported ? 1 : 0);
        }

        private sealed class UnsupportedHorizontalOption
        {
            public List<Line> Lines = new List<Line>();
            public List<Guid> Ids = new List<Guid>();
            public int Score;
            public double Length => Lines.Sum(line => line.Length);
            public bool EndsAtPrinted;
        }

        private sealed class HorizontalExtensionCandidate
        {
            public PathCurve Path;
            public Line Line;
            public int Score;
        }

        private struct PointGridKey : IEquatable<PointGridKey>
        {
            public int X;
            public int Y;
            public int Z;

            public PointGridKey(int x, int y, int z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            public bool Equals(PointGridKey other)
            {
                return X == other.X && Y == other.Y && Z == other.Z;
            }

            public override bool Equals(object obj)
            {
                return obj is PointGridKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 31 + X;
                    hash = hash * 31 + Y;
                    hash = hash * 31 + Z;
                    return hash;
                }
            }
        }

        private sealed class EndpointPathIndex
        {
            private readonly double _tolerance;
            private readonly double _cellSize;
            private readonly Dictionary<PointGridKey, List<PathCurve>> _pathsByCell = new Dictionary<PointGridKey, List<PathCurve>>();

            public EndpointPathIndex(IEnumerable<PathCurve> paths, double tolerance)
            {
                _tolerance = tolerance;
                _cellSize = Math.Max(tolerance, 1e-9);

                foreach (PathCurve path in paths)
                {
                    Add(path.StartPoint, path);
                    Add(path.EndPoint, path);
                }
            }

            public List<PathCurve> FindConnected(Point3d point)
            {
                var results = new List<PathCurve>();
                var seen = new HashSet<Guid>();

                foreach (PointGridKey key in NeighborKeys(point))
                {
                    if (!_pathsByCell.TryGetValue(key, out List<PathCurve> paths))
                    {
                        continue;
                    }

                    foreach (PathCurve path in paths)
                    {
                        if (seen.Contains(path.Id))
                        {
                            continue;
                        }

                        if (path.StartPoint.DistanceTo(point) <= _tolerance ||
                            path.EndPoint.DistanceTo(point) <= _tolerance)
                        {
                            seen.Add(path.Id);
                            results.Add(path);
                        }
                    }
                }

                return results;
            }

            public int CountConnected(Point3d point)
            {
                return FindConnected(point).Count;
            }

            private void Add(Point3d point, PathCurve path)
            {
                PointGridKey key = ToKey(point);
                if (!_pathsByCell.TryGetValue(key, out List<PathCurve> paths))
                {
                    paths = new List<PathCurve>();
                    _pathsByCell[key] = paths;
                }

                paths.Add(path);
            }

            private PointGridKey ToKey(Point3d point)
            {
                return new PointGridKey(
                    (int)Math.Floor(point.X / _cellSize),
                    (int)Math.Floor(point.Y / _cellSize),
                    (int)Math.Floor(point.Z / _cellSize));
            }

            private IEnumerable<PointGridKey> NeighborKeys(Point3d point)
            {
                PointGridKey center = ToKey(point);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            yield return new PointGridKey(center.X + x, center.Y + y, center.Z + z);
                        }
                    }
                }
            }
        }

        private sealed class VerticalSupportIndex
        {
            private readonly double _tolerance;
            private readonly double _cellSize;
            private readonly Dictionary<PointGridKey, List<VerticalSupportEndpoint>> _supportsByCell = new Dictionary<PointGridKey, List<VerticalSupportEndpoint>>();

            public VerticalSupportIndex(IEnumerable<VerticalSupportEndpoint> supports, double tolerance)
            {
                _tolerance = tolerance;
                _cellSize = Math.Max(tolerance, 1e-9);

                foreach (VerticalSupportEndpoint support in supports)
                {
                    Add(support);
                }
            }

            public bool IsSupported(Point3d point)
            {
                foreach (PointGridKey key in NeighborKeys(point))
                {
                    if (!_supportsByCell.TryGetValue(key, out List<VerticalSupportEndpoint> supports))
                    {
                        continue;
                    }

                    foreach (VerticalSupportEndpoint support in supports)
                    {
                        if (support.Point.DistanceTo(point) <= _tolerance &&
                            support.BottomZ < point.Z - _tolerance)
                        {
                            return true;
                        }
                    }
                }

                return false;
            }

            private void Add(VerticalSupportEndpoint support)
            {
                PointGridKey key = ToKey(support.Point);
                if (!_supportsByCell.TryGetValue(key, out List<VerticalSupportEndpoint> supports))
                {
                    supports = new List<VerticalSupportEndpoint>();
                    _supportsByCell[key] = supports;
                }

                supports.Add(support);
            }

            private PointGridKey ToKey(Point3d point)
            {
                return new PointGridKey(
                    (int)Math.Floor(point.X / _cellSize),
                    (int)Math.Floor(point.Y / _cellSize),
                    (int)Math.Floor(point.Z / _cellSize));
            }

            private IEnumerable<PointGridKey> NeighborKeys(Point3d point)
            {
                PointGridKey center = ToKey(point);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            yield return new PointGridKey(center.X + x, center.Y + y, center.Z + z);
                        }
                    }
                }
            }
        }

        private sealed class PrintedEndpointIndex
        {
            private readonly double _tolerance;
            private readonly double _cellSize;
            private readonly Dictionary<PointGridKey, List<Point3d>> _pointsByCell = new Dictionary<PointGridKey, List<Point3d>>();
            private readonly List<Line> _printedLines = new List<Line>();

            public PrintedEndpointIndex(double tolerance)
            {
                _tolerance = tolerance;
                _cellSize = Math.Max(tolerance, 1e-9);
            }

            public void Add(Point3d point)
            {
                PointGridKey key = ToKey(point);
                if (!_pointsByCell.TryGetValue(key, out List<Point3d> points))
                {
                    points = new List<Point3d>();
                    _pointsByCell[key] = points;
                }

                points.Add(point);
            }

            public void Add(Line line)
            {
                if (line.IsValid && line.Length > RhinoMath.ZeroTolerance)
                {
                    _printedLines.Add(line);
                }

                Add(line.From);
                Add(line.To);
            }

            public void AddRange(IEnumerable<Line> lines)
            {
                foreach (Line line in lines.Where(line => line.IsValid))
                {
                    Add(line);
                }
            }

            public bool Contains(Point3d point)
            {
                foreach (PointGridKey key in NeighborKeys(point))
                {
                    if (!_pointsByCell.TryGetValue(key, out List<Point3d> points))
                    {
                        continue;
                    }

                    if (points.Any(printedPoint => printedPoint.DistanceTo(point) <= _tolerance))
                    {
                        return true;
                    }
                }

                foreach (Line printedLine in _printedLines)
                {
                    Point3d closest = printedLine.ClosestPoint(point, true);
                    if (closest.DistanceTo(point) <= _tolerance)
                    {
                        return true;
                    }
                }

                return false;
            }

            public int SupportedEndpointCount(Line line)
            {
                return (Contains(line.From) ? 1 : 0) +
                       (Contains(line.To) ? 1 : 0);
            }

            public bool SupportsBothEndpoints(Line line)
            {
                return Contains(line.From) && Contains(line.To);
            }

            public bool TryOrientWithSupportedStart(PathCurve path, out Line line)
            {
                bool startSupported = Contains(path.StartPoint);
                bool endSupported = Contains(path.EndPoint);

                if (startSupported)
                {
                    line = path.Line;
                    return line.IsValid;
                }

                if (endSupported)
                {
                    line = new Line(path.EndPoint, path.StartPoint);
                    return line.IsValid;
                }

                line = Line.Unset;
                return false;
            }

            public bool TryOrientWithBothEndpointsSupported(PathCurve path, out Line line)
            {
                line = Line.Unset;
                if (!Contains(path.StartPoint) || !Contains(path.EndPoint))
                {
                    return false;
                }

                line = path.Line;
                return line.IsValid;
            }

            private PointGridKey ToKey(Point3d point)
            {
                return new PointGridKey(
                    (int)Math.Floor(point.X / _cellSize),
                    (int)Math.Floor(point.Y / _cellSize),
                    (int)Math.Floor(point.Z / _cellSize));
            }

            private IEnumerable<PointGridKey> NeighborKeys(Point3d point)
            {
                PointGridKey center = ToKey(point);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            yield return new PointGridKey(center.X + x, center.Y + y, center.Z + z);
                        }
                    }
                }
            }
        }

        private static List<List<Guid>> OrderClustersByHeight(SpatialPaths spatialPaths, List<List<Guid>> clusters, double tolerance)
        {
            return clusters
                .Where(cluster => cluster.Count > 0)
                .OrderBy(cluster => ClusterMinZ(spatialPaths, cluster))
                .ThenBy(cluster => ClusterCenter(spatialPaths, cluster).X)
                .ThenBy(cluster => ClusterCenter(spatialPaths, cluster).Y)
                .ToList();
        }

        private static List<List<Guid>> CombineClustersAtSameHeight(SpatialPaths spatialPaths, List<List<Guid>> orderedClusters, double zTolerance)
        {
            var combined = new List<List<Guid>>();
            double currentZ = double.NaN;

            foreach (List<Guid> cluster in orderedClusters)
            {
                double clusterZ = ClusterMinZ(spatialPaths, cluster);
                if (combined.Count == 0 || Math.Abs(clusterZ - currentZ) > zTolerance)
                {
                    combined.Add(new List<Guid>(cluster));
                    currentZ = clusterZ;
                }
                else
                {
                    combined[combined.Count - 1].AddRange(cluster);
                }
            }

            return combined;
        }

        private static double ClusterMinZ(SpatialPaths spatialPaths, List<Guid> cluster)
        {
            return cluster
                .Where(id => spatialPaths.Paths.ContainsKey(id))
                .Select(id => spatialPaths.Paths[id])
                .Min(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z));
        }

        private static Point3d ClusterCenter(SpatialPaths spatialPaths, List<Guid> cluster)
        {
            return AveragePoint(cluster
                .Where(id => spatialPaths.Paths.ContainsKey(id))
                .Select(id => spatialPaths.Paths[id].MidPoint));
        }

        private static Point3d AveragePoint(IEnumerable<Point3d> points)
        {
            var valid = points.Where(point => point.IsValid).ToList();
            if (valid.Count == 0)
            {
                return Point3d.Origin;
            }

            return new Point3d(
                valid.Average(point => point.X),
                valid.Average(point => point.Y),
                valid.Average(point => point.Z));
        }

        private static List<VerticalAngledPairCandidate> BuildVerticalAngledPairCandidates(
            SpatialPaths spatialPaths,
            List<Guid> clusterIds,
            double tolerance)
        {
            var verticals = clusterIds
                .Where(id => spatialPaths.Paths.ContainsKey(id))
                .Select(id => spatialPaths.Paths[id])
                .Where(path => path.Orientation == PathCurve.OrientationType.Vertical)
                .ToList();

            var angleds = clusterIds
                .Where(id => spatialPaths.Paths.ContainsKey(id))
                .Select(id => spatialPaths.Paths[id])
                .Where(path => path.Orientation == PathCurve.OrientationType.AngledDown ||
                               path.Orientation == PathCurve.OrientationType.AngledUp)
                .ToList();

            var pairs = new List<VerticalAngledPairCandidate>();
            var angledEndpointIndex = new EndpointPathIndex(angleds, tolerance);
            foreach (PathCurve vertical in verticals)
            {
                Point3d verticalTop = vertical.StartPoint.Z >= vertical.EndPoint.Z
                    ? vertical.StartPoint
                    : vertical.EndPoint;

                foreach (PathCurve angled in angledEndpointIndex.FindConnected(verticalTop))
                {
                    if (TryCreateVerticalAngledPair(vertical, angled, tolerance, out VerticalAngledPairCandidate pair))
                    {
                        pairs.Add(pair);
                    }
                }
            }

            return pairs
                .OrderBy(pair => pair.Start.Z)
                .ThenBy(pair => pair.Start.X)
                .ThenBy(pair => pair.Start.Y)
                .ToList();
        }

        private static bool TryCreateVerticalAngledPair(
            PathCurve vertical,
            PathCurve angled,
            double tolerance,
            out VerticalAngledPairCandidate pair)
        {
            pair = null;

            if (!TryGetSharedEndpoint(vertical.Line, angled.Line, tolerance, out Point3d shared))
            {
                return false;
            }

            Point3d[] endpoints = { vertical.StartPoint, vertical.EndPoint, angled.StartPoint, angled.EndPoint };
            double maxZ = endpoints.Max(point => point.Z);
            if (Math.Abs(shared.Z - maxZ) > tolerance)
            {
                return false;
            }

            Point3d verticalOther = OtherEndpoint(vertical.Line, shared, tolerance);
            Point3d angledOther = OtherEndpoint(angled.Line, shared, tolerance);

            if (verticalOther.Z > shared.Z + tolerance || angledOther.Z > shared.Z + tolerance)
            {
                return false;
            }

            Line verticalLine = new Line(verticalOther, shared);
            Line angledLine = new Line(shared, angledOther);
            if (!verticalLine.IsValid || !angledLine.IsValid ||
                verticalLine.Length <= RhinoMath.ZeroTolerance ||
                angledLine.Length <= RhinoMath.ZeroTolerance)
            {
                return false;
            }

            pair = new VerticalAngledPairCandidate
            {
                Vertical = vertical,
                Angled = angled,
                VerticalLine = verticalLine,
                AngledLine = angledLine,
                Start = verticalLine.From,
                Joint = shared,
                End = angledLine.To,
                Direction = ClassifyAngledDirection(angledLine)
            };
            return true;
        }

        private static bool TryGetSharedEndpoint(Line first, Line second, double tolerance, out Point3d shared)
        {
            foreach (Point3d firstPoint in new[] { first.From, first.To })
            {
                foreach (Point3d secondPoint in new[] { second.From, second.To })
                {
                    if (firstPoint.DistanceTo(secondPoint) <= tolerance)
                    {
                        shared = firstPoint;
                        return true;
                    }
                }
            }

            shared = Point3d.Unset;
            return false;
        }

        private static Point3d OtherEndpoint(Line line, Point3d endpoint, double tolerance)
        {
            return line.From.DistanceTo(endpoint) <= tolerance ? line.To : line.From;
        }

        private static AngledDirection ClassifyAngledDirection(Line line)
        {
            Vector3d xy = new Vector3d(line.Direction.X, line.Direction.Y, 0.0);
            if (Math.Abs(xy.X) >= Math.Abs(xy.Y))
            {
                return xy.X >= 0.0 ? AngledDirection.Right : AngledDirection.Left;
            }

            return xy.Y >= 0.0 ? AngledDirection.Forward : AngledDirection.Back;
        }

        private static List<VerticalAngledChainCandidate> SelectOptimizedVerticalAngledChains(
            List<VerticalAngledPairCandidate> pairCandidates,
            OptimizationWeights weights)
        {
            var selected = new List<VerticalAngledChainCandidate>();
            var usedVerticals = new HashSet<Guid>();
            var usedAngleds = new HashSet<Guid>();

            while (true)
            {
                List<VerticalAngledPairCandidate> available = pairCandidates
                    .Where(pair => !usedVerticals.Contains(pair.Vertical.Id) &&
                                   !usedAngleds.Contains(pair.Angled.Id))
                    .ToList();

                if (available.Count == 0)
                {
                    break;
                }

                List<VerticalAngledChainCandidate> candidates = BuildVerticalAngledChainCandidates(available, weights.ConnectionTolerance);
                if (candidates.Count == 0)
                {
                    break;
                }

                ScoreVerticalAngledChains(candidates, weights);
                VerticalAngledChainCandidate best = candidates
                    .OrderByDescending(candidate => candidate.Score)
                    .ThenByDescending(candidate => candidate.Pairs.Count)
                    .ThenByDescending(candidate => candidate.Length)
                    .ThenBy(candidate => candidate.MinZ)
                    .ThenBy(candidate => candidate.Center.X)
                    .ThenBy(candidate => candidate.Center.Y)
                    .First();

                selected.Add(best);
                foreach (VerticalAngledPairCandidate pair in best.Pairs)
                {
                    usedVerticals.Add(pair.Vertical.Id);
                    usedAngleds.Add(pair.Angled.Id);
                }
            }

            return selected
                .OrderBy(chain => chain.MinZ)
                .ThenByDescending(chain => chain.Pairs.Count)
                .ThenBy(chain => chain.Center.X)
                .ThenBy(chain => chain.Center.Y)
                .ToList();
        }

        private static int AppendUnusedVerticalsToEndOfGroup(
            SpatialPaths spatialPaths,
            List<Guid> groupIds,
            List<VerticalAngledChainCandidate> selectedChains,
            double tolerance)
        {
            var usedVerticals = new HashSet<Guid>(
                selectedChains
                    .SelectMany(chain => chain.Pairs)
                    .Select(pair => pair.Vertical.Id));

            List<Line> unusedVerticalLines = groupIds
                .Where(id => spatialPaths.Paths.ContainsKey(id))
                .Select(id => spatialPaths.Paths[id])
                .Where(path => path.Orientation == PathCurve.OrientationType.Vertical &&
                               !usedVerticals.Contains(path.Id))
                .OrderBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .Select(path => OrientVerticalBottomToTop(path.Line, tolerance))
                .Where(line => line.IsValid && line.Length > RhinoMath.ZeroTolerance)
                .ToList();

            int appended = 0;
            var unattachedVerticalLines = new List<Line>();
            foreach (Line unusedVerticalLine in unusedVerticalLines)
            {
                if (TryAttachStandaloneVerticalToConnectedChain(unusedVerticalLine, selectedChains, tolerance))
                {
                    appended++;
                }
                else
                {
                    unattachedVerticalLines.Add(unusedVerticalLine);
                }
            }

            if (unattachedVerticalLines.Count == 0)
            {
                return appended;
            }

            VerticalAngledChainCandidate target = selectedChains
                .OrderBy(chain => chain.MinZ)
                .ThenBy(chain => chain.Center.X)
                .ThenBy(chain => chain.Center.Y)
                .LastOrDefault();

            if (target == null)
            {
                target = new VerticalAngledChainCandidate();
                selectedChains.Add(target);
            }

            target.StandaloneVerticalLines.AddRange(unattachedVerticalLines);
            return appended + unattachedVerticalLines.Count;
        }

        private static Line OrientVerticalBottomToTop(Line line, double tolerance)
        {
            if (line.From.Z <= line.To.Z + tolerance)
            {
                return line;
            }

            return new Line(line.To, line.From);
        }

        private static bool TryAttachStandaloneVerticalToConnectedChain(
            Line verticalLine,
            List<VerticalAngledChainCandidate> selectedChains,
            double tolerance)
        {
            Point3d verticalBottom = verticalLine.From.Z <= verticalLine.To.Z + tolerance
                ? verticalLine.From
                : verticalLine.To;

            VerticalAngledChainCandidate target = null;
            double bestDistance = double.MaxValue;
            foreach (VerticalAngledChainCandidate chain in selectedChains)
            {
                if (!TryGetVerticalAngledChainTail(chain, out Point3d tail))
                {
                    continue;
                }

                double distance = tail.DistanceTo(verticalBottom);
                if (distance > tolerance)
                {
                    continue;
                }

                if (target == null ||
                    distance < bestDistance ||
                    (Math.Abs(distance - bestDistance) <= tolerance && chain.MinZ < target.MinZ))
                {
                    target = chain;
                    bestDistance = distance;
                }
            }

            if (target == null)
            {
                return false;
            }

            target.StandaloneVerticalLines.Add(OrientVerticalBottomToTop(verticalLine, tolerance));
            return true;
        }

        private static bool TryGetVerticalAngledChainTail(VerticalAngledChainCandidate chain, out Point3d tail)
        {
            if (chain.StandaloneVerticalLines.Count > 0)
            {
                tail = chain.StandaloneVerticalLines[chain.StandaloneVerticalLines.Count - 1].To;
                return true;
            }

            if (chain.Pairs.Count > 0)
            {
                tail = chain.Pairs[chain.Pairs.Count - 1].End;
                return true;
            }

            tail = Point3d.Unset;
            return false;
        }

        private static int CullAngledDownIntersectionsWithPrintedVerticals(
            List<VerticalAngledChainCandidate> selectedChains,
            double tolerance)
        {
            int culled = 0;
            var printedVerticals = new List<Line>();

            foreach (VerticalAngledChainCandidate chain in selectedChains
                .OrderBy(chain => chain.MinZ)
                .ThenBy(chain => chain.Center.X)
                .ThenBy(chain => chain.Center.Y))
            {
                foreach (VerticalAngledPairCandidate pair in chain.Pairs)
                {
                    if (pair.VerticalLine.IsValid && pair.VerticalLine.Length > RhinoMath.ZeroTolerance)
                    {
                        printedVerticals.Add(pair.VerticalLine);
                    }

                    if (!IsAngledDownPrintMove(pair, tolerance))
                    {
                        continue;
                    }

                    if (AngledLineHitsPrintedVertical(pair, printedVerticals, tolerance))
                    {
                        pair.CulledAngledCollision = true;
                        culled++;
                    }
                }

                foreach (Line standaloneVertical in chain.StandaloneVerticalLines)
                {
                    if (standaloneVertical.IsValid && standaloneVertical.Length > RhinoMath.ZeroTolerance)
                    {
                        printedVerticals.Add(standaloneVertical);
                    }
                }
            }

            return culled;
        }

        private static bool IsAngledDownPrintMove(VerticalAngledPairCandidate pair, double tolerance)
        {
            return pair.Angled != null &&
                   (pair.Angled.Orientation == PathCurve.OrientationType.AngledDown ||
                    pair.AngledLine.To.Z < pair.AngledLine.From.Z - tolerance);
        }

        private static bool AngledLineHitsPrintedVertical(
            VerticalAngledPairCandidate pair,
            List<Line> printedVerticals,
            double tolerance)
        {
            foreach (Line printedVertical in printedVerticals)
            {
                if (!printedVertical.IsValid || printedVertical.Length <= RhinoMath.ZeroTolerance)
                {
                    continue;
                }

                if (!Rhino.Geometry.Intersect.Intersection.LineLine(
                    pair.AngledLine,
                    printedVertical,
                    out double angledParameter,
                    out double verticalParameter,
                    tolerance,
                    true))
                {
                    continue;
                }

                Point3d angledPoint = pair.AngledLine.PointAt(angledParameter);
                Point3d verticalPoint = printedVertical.PointAt(verticalParameter);
                if (angledPoint.DistanceTo(verticalPoint) > tolerance)
                {
                    continue;
                }

                if (angledPoint.DistanceTo(pair.Joint) <= tolerance)
                {
                    continue;
                }

                if (IsPrintedVerticalStartContact(pair, printedVertical, angledPoint, verticalPoint, tolerance))
                {
                    return true;
                }

                if (IsAllowedAngledEndpointContact(
                    pair,
                    printedVertical,
                    angledPoint,
                    verticalPoint,
                    verticalParameter,
                    tolerance))
                {
                    continue;
                }

                return true;
            }

            return false;
        }

        private static bool IsPrintedVerticalStartContact(
            VerticalAngledPairCandidate pair,
            Line printedVertical,
            Point3d angledPoint,
            Point3d verticalPoint,
            double tolerance)
        {
            return angledPoint.DistanceTo(pair.End) <= tolerance &&
                   verticalPoint.DistanceTo(pair.End) <= tolerance &&
                   verticalPoint.DistanceTo(printedVertical.From) <= tolerance;
        }

        private static bool IsAllowedAngledEndpointContact(
            VerticalAngledPairCandidate pair,
            Line printedVertical,
            Point3d angledPoint,
            Point3d verticalPoint,
            double verticalParameter,
            double tolerance)
        {
            return angledPoint.DistanceTo(pair.End) <= tolerance &&
                   verticalPoint.DistanceTo(pair.End) <= tolerance &&
                   IsLineEndPointContact(printedVertical, verticalParameter, verticalPoint, tolerance);
        }

        private static bool IsLineEndPointContact(
            Line line,
            double parameter,
            Point3d point,
            double tolerance)
        {
            double parameterTolerance = Math.Max(1e-6, tolerance / Math.Max(line.Length, tolerance));
            return parameter >= 1.0 - parameterTolerance ||
                   point.DistanceTo(line.To) <= tolerance;
        }

        private static List<VerticalAngledChainCandidate> BuildVerticalAngledChainCandidates(
            List<VerticalAngledPairCandidate> available,
            double tolerance)
        {
            var candidates = new List<VerticalAngledChainCandidate>();
            const int maxCandidates = 4096;

            foreach (VerticalAngledPairCandidate start in available)
            {
                ExploreVerticalAngledChains(
                    available,
                    new List<VerticalAngledPairCandidate> { start },
                    new HashSet<Guid> { start.Vertical.Id },
                    new HashSet<Guid> { start.Angled.Id },
                    candidates,
                    tolerance,
                    maxCandidates);

                if (candidates.Count >= maxCandidates)
                {
                    break;
                }
            }

            return candidates;
        }

        private static void ExploreVerticalAngledChains(
            List<VerticalAngledPairCandidate> available,
            List<VerticalAngledPairCandidate> current,
            HashSet<Guid> usedVerticals,
            HashSet<Guid> usedAngleds,
            List<VerticalAngledChainCandidate> candidates,
            double tolerance,
            int maxCandidates)
        {
            if (candidates.Count >= maxCandidates)
            {
                return;
            }

            candidates.Add(new VerticalAngledChainCandidate
            {
                Pairs = new List<VerticalAngledPairCandidate>(current)
            });

            Point3d tail = current[current.Count - 1].End;
            List<VerticalAngledPairCandidate> nextPairs = available
                .Where(pair => !usedVerticals.Contains(pair.Vertical.Id) &&
                               !usedAngleds.Contains(pair.Angled.Id) &&
                               pair.Start.DistanceTo(tail) <= tolerance)
                .OrderBy(pair => pair.Start.DistanceTo(tail))
                .ThenBy(pair => pair.End.Z)
                .ThenBy(pair => pair.End.X)
                .ThenBy(pair => pair.End.Y)
                .ToList();

            foreach (VerticalAngledPairCandidate next in nextPairs)
            {
                current.Add(next);
                usedVerticals.Add(next.Vertical.Id);
                usedAngleds.Add(next.Angled.Id);

                ExploreVerticalAngledChains(available, current, usedVerticals, usedAngleds, candidates, tolerance, maxCandidates);

                current.RemoveAt(current.Count - 1);
                usedVerticals.Remove(next.Vertical.Id);
                usedAngleds.Remove(next.Angled.Id);
            }
        }

        private static void ScoreVerticalAngledChains(
            List<VerticalAngledChainCandidate> candidates,
            OptimizationWeights weights)
        {
            double maxCurveCount = Math.Max(1.0, candidates.Max(candidate => candidate.CurveCount));
            double maxLength = Math.Max(1.0, candidates.Max(candidate => candidate.Length));
            double maxPairCount = Math.Max(1.0, candidates.Max(candidate => candidate.Pairs.Count));

            foreach (VerticalAngledChainCandidate candidate in candidates)
            {
                double useScore = candidate.CurveCount / maxCurveCount;
                double pathScore = 0.5 * (candidate.Pairs.Count / maxPairCount) + 0.5 * (candidate.Length / maxLength);
                double directionScore = DirectionBalanceScore(candidate.Pairs);

                candidate.Score =
                    weights.UseMostCurves * useScore +
                    weights.LongestPaths * pathScore +
                    weights.EqualDirection * directionScore;
            }
        }

        private static double DirectionBalanceScore(List<VerticalAngledPairCandidate> pairs)
        {
            pairs = pairs.Where(pair => pair.HasAngled).ToList();
            if (pairs.Count == 0)
            {
                return 0.0;
            }

            var counts = new Dictionary<AngledDirection, int>
            {
                { AngledDirection.Left, 0 },
                { AngledDirection.Right, 0 },
                { AngledDirection.Forward, 0 },
                { AngledDirection.Back, 0 }
            };

            foreach (VerticalAngledPairCandidate pair in pairs)
            {
                counts[pair.Direction]++;
            }

            double ideal = pairs.Count / 4.0;
            double deviation = counts.Values.Sum(count => Math.Abs(count - ideal));
            double maxDeviation = Math.Max(1.0, pairs.Count * 1.5);
            return Math.Max(0.0, 1.0 - deviation / maxDeviation);
        }

        private static List<VerticalSupportEndpoint> BuildVerticalSupportEndpoints(List<VerticalAngledChainCandidate> selectedChains, double tolerance)
        {
            var supports = new List<VerticalSupportEndpoint>();

            foreach (VerticalAngledPairCandidate pair in selectedChains.SelectMany(chain => chain.Pairs))
            {
                Line vertical = pair.VerticalLine;
                Point3d top = vertical.From.Z >= vertical.To.Z ? vertical.From : vertical.To;
                Point3d bottom = vertical.From.Z < vertical.To.Z ? vertical.From : vertical.To;

                if (top.Z > bottom.Z + tolerance)
                {
                    supports.Add(new VerticalSupportEndpoint
                    {
                        Point = top,
                        BottomZ = bottom.Z
                    });
                }
            }

            foreach (Line vertical in selectedChains.SelectMany(chain => chain.StandaloneVerticalLines))
            {
                Point3d top = vertical.From.Z >= vertical.To.Z ? vertical.From : vertical.To;
                Point3d bottom = vertical.From.Z < vertical.To.Z ? vertical.From : vertical.To;

                if (top.Z > bottom.Z + tolerance)
                {
                    supports.Add(new VerticalSupportEndpoint
                    {
                        Point = top,
                        BottomZ = bottom.Z
                    });
                }
            }

            return supports;
        }

        private static double GetFirstHorizontalLayerZ(IEnumerable<PathCurve> horizontals)
        {
            var zValues = horizontals
                .Select(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ToList();

            return zValues.Count == 0 ? double.NaN : zValues.Min();
        }

        private static double GetLineLayerZ(Line line)
        {
            return Math.Min(line.From.Z, line.To.Z);
        }

        private static bool IsFirstHorizontalLayer(List<Line> chain, double firstHorizontalZ, double tolerance)
        {
            return chain.Count == 0 ||
                   double.IsNaN(firstHorizontalZ) ||
                   chain.Min(GetLineLayerZ) <= firstHorizontalZ + tolerance;
        }

        private static bool IsPointSupportedByVertical(Point3d point, VerticalSupportIndex supports)
        {
            return supports != null && supports.IsSupported(point);
        }

        private static HorizontalSupportMetrics GetHorizontalSupportMetrics(
            List<Line> chain,
            VerticalSupportIndex supports,
            double firstHorizontalZ,
            double tolerance)
        {
            var metrics = new HorizontalSupportMetrics();
            if (chain.Count == 0 || supports == null || IsFirstHorizontalLayer(chain, firstHorizontalZ, tolerance))
            {
                return metrics;
            }

            foreach (Line line in chain)
            {
                bool fromSupported = IsPointSupportedByVertical(line.From, supports);
                bool toSupported = IsPointSupportedByVertical(line.To, supports);

                if (fromSupported && toSupported)
                {
                    metrics.FullySupportedLines++;
                }

                if (fromSupported)
                {
                    metrics.SupportedEndpointCount++;
                }

                if (toSupported)
                {
                    metrics.SupportedEndpointCount++;
                }
            }

            metrics.StartSupported = IsPointSupportedByVertical(chain[0].From, supports);
            bool endSupported = IsPointSupportedByVertical(chain[chain.Count - 1].To, supports);
            metrics.TerminalSupportedEndpointCount =
                (metrics.StartSupported ? 1 : 0) +
                (endSupported ? 1 : 0);

            return metrics;
        }

        private static bool IsFullySupportedHorizontalLine(
            Line line,
            VerticalSupportIndex supports,
            double firstHorizontalZ,
            double tolerance)
        {
            if (GetLineLayerZ(line) <= firstHorizontalZ + tolerance)
            {
                return true;
            }

            return IsPointSupportedByVertical(line.From, supports) &&
                   IsPointSupportedByVertical(line.To, supports);
        }

        private static bool IsFullySupportedHorizontalPath(
            PathCurve path,
            VerticalSupportIndex supports,
            double firstHorizontalZ,
            double tolerance)
        {
            return Math.Min(path.StartPoint.Z, path.EndPoint.Z) <= firstHorizontalZ + tolerance ||
                   (IsPointSupportedByVertical(path.StartPoint, supports) &&
                    IsPointSupportedByVertical(path.EndPoint, supports));
        }

        private static int GetHorizontalSupportGroup(
            List<Line> chain,
            VerticalSupportIndex supports,
            double firstHorizontalZ,
            double tolerance)
        {
            return chain.All(line => IsFullySupportedHorizontalLine(line, supports, firstHorizontalZ, tolerance))
                ? 0
                : 1;
        }

        private static List<Line> ReverseHorizontalChain(List<Line> chain)
        {
            return chain
                .AsEnumerable()
                .Reverse()
                .Select(line => new Line(line.To, line.From))
                .ToList();
        }

        private static List<Line> OrientHorizontalChainForSupportedStart(
            List<Line> chain,
            VerticalSupportIndex supports,
            double firstHorizontalZ,
            double tolerance)
        {
            if (chain.Count == 0 || IsFirstHorizontalLayer(chain, firstHorizontalZ, tolerance))
            {
                return chain;
            }

            HorizontalSupportMetrics current = GetHorizontalSupportMetrics(chain, supports, firstHorizontalZ, tolerance);
            List<Line> reversed = ReverseHorizontalChain(chain);
            HorizontalSupportMetrics reversedMetrics = GetHorizontalSupportMetrics(reversed, supports, firstHorizontalZ, tolerance);

            return reversedMetrics.PriorityScore > current.PriorityScore ? reversed : chain;
        }

        private static bool IsBetterHorizontalChain(
            List<Line> candidate,
            HorizontalSupportMetrics candidateSupport,
            List<Line> best,
            HorizontalSupportMetrics bestSupport,
            double tolerance)
        {
            if (candidate.Count == 0)
            {
                return false;
            }

            if (best.Count == 0 || bestSupport == null)
            {
                return true;
            }

            if (candidateSupport.PriorityScore != bestSupport.PriorityScore)
            {
                return candidateSupport.PriorityScore > bestSupport.PriorityScore;
            }

            if (candidate.Count != best.Count)
            {
                return candidate.Count > best.Count;
            }

            return candidate.Sum(line => line.Length) > best.Sum(line => line.Length) + tolerance;
        }

        private static List<List<Line>> BuildHorizontalLongestPaths(
            SpatialPaths spatialPaths,
            VerticalSupportIndex verticalSupports,
            IEnumerable<Line> printedSeedLines,
            double tolerance,
            out int unscheduledUnsupportedHorizontals)
        {
            unscheduledUnsupportedHorizontals = 0;
            List<PathCurve> horizontals = spatialPaths.Paths.Values
                .Where(path => path.Orientation == PathCurve.OrientationType.Horizontal)
                .OrderBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .ToList();

            double firstHorizontalZ = GetFirstHorizontalLayerZ(horizontals);
            var paths = new List<List<Line>>();
            var printedGraph = new PrintedEndpointIndex(tolerance);
            List<Line> seedLines = (printedSeedLines ?? Enumerable.Empty<Line>())
                .Where(line => line.IsValid && line.Length > RhinoMath.ZeroTolerance)
                .OrderBy(line => Math.Max(line.From.Z, line.To.Z))
                .ThenBy(line => Math.Min(line.From.Z, line.To.Z))
                .ThenBy(line => line.PointAt(0.5).X)
                .ThenBy(line => line.PointAt(0.5).Y)
                .ToList();
            var seededLineIndices = new HashSet<int>();

            List<List<PathCurve>> horizontalComponents = BuildHorizontalComponents(horizontals, tolerance)
                .OrderBy(component => component.Min(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z)))
                .ThenBy(component => AveragePoint(component.Select(path => path.MidPoint)).X)
                .ThenBy(component => AveragePoint(component.Select(path => path.MidPoint)).Y)
                .ToList();

            foreach (List<PathCurve> component in horizontalComponents)
            {
                double componentZ = component.Min(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z));
                AddSeedLinesUpToLayer(seedLines, seededLineIndices, printedGraph, componentZ, tolerance);

                List<List<Line>> componentPaths;
                if (componentZ <= firstHorizontalZ + tolerance)
                {
                    componentPaths = BuildLongestLineChains(
                            component,
                            verticalSupports,
                            firstHorizontalZ,
                            tolerance)
                        .Where(path => path.Count > 0)
                        .Select(path => OrientHorizontalChainForSupportedStart(path, verticalSupports, firstHorizontalZ, tolerance))
                        .OrderBy(path => path.Min(line => Math.Min(line.From.Z, line.To.Z)))
                        .ThenByDescending(path => path.Count)
                        .ThenByDescending(path => path.Sum(line => line.Length))
                        .ThenBy(path => AveragePoint(path.Select(line => line.PointAt(0.5))).X)
                        .ThenBy(path => AveragePoint(path.Select(line => line.PointAt(0.5))).Y)
                        .ToList();
                }
                else
                {
                    componentPaths = BuildGraphSupportedHorizontalPaths(
                        component,
                        printedGraph,
                        tolerance,
                        out int unscheduledInComponent);
                    unscheduledUnsupportedHorizontals += unscheduledInComponent;
                }

                paths.AddRange(componentPaths);
                foreach (List<Line> componentPath in componentPaths)
                {
                    printedGraph.AddRange(componentPath);
                }
            }

            return paths
                .Where(path => path.Count > 0)
                .ToList();
        }

        private static void AddSeedLinesUpToLayer(
            List<Line> seedLines,
            HashSet<int> seededLineIndices,
            PrintedEndpointIndex printedGraph,
            double layerZ,
            double tolerance)
        {
            for (int i = 0; i < seedLines.Count; i++)
            {
                if (seededLineIndices.Contains(i))
                {
                    continue;
                }

                Line line = seedLines[i];
                if (Math.Max(line.From.Z, line.To.Z) <= layerZ + tolerance)
                {
                    printedGraph.Add(line);
                    seededLineIndices.Add(i);
                }
            }
        }

        private static List<List<Line>> BuildGraphSupportedHorizontalPaths(
            List<PathCurve> horizontals,
            PrintedEndpointIndex printedGraph,
            double tolerance,
            out int unscheduledHorizontals)
        {
            unscheduledHorizontals = 0;
            var ordered = horizontals
                .OrderBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .ToList();
            var remaining = new HashSet<Guid>(ordered.Select(path => path.Id));
            var endpointIndex = new EndpointPathIndex(ordered, tolerance);
            var paths = new List<List<Line>>();

            while (remaining.Count > 0)
            {
                UnsupportedHorizontalOption option = FindBestUnsupportedHorizontalOption(
                    ordered,
                    remaining,
                    endpointIndex,
                    printedGraph,
                    tolerance);

                if (option == null || option.Lines.Count == 0)
                {
                    unscheduledHorizontals += remaining.Count;
                    break;
                }

                foreach (Guid id in option.Ids)
                {
                    remaining.Remove(id);
                }

                paths.Add(option.Lines);
                printedGraph.AddRange(option.Lines);
            }

            return paths;
        }

        private static UnsupportedHorizontalOption FindBestUnsupportedHorizontalOption(
            List<PathCurve> ordered,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            UnsupportedHorizontalOption bestSupportedChain = FindBestPrintedToPrintedChain(
                ordered,
                remaining,
                endpointIndex,
                printedEndpoints,
                tolerance);
            if (bestSupportedChain != null)
            {
                return bestSupportedChain;
            }

            UnsupportedHorizontalOption bestPair = FindBestSameDirectionSupportedChain(
                ordered,
                remaining,
                endpointIndex,
                printedEndpoints,
                tolerance);
            if (bestPair != null)
            {
                return bestPair;
            }

            return null;
        }

        private static UnsupportedHorizontalOption FindBestPrintedToPrintedChain(
            List<PathCurve> ordered,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            List<PathCurve> ready = ordered
                .Where(path => remaining.Contains(path.Id) && printedEndpoints.SupportsBothEndpoints(path.Line))
                .ToList();
            if (ready.Count == 0)
            {
                return null;
            }

            var readyIds = new HashSet<Guid>(ready.Select(path => path.Id));
            UnsupportedHorizontalOption best = null;

            foreach (PathCurve seed in GetReadyHorizontalSeedCandidates(ready, readyIds, endpointIndex, tolerance))
            {
                UnsupportedHorizontalOption forward = BuildReadyPrintedChain(
                    seed,
                    false,
                    readyIds,
                    endpointIndex,
                    tolerance);
                if (IsBetterUnsupportedOption(forward, best))
                {
                    best = forward;
                }

                UnsupportedHorizontalOption reverse = BuildReadyPrintedChain(
                    seed,
                    true,
                    readyIds,
                    endpointIndex,
                    tolerance);
                if (IsBetterUnsupportedOption(reverse, best))
                {
                    best = reverse;
                }
            }

            return best ?? FindBestPrintedToPrintedSingle(ordered, remaining, printedEndpoints);
        }

        private static IEnumerable<PathCurve> GetReadyHorizontalSeedCandidates(
            List<PathCurve> ready,
            HashSet<Guid> readyIds,
            EndpointPathIndex endpointIndex,
            double tolerance)
        {
            const int maxSeedCandidates = 128;
            List<PathCurve> boundarySeeds = ready
                .Where(path =>
                    CountReadyConnected(path.StartPoint, readyIds, null, endpointIndex) <= 1 ||
                    CountReadyConnected(path.EndPoint, readyIds, null, endpointIndex) <= 1)
                .OrderByDescending(path => path.Line.Length)
                .ThenBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .Take(maxSeedCandidates)
                .ToList();

            if (boundarySeeds.Count > 0)
            {
                return boundarySeeds;
            }

            return ready
                .OrderByDescending(path => path.Line.Length)
                .ThenBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .Take(maxSeedCandidates);
        }

        private static UnsupportedHorizontalOption BuildReadyPrintedChain(
            PathCurve seed,
            bool reverseSeed,
            HashSet<Guid> readyIds,
            EndpointPathIndex endpointIndex,
            double tolerance)
        {
            var used = new HashSet<Guid> { seed.Id };
            var ids = new List<Guid> { seed.Id };
            var lines = new List<Line>
            {
                reverseSeed ? new Line(seed.EndPoint, seed.StartPoint) : seed.Line
            };

            while (true)
            {
                HorizontalExtensionCandidate tailCandidate = FindBestReadyExtension(
                    lines[lines.Count - 1].To,
                    lines[lines.Count - 1],
                    readyIds,
                    used,
                    endpointIndex,
                    tolerance,
                    false);
                HorizontalExtensionCandidate headCandidate = FindBestReadyExtension(
                    lines[0].From,
                    lines[0],
                    readyIds,
                    used,
                    endpointIndex,
                    tolerance,
                    true);

                bool useHead =
                    headCandidate != null &&
                    (tailCandidate == null || headCandidate.Score > tailCandidate.Score);
                HorizontalExtensionCandidate best = useHead ? headCandidate : tailCandidate;
                if (best == null || !best.Line.IsValid)
                {
                    break;
                }

                if (useHead)
                {
                    lines.Insert(0, best.Line);
                    ids.Insert(0, best.Path.Id);
                }
                else
                {
                    lines.Add(best.Line);
                    ids.Add(best.Path.Id);
                }

                used.Add(best.Path.Id);
            }

            List<Line> outputLines = TryMergeSameDirectionChain(lines, tolerance, out Line mergedLine)
                ? new List<Line> { mergedLine }
                : lines;

            return new UnsupportedHorizontalOption
            {
                Lines = outputLines,
                Ids = ids,
                Score = 3500000 + ids.Count * 10000 + (int)Math.Min(9999.0, lines.Sum(line => line.Length)),
                EndsAtPrinted = true
            };
        }

        private static HorizontalExtensionCandidate FindBestReadyExtension(
            Point3d connectionPoint,
            Line referenceLine,
            HashSet<Guid> readyIds,
            HashSet<Guid> used,
            EndpointPathIndex endpointIndex,
            double tolerance,
            bool prepend)
        {
            HorizontalExtensionCandidate best = null;

            foreach (PathCurve candidate in endpointIndex.FindConnected(connectionPoint))
            {
                if (!readyIds.Contains(candidate.Id) || used.Contains(candidate.Id))
                {
                    continue;
                }

                Line candidateLine;
                if (prepend)
                {
                    if (!TryOrientLineToPoint(candidate, connectionPoint, tolerance, out candidateLine))
                    {
                        continue;
                    }
                }
                else if (!TryOrientLineFromPoint(candidate, connectionPoint, tolerance, out candidateLine))
                {
                    continue;
                }

                Point3d nextPoint = prepend ? candidateLine.From : candidateLine.To;
                int unusedDegree = CountReadyConnected(nextPoint, readyIds, used, endpointIndex);
                bool sameDirection = prepend
                    ? AreSameForwardHorizontalDirection(candidateLine, referenceLine)
                    : AreSameForwardHorizontalDirection(referenceLine, candidateLine);
                int score =
                    unusedDegree * 10000 +
                    (sameDirection ? 1000 : 0) +
                    (int)Math.Min(999.0, candidateLine.Length);

                if (best == null || score > best.Score)
                {
                    best = new HorizontalExtensionCandidate
                    {
                        Path = candidate,
                        Line = candidateLine,
                        Score = score
                    };
                }
            }

            return best;
        }

        private static int CountReadyConnected(
            Point3d point,
            HashSet<Guid> readyIds,
            HashSet<Guid> used,
            EndpointPathIndex endpointIndex)
        {
            return endpointIndex.FindConnected(point)
                .Count(path => readyIds.Contains(path.Id) && (used == null || !used.Contains(path.Id)));
        }

        private static UnsupportedHorizontalOption FindBestPrintedToPrintedSingle(
            List<PathCurve> ordered,
            HashSet<Guid> remaining,
            PrintedEndpointIndex printedEndpoints)
        {
            return ordered
                .Where(path => remaining.Contains(path.Id))
                .Select(path =>
                {
                    if (!printedEndpoints.TryOrientWithBothEndpointsSupported(path, out Line line))
                    {
                        return null;
                    }

                    return new UnsupportedHorizontalOption
                    {
                        Lines = new List<Line> { line },
                        Ids = new List<Guid> { path.Id },
                        Score = 3000000,
                        EndsAtPrinted = true
                    };
                })
                .Where(option => option != null)
                .OrderByDescending(option => option.Length)
                .FirstOrDefault();
        }

        private static UnsupportedHorizontalOption FindBestSameDirectionSupportedChain(
            List<PathCurve> ordered,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            UnsupportedHorizontalOption best = null;

            foreach (PathCurve first in ordered.Where(path => remaining.Contains(path.Id)))
            {
                if (!printedEndpoints.TryOrientWithSupportedStart(first, out Line firstLine))
                {
                    continue;
                }

                UnsupportedHorizontalOption option = BuildSameDirectionSupportedChain(
                    first,
                    firstLine,
                    remaining,
                    endpointIndex,
                    printedEndpoints,
                    tolerance);

                if (IsBetterUnsupportedOption(option, best))
                {
                    best = option;
                }
            }

            return best;
        }

        private static UnsupportedHorizontalOption BuildSameDirectionSupportedChain(
            PathCurve seed,
            Line seedLine,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            if (!printedEndpoints.Contains(seedLine.From))
            {
                return null;
            }

            var used = new HashSet<Guid> { seed.Id };
            var lines = new List<Line> { seedLine };
            var ids = new List<Guid> { seed.Id };
            Point3d tail = seedLine.To;
            const int maxChainLength = 32;

            while (!printedEndpoints.Contains(tail) && lines.Count < maxChainLength)
            {
                PathCurve best = null;
                Line bestLine = Line.Unset;
                int bestScore = int.MinValue;

                foreach (PathCurve candidate in endpointIndex.FindConnected(tail))
                {
                    if (!remaining.Contains(candidate.Id) || used.Contains(candidate.Id))
                    {
                        continue;
                    }

                    if (!TryOrientLineFromPoint(candidate, tail, tolerance, out Line candidateLine))
                    {
                        continue;
                    }

                    if (!AreSameForwardHorizontalDirection(seedLine, candidateLine))
                    {
                        continue;
                    }

                    int score =
                        (printedEndpoints.Contains(candidateLine.To) ? 100000 : 0) +
                        (CanMergeSameDirectionChain(lines.Concat(new[] { candidateLine }).ToList(), tolerance) ? 10000 : 0) +
                        (int)Math.Min(999.0, candidateLine.Length);

                    if (score > bestScore)
                    {
                        best = candidate;
                        bestLine = candidateLine;
                        bestScore = score;
                    }
                }

                if (best == null || !bestLine.IsValid)
                {
                    break;
                }

                lines.Add(bestLine);
                ids.Add(best.Id);
                used.Add(best.Id);
                tail = bestLine.To;
            }

            if (ids.Count < 2 || !printedEndpoints.Contains(lines[lines.Count - 1].To))
            {
                return null;
            }

            List<Line> outputLines = TryMergeSameDirectionChain(lines, tolerance, out Line mergedLine)
                ? new List<Line> { mergedLine }
                : lines;

            return new UnsupportedHorizontalOption
            {
                Lines = outputLines,
                Ids = ids,
                Score = 2500000 + ids.Count * 1000 + (int)Math.Min(999.0, lines.Sum(line => line.Length)),
                EndsAtPrinted = true
            };
        }

        private static UnsupportedHorizontalOption FindBestPrintedStartChain(
            List<PathCurve> ordered,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            UnsupportedHorizontalOption best = null;

            foreach (PathCurve seed in ordered.Where(path => remaining.Contains(path.Id)))
            {
                foreach (Line seedLine in GetPrintedStartOrientations(seed, printedEndpoints))
                {
                    UnsupportedHorizontalOption option = BuildPrintedStartUnsupportedChain(
                        seed,
                        seedLine,
                        remaining,
                        endpointIndex,
                        printedEndpoints,
                        tolerance);

                    if (IsBetterUnsupportedOption(option, best))
                    {
                        best = option;
                    }
                }
            }

            return best;
        }

        private static UnsupportedHorizontalOption BuildPrintedStartUnsupportedChain(
            PathCurve seed,
            Line seedLine,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            PrintedEndpointIndex printedEndpoints,
            double tolerance)
        {
            var used = new HashSet<Guid> { seed.Id };
            var option = new UnsupportedHorizontalOption
            {
                Lines = new List<Line> { seedLine },
                Ids = new List<Guid> { seed.Id },
                Score = 1000000
            };

            Point3d tail = seedLine.To;
            Line previousLine = seedLine;

            while (!printedEndpoints.Contains(tail))
            {
                PathCurve best = null;
                Line bestLine = Line.Unset;
                int bestScore = int.MinValue;

                foreach (PathCurve candidate in endpointIndex.FindConnected(tail))
                {
                    if (!remaining.Contains(candidate.Id) || used.Contains(candidate.Id))
                    {
                        continue;
                    }

                    if (!TryOrientLineFromPoint(candidate, tail, tolerance, out Line candidateLine))
                    {
                        continue;
                    }

                    int score =
                        (printedEndpoints.Contains(candidateLine.To) ? 10000 : 0) +
                        (AreSameHorizontalDirection(previousLine, candidateLine) ? 1000 : 0) +
                        (int)Math.Min(999.0, candidateLine.Length);

                    if (score > bestScore)
                    {
                        best = candidate;
                        bestLine = candidateLine;
                        bestScore = score;
                    }
                }

                if (best == null || !bestLine.IsValid)
                {
                    break;
                }

                option.Lines.Add(bestLine);
                option.Ids.Add(best.Id);
                used.Add(best.Id);
                previousLine = bestLine;
                tail = bestLine.To;
            }

            option.EndsAtPrinted = printedEndpoints.Contains(option.Lines[option.Lines.Count - 1].To);
            if (option.EndsAtPrinted)
            {
                option.Score += 100000;
            }

            option.Score += option.Lines.Count * 100;
            return option;
        }

        private static bool IsBetterUnsupportedOption(UnsupportedHorizontalOption candidate, UnsupportedHorizontalOption best)
        {
            if (candidate == null || candidate.Lines.Count == 0)
            {
                return false;
            }

            if (best == null || best.Lines.Count == 0)
            {
                return true;
            }

            if (candidate.EndsAtPrinted != best.EndsAtPrinted)
            {
                return candidate.EndsAtPrinted;
            }

            if (candidate.Score != best.Score)
            {
                return candidate.Score > best.Score;
            }

            if (candidate.Lines.Count != best.Lines.Count)
            {
                return candidate.Lines.Count > best.Lines.Count;
            }

            return candidate.Length > best.Length;
        }

        private static int PrintedEndpointCount(PathCurve path, PrintedEndpointIndex printedEndpoints)
        {
            return (printedEndpoints.Contains(path.StartPoint) ? 1 : 0) +
                   (printedEndpoints.Contains(path.EndPoint) ? 1 : 0);
        }

        private static IEnumerable<Line> GetPrintedStartOrientations(PathCurve path, PrintedEndpointIndex printedEndpoints)
        {
            bool startPrinted = printedEndpoints.Contains(path.StartPoint);
            bool endPrinted = printedEndpoints.Contains(path.EndPoint);

            if (startPrinted)
            {
                yield return path.Line;
            }

            if (endPrinted)
            {
                yield return new Line(path.EndPoint, path.StartPoint);
            }
        }

        private static Line OrientLineFromPrintedEndpoint(PathCurve path, PrintedEndpointIndex printedEndpoints)
        {
            bool startPrinted = printedEndpoints.Contains(path.StartPoint);
            bool endPrinted = printedEndpoints.Contains(path.EndPoint);

            if (!startPrinted && endPrinted)
            {
                return new Line(path.EndPoint, path.StartPoint);
            }

            return path.Line;
        }

        private static bool TryOrientLineFromPoint(PathCurve path, Point3d startPoint, double tolerance, out Line line)
        {
            line = Line.Unset;
            if (path.StartPoint.DistanceTo(startPoint) <= tolerance)
            {
                line = path.Line;
                return line.IsValid;
            }

            if (path.EndPoint.DistanceTo(startPoint) <= tolerance)
            {
                line = new Line(path.EndPoint, path.StartPoint);
                return line.IsValid;
            }

            return false;
        }

        private static bool TryOrientLineToPoint(PathCurve path, Point3d endPoint, double tolerance, out Line line)
        {
            line = Line.Unset;
            if (path.EndPoint.DistanceTo(endPoint) <= tolerance)
            {
                line = path.Line;
                return line.IsValid;
            }

            if (path.StartPoint.DistanceTo(endPoint) <= tolerance)
            {
                line = new Line(path.EndPoint, path.StartPoint);
                return line.IsValid;
            }

            return false;
        }

        private static bool AreSameHorizontalDirection(Line first, Line second)
        {
            Vector3d firstDirection = first.To - first.From;
            Vector3d secondDirection = second.To - second.From;
            firstDirection.Z = 0.0;
            secondDirection.Z = 0.0;

            if (!firstDirection.Unitize() || !secondDirection.Unitize())
            {
                return false;
            }

            return Math.Abs(firstDirection * secondDirection) >= Math.Cos(10.0 * Math.PI / 180.0);
        }

        private static bool AreSameForwardHorizontalDirection(Line first, Line second)
        {
            Vector3d firstDirection = first.To - first.From;
            Vector3d secondDirection = second.To - second.From;
            firstDirection.Z = 0.0;
            secondDirection.Z = 0.0;

            if (!firstDirection.Unitize() || !secondDirection.Unitize())
            {
                return false;
            }

            return firstDirection * secondDirection >= Math.Cos(10.0 * Math.PI / 180.0);
        }

        private static bool CanMergeSameDirectionChain(List<Line> lines, double tolerance)
        {
            return TryMergeSameDirectionChain(lines, tolerance, out _);
        }

        private static bool TryMergeSameDirectionChain(List<Line> lines, double tolerance, out Line mergedLine)
        {
            mergedLine = Line.Unset;
            if (lines == null || lines.Count == 0)
            {
                return false;
            }

            if (lines.Count == 1)
            {
                mergedLine = lines[0];
                return mergedLine.IsValid;
            }

            for (int i = 1; i < lines.Count; i++)
            {
                if (lines[i - 1].To.DistanceTo(lines[i].From) > tolerance ||
                    !AreSameForwardHorizontalDirection(lines[0], lines[i]))
                {
                    return false;
                }
            }

            Line candidate = new Line(lines[0].From, lines[lines.Count - 1].To);
            if (!candidate.IsValid || candidate.Length <= RhinoMath.ZeroTolerance)
            {
                return false;
            }

            foreach (Line line in lines)
            {
                Point3d from = candidate.ClosestPoint(line.From, true);
                Point3d to = candidate.ClosestPoint(line.To, true);
                if (from.DistanceTo(line.From) > tolerance || to.DistanceTo(line.To) > tolerance)
                {
                    return false;
                }
            }

            mergedLine = candidate;
            return true;
        }

        private static List<List<PathCurve>> BuildHorizontalComponents(List<PathCurve> horizontals, double tolerance)
        {
            var components = new List<List<PathCurve>>();
            var remaining = new HashSet<Guid>(horizontals.Select(path => path.Id));
            var endpointIndex = new EndpointPathIndex(horizontals, tolerance);

            foreach (PathCurve seed in horizontals)
            {
                if (!remaining.Contains(seed.Id))
                {
                    continue;
                }

                var component = new List<PathCurve>();
                var queue = new Queue<PathCurve>();
                queue.Enqueue(seed);
                remaining.Remove(seed.Id);

                while (queue.Count > 0)
                {
                    PathCurve current = queue.Dequeue();
                    component.Add(current);

                    foreach (PathCurve candidate in endpointIndex.FindConnected(current.StartPoint)
                        .Concat(endpointIndex.FindConnected(current.EndPoint)))
                    {
                        if (remaining.Remove(candidate.Id))
                        {
                            queue.Enqueue(candidate);
                        }
                    }
                }

                components.Add(component);
            }

            return components;
        }

        private static List<List<Line>> BuildLongestLineChains(
            List<PathCurve> component,
            VerticalSupportIndex verticalSupports,
            double firstHorizontalZ,
            double tolerance)
        {
            var chains = new List<List<Line>>();
            var remaining = new HashSet<Guid>(component.Select(path => path.Id));
            var endpointIndex = new EndpointPathIndex(component, tolerance);

            while (remaining.Count > 0)
            {
                PathCurve seed = SelectHorizontalSeed(component, remaining, endpointIndex, verticalSupports, firstHorizontalZ, tolerance);
                bool reverseSeed = ShouldReverseHorizontalSeed(seed, endpointIndex, verticalSupports, firstHorizontalZ, tolerance);
                List<Guid> bestIds;
                List<Line> bestChain = BuildGreedyHorizontalChain(
                    endpointIndex,
                    remaining,
                    seed,
                    reverseSeed,
                    verticalSupports,
                    firstHorizontalZ,
                    tolerance,
                    out bestIds);
                bestChain = OrientHorizontalChainForSupportedStart(bestChain, verticalSupports, firstHorizontalZ, tolerance);

                if (bestChain.Count == 0)
                {
                    Guid fallbackId = remaining.First();
                    PathCurve fallback = component.First(path => path.Id == fallbackId);
                    bestChain.Add(fallback.Line);
                    bestIds.Add(fallbackId);
                }

                foreach (Guid id in bestIds)
                {
                    remaining.Remove(id);
                }

                chains.Add(bestChain);
            }

            return chains;
        }

        private static PathCurve SelectHorizontalSeed(
            List<PathCurve> component,
            HashSet<Guid> remaining,
            EndpointPathIndex endpointIndex,
            VerticalSupportIndex verticalSupports,
            double firstHorizontalZ,
            double tolerance)
        {
            return component
                .Where(path => remaining.Contains(path.Id))
                .OrderBy(path => IsBoundaryHorizontal(path, endpointIndex) ? 0 : 1)
                .ThenByDescending(path => EndpointSupportCount(path, verticalSupports, firstHorizontalZ, tolerance))
                .ThenByDescending(path => path.Line.Length)
                .ThenBy(path => Math.Min(path.StartPoint.Z, path.EndPoint.Z))
                .ThenBy(path => path.MidPoint.X)
                .ThenBy(path => path.MidPoint.Y)
                .First();
        }

        private static bool IsBoundaryHorizontal(PathCurve path, EndpointPathIndex endpointIndex)
        {
            return endpointIndex.CountConnected(path.StartPoint) <= 1 ||
                   endpointIndex.CountConnected(path.EndPoint) <= 1;
        }

        private static int EndpointSupportCount(
            PathCurve path,
            VerticalSupportIndex verticalSupports,
            double firstHorizontalZ,
            double tolerance)
        {
            if (Math.Min(path.StartPoint.Z, path.EndPoint.Z) <= firstHorizontalZ + tolerance)
            {
                return 2;
            }

            return (IsPointSupportedByVertical(path.StartPoint, verticalSupports) ? 1 : 0) +
                   (IsPointSupportedByVertical(path.EndPoint, verticalSupports) ? 1 : 0);
        }

        private static bool ShouldReverseHorizontalSeed(
            PathCurve seed,
            EndpointPathIndex endpointIndex,
            VerticalSupportIndex verticalSupports,
            double firstHorizontalZ,
            double tolerance)
        {
            bool startSupported = Math.Min(seed.StartPoint.Z, seed.EndPoint.Z) <= firstHorizontalZ + tolerance ||
                                  IsPointSupportedByVertical(seed.StartPoint, verticalSupports);
            bool endSupported = Math.Min(seed.StartPoint.Z, seed.EndPoint.Z) <= firstHorizontalZ + tolerance ||
                                IsPointSupportedByVertical(seed.EndPoint, verticalSupports);

            if (startSupported != endSupported)
            {
                return endSupported;
            }

            int startDegree = endpointIndex.CountConnected(seed.StartPoint);
            int endDegree = endpointIndex.CountConnected(seed.EndPoint);
            return endDegree < startDegree;
        }

        private static List<Line> BuildGreedyHorizontalChain(
            EndpointPathIndex endpointIndex,
            HashSet<Guid> allowed,
            PathCurve seed,
            bool reverseSeed,
            VerticalSupportIndex verticalSupports,
            double firstHorizontalZ,
            double tolerance,
            out List<Guid> usedIds)
        {
            usedIds = new List<Guid> { seed.Id };
            var used = new HashSet<Guid> { seed.Id };
            var chain = new List<Line>();
            Line seedLine = reverseSeed ? new Line(seed.EndPoint, seed.StartPoint) : seed.Line;
            chain.Add(seedLine);
            Point3d tail = seedLine.To;

            bool extended;
            do
            {
                extended = false;
                PathCurve best = null;
                Line bestLine = Line.Unset;
                int bestSupportScore = int.MinValue;
                double bestLength = 0.0;

                foreach (PathCurve candidate in endpointIndex.FindConnected(tail))
                {
                    if (!allowed.Contains(candidate.Id) || used.Contains(candidate.Id))
                    {
                        continue;
                    }

                    Line candidateLine = Line.Unset;
                    if (candidate.StartPoint.DistanceTo(tail) <= tolerance)
                    {
                        candidateLine = candidate.Line;
                    }
                    else if (candidate.EndPoint.DistanceTo(tail) <= tolerance)
                    {
                        candidateLine = new Line(candidate.EndPoint, candidate.StartPoint);
                    }

                    if (!candidateLine.IsValid)
                    {
                        continue;
                    }

                    int supportScore = GetHorizontalSupportMetrics(
                        new List<Line> { candidateLine },
                        verticalSupports,
                        firstHorizontalZ,
                        tolerance).PriorityScore;
                    int nextDegree = endpointIndex.CountConnected(candidateLine.To);

                    bool better =
                        best == null ||
                        supportScore > bestSupportScore ||
                        (supportScore == bestSupportScore && nextDegree > endpointIndex.CountConnected(bestLine.To)) ||
                        (supportScore == bestSupportScore && nextDegree == endpointIndex.CountConnected(bestLine.To) && candidateLine.Length > bestLength + tolerance);

                    if (better)
                    {
                        best = candidate;
                        bestLine = candidateLine;
                        bestSupportScore = supportScore;
                        bestLength = candidateLine.Length;
                    }
                }

                if (best != null && bestLine.IsValid)
                {
                    chain.Add(bestLine);
                    used.Add(best.Id);
                    usedIds.Add(best.Id);
                    tail = bestLine.To;
                    extended = true;
                }
            } while (extended);

            return chain;
        }

        private static bool LinesShareEndpoint(Line first, Line second, double tolerance)
        {
            return first.From.DistanceTo(second.From) <= tolerance ||
                   first.From.DistanceTo(second.To) <= tolerance ||
                   first.To.DistanceTo(second.From) <= tolerance ||
                   first.To.DistanceTo(second.To) <= tolerance;
        }

        private static DataTree<Curve> LinesToTree(List<List<Line>> paths)
        {
            var tree = new DataTree<Curve>();
            for (int i = 0; i < paths.Count; i++)
            {
                GH_Path path = new GH_Path(i);
                foreach (Line line in paths[i])
                {
                    tree.Add(new LineCurve(line), path);
                }
            }

            return tree;
        }

        private static DataTree<Curve> PairCandidatesToTree(List<VerticalAngledPairCandidate> pairs)
        {
            var tree = new DataTree<Curve>();
            for (int i = 0; i < pairs.Count; i++)
            {
                GH_Path path = new GH_Path(i);
                tree.Add(new LineCurve(pairs[i].VerticalLine), path);
                if (pairs[i].HasAngled)
                {
                    tree.Add(new LineCurve(pairs[i].AngledLine), path);
                }
            }

            return tree;
        }

        private static DataTree<Curve> VerticalAngledChainsToTree(List<VerticalAngledChainCandidate> chains)
        {
            var tree = new DataTree<Curve>();
            for (int i = 0; i < chains.Count; i++)
            {
                GH_Path path = new GH_Path(i);
                foreach (VerticalAngledPairCandidate pair in chains[i].Pairs)
                {
                    tree.Add(new LineCurve(pair.VerticalLine), path);
                    if (pair.HasAngled)
                    {
                        tree.Add(new LineCurve(pair.AngledLine), path);
                    }
                }

                foreach (Line standaloneVertical in chains[i].StandaloneVerticalLines)
                {
                    tree.Add(new LineCurve(standaloneVertical), path);
                }
            }

            return tree;
        }

        private static void AddSequenceOutputs(
            List<PrintSequenceItem> sequence,
            out DataTree<Curve> horizontalTree,
            out DataTree<Curve> verticalTree,
            out DataTree<Curve> angledTree,
            out List<string> pattern,
            out List<Curve> previewCurves)
        {
            horizontalTree = new DataTree<Curve>();
            verticalTree = new DataTree<Curve>();
            angledTree = new DataTree<Curve>();
            pattern = new List<string>();
            previewCurves = new List<Curve>();

            int hBranch = 0;
            int vaBranch = 0;

            foreach (PrintSequenceItem item in sequence)
            {
                if (item.Kind == "H")
                {
                    GH_Path path = new GH_Path(hBranch++);
                    foreach (Line line in item.Lines)
                    {
                        horizontalTree.Add(new LineCurve(line), path);
                        previewCurves.Add(new LineCurve(line));
                        pattern.Add("H");
                    }
                }
                else
                {
                    GH_Path path = new GH_Path(vaBranch++);
                    foreach (VerticalAngledPairCandidate pair in item.Pairs)
                    {
                        verticalTree.Add(new LineCurve(pair.VerticalLine), path);
                        previewCurves.Add(new LineCurve(pair.VerticalLine));
                        pattern.Add("V");

                        if (pair.HasAngled)
                        {
                            angledTree.Add(new LineCurve(pair.AngledLine), path);
                            previewCurves.Add(new LineCurve(pair.AngledLine));
                            pattern.Add("A");
                        }
                    }

                    foreach (Line standaloneVertical in item.Lines)
                    {
                        verticalTree.Add(new LineCurve(standaloneVertical), path);
                        previewCurves.Add(new LineCurve(standaloneVertical));
                        pattern.Add("V");
                    }
                }
            }
        }

        private static string BuildOptimizationReport(
            List<List<Guid>> orderedClusters,
            List<List<Guid>> heightGroups,
            List<List<Line>> horizontalPaths,
            int supportedHorizontalPaths,
            int unsupportedHorizontalPaths,
            List<VerticalAngledPairCandidate> pairCandidates,
            List<VerticalAngledChainCandidate> selectedChains,
            int appendedStandaloneVerticals,
            int culledAngledDownCollisions,
            int unscheduledUnsupportedHorizontals,
            List<string> pattern,
            OptimizationWeights weights)
        {
            var directionCounts = selectedChains
                .SelectMany(chain => chain.Pairs)
                .Where(pair => pair.HasAngled)
                .GroupBy(pair => pair.Direction)
                .ToDictionary(group => group.Key, group => group.Count());

            string directions = string.Join(", ", Enum.GetValues(typeof(AngledDirection))
                .Cast<AngledDirection>()
                .Select(direction => $"{direction}: {(directionCounts.ContainsKey(direction) ? directionCounts[direction] : 0)}"));

            return string.Join(Environment.NewLine, new[]
            {
                $"Ordered clusters: {orderedClusters.Count}",
                $"Same-height vertical/angled search groups: {heightGroups.Count}",
                $"Horizontal print paths: {horizontalPaths.Count}",
                $"Supported/base horizontal paths: {supportedHorizontalPaths}",
                $"Unsupported horizontal paths: {unsupportedHorizontalPaths}",
                $"Vertical/angled pair candidates: {pairCandidates.Count}",
                $"Selected vertical/angled paths: {selectedChains.Count}",
                $"Selected vertical/angled pairs: {selectedChains.Sum(chain => chain.Pairs.Count)}",
                $"Selected angled members: {selectedChains.SelectMany(chain => chain.Pairs).Count(pair => pair.HasAngled)}",
                $"Standalone verticals appended: {appendedStandaloneVerticals}",
                $"Culled angled-down vertical collisions: {culledAngledDownCollisions}",
                $"Unsupported horizontals held out: {unscheduledUnsupportedHorizontals}",
                $"Pattern tokens: {pattern.Count}",
                $"Direction balance: {directions}",
                $"Weights: use most curves={weights.UseMostCurves:F2}, equal direction={weights.EqualDirection:F2}, longest paths={weights.LongestPaths:F2}"
            });
        }



        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> pathCurves = new List<Line>();
            List<string> orientation = new List<string>();
            List<string> connectivity = new List<string>();
            double useMostCurvesWeight = 1.0;
            double equalDirectionWeight = 0.5;
            double longestPathsWeight = 1.0;
            double sameHeightTolerance = 0.01;
            double connectionTolerance = 0.01;

            if (!DA.GetDataList(0, pathCurves)) { return; }
            DA.GetData(1, ref useMostCurvesWeight);
            DA.GetData(2, ref equalDirectionWeight);
            DA.GetData(3, ref longestPathsWeight);
            DA.GetData(4, ref sameHeightTolerance);
            DA.GetData(5, ref connectionTolerance);

            List<Line> inputLines = pathCurves
                .Where(line => line.IsValid && line.Length > RhinoMath.ZeroTolerance)
                .ToList();

            if (inputLines.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No valid lines were supplied.");
                return;
            }

            var weights = new OptimizationWeights
            {
                UseMostCurves = Math.Max(0.0, useMostCurvesWeight),
                EqualDirection = Math.Max(0.0, equalDirectionWeight),
                LongestPaths = Math.Max(0.0, longestPathsWeight),
                SameHeightTolerance = Math.Max(1e-6, sameHeightTolerance),
                ConnectionTolerance = Math.Max(1e-6, connectionTolerance)
            };

            var spatialPaths = new SpatialPaths(inputLines, weights.ConnectionTolerance);

            var stats = spatialPaths.GetOrientationStats();
            foreach (var kvp in stats)
                orientation.Add($"{kvp.Key}: {kvp.Value}");
                

            foreach (var path in spatialPaths.Paths.Values)
            {
                connectivity.Add($"Curve {path.Id} has {path.StartConnections.Count} connections at start, " +
                                         $"{path.EndConnections.Count} at end. Orientation: {path.Orientation}");
            }

            List<List<Guid>> orderedClusters = OrderClustersByHeight(
                spatialPaths,
                spatialPaths.FindZLayeredClusters(weights.SameHeightTolerance),
                weights.ConnectionTolerance);

            DataTree<Curve> clusterTree = ConvertToTree(
                orderedClusters
                    .Select(cluster => cluster
                        .Where(id => spatialPaths.Paths.ContainsKey(id))
                        .Select(id => spatialPaths.Paths[id].Line)
                        .ToList())
                    .ToList());

            List<List<Guid>> heightGroups = CombineClustersAtSameHeight(
                spatialPaths,
                orderedClusters,
                weights.SameHeightTolerance);

            List<VerticalAngledPairCandidate> allPairCandidates = new List<VerticalAngledPairCandidate>();
            List<VerticalAngledChainCandidate> selectedVerticalAngledChains = new List<VerticalAngledChainCandidate>();
            int appendedStandaloneVerticals = 0;

            foreach (List<Guid> heightGroup in heightGroups)
            {
                List<VerticalAngledPairCandidate> candidates = BuildVerticalAngledPairCandidates(
                    spatialPaths,
                    heightGroup,
                    weights.ConnectionTolerance);
                List<VerticalAngledChainCandidate> selectedForGroup = SelectOptimizedVerticalAngledChains(candidates, weights);

                allPairCandidates.AddRange(candidates);
                appendedStandaloneVerticals += AppendUnusedVerticalsToEndOfGroup(
                    spatialPaths,
                    heightGroup,
                    selectedForGroup,
                    weights.ConnectionTolerance);
                selectedVerticalAngledChains.AddRange(selectedForGroup);
            }

            int culledAngledDownCollisions = CullAngledDownIntersectionsWithPrintedVerticals(
                selectedVerticalAngledChains,
                weights.ConnectionTolerance);
            if (culledAngledDownCollisions > 0)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    $"{culledAngledDownCollisions} angled-down member(s) were removed because they intersected verticals that were already printed.");
            }

            DataTree<Curve> pairsTree = PairCandidatesToTree(allPairCandidates);
            DataTree<Curve> connectedVerticalAngledTree = VerticalAngledChainsToTree(selectedVerticalAngledChains);

            List<VerticalSupportEndpoint> verticalSupportEndpoints = BuildVerticalSupportEndpoints(
                selectedVerticalAngledChains,
                weights.ConnectionTolerance);
            var verticalSupports = new VerticalSupportIndex(
                verticalSupportEndpoints,
                weights.ConnectionTolerance);
            List<Line> verticalAngledPrintedSeedLines = selectedVerticalAngledChains
                .SelectMany(chain => chain.Pairs
                    .SelectMany(pair => pair.HasAngled
                        ? new[] { pair.VerticalLine, pair.AngledLine }
                        : new[] { pair.VerticalLine })
                    .Concat(chain.StandaloneVerticalLines))
                .ToList();
            double firstHorizontalZ = GetFirstHorizontalLayerZ(spatialPaths.Paths.Values
                .Where(path => path.Orientation == PathCurve.OrientationType.Horizontal));
            List<List<Line>> horizontalPaths = BuildHorizontalLongestPaths(
                spatialPaths,
                verticalSupports,
                verticalAngledPrintedSeedLines,
                weights.ConnectionTolerance,
                out int unscheduledUnsupportedHorizontals);
            int supportedHorizontalPaths = horizontalPaths.Count(path =>
                GetHorizontalSupportGroup(path, verticalSupports, firstHorizontalZ, weights.ConnectionTolerance) == 0);
            int unsupportedHorizontalPaths = horizontalPaths.Count - supportedHorizontalPaths;
            if (unscheduledUnsupportedHorizontals > 0)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    $"{unscheduledUnsupportedHorizontals} unsupported horizontal member(s) were held out because no endpoint connected to printed geometry.");
            }

            var sequence = new List<PrintSequenceItem>();
            int horizontalSequenceOrder = 0;

            foreach (List<Line> horizontalPath in horizontalPaths)
            {
                HorizontalSupportMetrics support = GetHorizontalSupportMetrics(
                    horizontalPath,
                    verticalSupports,
                    firstHorizontalZ,
                    weights.ConnectionTolerance);
                int supportGroup = GetHorizontalSupportGroup(
                    horizontalPath,
                    verticalSupports,
                    firstHorizontalZ,
                    weights.ConnectionTolerance);

                sequence.Add(new PrintSequenceItem
                {
                    Kind = "H",
                    Lines = horizontalPath,
                    MinZ = horizontalPath.Min(line => Math.Min(line.From.Z, line.To.Z)),
                    Center = AveragePoint(horizontalPath.Select(line => line.PointAt(0.5))),
                    SupportGroup = supportGroup,
                    SupportPriority = support.PriorityScore,
                    SequenceOrder = horizontalSequenceOrder++
                });
            }

            foreach (VerticalAngledChainCandidate chain in selectedVerticalAngledChains)
            {
                sequence.Add(new PrintSequenceItem
                {
                    Kind = "VA",
                    Pairs = chain.Pairs,
                    Lines = chain.StandaloneVerticalLines,
                    MinZ = chain.MinZ,
                    Center = chain.Center
                });
            }

            sequence = sequence
                .OrderBy(item => item.MinZ)
                .ThenBy(item => item.Kind == "H" ? 0 : 1)
                .ThenBy(item => item.Kind == "H" ? item.SequenceOrder : 0)
                .ThenByDescending(item => item.Kind == "H" ? item.SupportPriority : 0)
                .ThenBy(item => item.Center.X)
                .ThenBy(item => item.Center.Y)
                .ToList();

            AddSequenceOutputs(
                sequence,
                out DataTree<Curve> horizontalTree,
                out DataTree<Curve> verticalTree,
                out DataTree<Curve> angledTree,
                out List<string> pattern,
                out List<Curve> previewCurves);

            string report = BuildOptimizationReport(
                orderedClusters,
                heightGroups,
                horizontalPaths,
                supportedHorizontalPaths,
                unsupportedHorizontalPaths,
                allPairCandidates,
                selectedVerticalAngledChains,
                appendedStandaloneVerticals,
                culledAngledDownCollisions,
                unscheduledUnsupportedHorizontals,
                pattern,
                weights);

            DA.SetDataList(0, orientation);
            DA.SetDataList(1, connectivity);
            DA.SetDataTree(2, clusterTree);
            DA.SetDataTree(3, pairsTree);
            DA.SetDataTree(4, connectedVerticalAngledTree);
            DA.SetDataTree(5, horizontalTree);
            DA.SetDataTree(6, verticalTree);
            DA.SetDataTree(7, angledTree);
            DA.SetDataList(8, pattern);
            DA.SetDataList(9, previewCurves);
            DA.SetData(10, report);


        }



        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("ccf4fb80-8601-4507-852e-e59fc5770f01"); }
        }
    }
}
