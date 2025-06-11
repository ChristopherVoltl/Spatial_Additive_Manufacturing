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
                    tree.Add(new LineCurve(a.Line), path);
                }
            }

            return tree;
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> pathCurves = new List<Line>();
            List<string> orientation = new List<string>();
            List<string> connectivity = new List<string>();



            if (!DA.GetDataList(0, pathCurves)) { return; }

            List<Line> inputLines = pathCurves;
            var spatialPaths = new SpatialPaths(inputLines);

            var stats = spatialPaths.GetOrientationStats();
            foreach (var kvp in stats)
                //Rhino.RhinoApp.WriteLine($"{kvp.Key}: {kvp.Value}");
                orientation.Add($"{kvp.Key}: {kvp.Value}");
                

            foreach (var path in spatialPaths.Paths.Values)
            {
                connectivity.Add($"Curve {path.Id} has {path.StartConnections.Count} connections at start, " +
                                         $"{path.EndConnections.Count} at end. Orientation: {path.Orientation}");
            }

            var clusters = spatialPaths.GetCurveClusters();
            DataTree<Curve> clusterTree = ConvertToTree(clusters);
            var VAclusters = spatialPaths.FindZLayeredClusters();

            

            foreach (var cluster in VAclusters)
            {
                var pairs = spatialPaths.FindVerticalAngledPairs(cluster);
                Rhino.RhinoApp.WriteLine($"Cluster has {pairs.Count} vertical-angled pairs");
                var chains = spatialPaths.BuildLongestChains(pairs);
            }

            // Build chains from the pairs
            var allChains = new List<List<(PathCurve, PathCurve)>>();

            foreach (var cluster in VAclusters)
            {
                var pairs = spatialPaths.FindVerticalAngledPairs(cluster);
                var chains = SortConnectedPairChains(pairs); // The method we just wrote
                allChains.AddRange(chains);
            }

            var tree = ConvertPairChainsToTree(allChains);

            var verticalTree = new DataTree<Curve>();
            var angledTree = new DataTree<Curve>();
            var pairsTree = new DataTree<Curve>();

            int pairIndex = 0;

            foreach (var cluster in VAclusters)
            {
                var pairs = spatialPaths.FindVerticalAngledPairs(cluster);
                var chains = SortConnectedPairChains(pairs);

                for (int i = 0; i < chains.Count; i++)
                {
                    RhinoApp.WriteLine($"Chain {i}: {chains[i].Count} pairs");
                }


                foreach (var (vertical, angled) in pairs)
                {
                    var path = new GH_Path(pairIndex++);
                    pairsTree.Add(new LineCurve(vertical.Line), path);
                    pairsTree.Add(new LineCurve(angled.Line), path);
                }

            }



            // 3. Set the outputs
            DA.SetDataList(0, orientation);
            DA.SetDataList(1, connectivity);
            DA.SetDataTree(2, clusterTree);
            DA.SetDataTree(3, pairsTree);
            DA.SetDataTree(4, tree);


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