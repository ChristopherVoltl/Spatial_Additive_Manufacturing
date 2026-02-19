using System;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using Rhino;
using Rhino.DocObjects;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System.Linq;
using Rhino.Render.ChangeQueue;
using System.Collections;
using Rhino.Geometry.Intersect;
using Rhino.UI;
using System.Diagnostics.Eventing.Reader;
using Compas.Robot.Link;
using System.Security.Cryptography;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class ContinuousToolpathMethods : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public ContinuousToolpathMethods()
          : base("Continuous Toolpath Methods", "CTM",
              "Generates a graph from a set of lines and nodes and applies differnt graphing Algorithms to the neetwork to find continuous paths",
              "FGAM", "Toolpathing")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("pathCurves", "pC", " an array of Curves", GH_ParamAccess.list);
            pManager.AddNumberParameter("Timeout", "T", "Timeout in seconds", GH_ParamAccess.item, 5);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Longest Trail", "LT", "Graph:   Longest Trail Algorithm", GH_ParamAccess.tree);
            pManager.AddPointParameter("Graph_Nodes", "GN", "Graph: Nodes", GH_ParamAccess.tree);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public class GraphLongestTrail
        {
            public class Node
            {
                public int Id;
                public Point3d P;
                public List<int> IncidentEdges = new List<int>();
            }

            public class Edge
            {
                public int Id;
                public int A; // node id
                public int B; // node id
                public Line L;

                public double Length;
                public Vector3d Dir;    // normalized
                public double ZMin;
                public double ZMax;

                public bool IsHorizontal;
                public bool IsVertical;
                public bool IsAngled;
                public int LowNode;  // node id of lower endpoint
                public int HighNode; // node id of higher endpoint
            }

            public List<Node> Nodes { get; private set; } = new List<Node>();
            public List<Edge> Edges { get; private set; } = new List<Edge>();

            // adjacency by node id -> edge ids
            private List<List<int>> adjacency;

            private readonly double tol;
            private readonly double cosVertical;   // cos(angle threshold)
            private readonly double cosHorizontal; // cos(angle threshold)

            // Simple spatial hash for tolerance merging
            private readonly Dictionary<(int, int, int), List<int>> nodeBuckets = new();

            public GraphLongestTrail(List<Curve> curves, double tolerance = 1e-3, double verticalDeg = 10.0, double horizontalDeg = 10.0)
            {
                tol = tolerance;

                cosVertical = Math.Cos(RhinoMath.ToRadians(verticalDeg));
                cosHorizontal = Math.Cos(RhinoMath.ToRadians(horizontalDeg));

                BuildGraph(curves);

                adjacency = new List<List<int>>(Nodes.Count);
                for (int i = 0; i < Nodes.Count; i++) adjacency.Add(new List<int>());
                for (int e = 0; e < Edges.Count; e++)
                {
                    var edge = Edges[e];
                    adjacency[edge.A].Add(e);
                    adjacency[edge.B].Add(e);
                    Nodes[edge.A].IncidentEdges.Add(e);
                    Nodes[edge.B].IncidentEdges.Add(e);
                }
            }

            private void BuildGraph(List<Curve> curves)
            {
                foreach (var curve in curves)
                {
                    if (!curve.TryGetPolyline(out Polyline pl)) continue;
                    if (pl.Count != 2) continue;

                    Point3d p0 = pl[0];
                    Point3d p1 = pl[1];
                    if (p0.DistanceTo(p1) < tol) continue;

                    int a = GetOrCreateNode(p0);
                    int b = GetOrCreateNode(p1);

                    var line = new Line(Nodes[a].P, Nodes[b].P);

                    var edge = new Edge();
                    edge.Id = Edges.Count;
                    edge.A = a;
                    edge.B = b;
                    edge.L = line;
                    edge.Length = line.Length;

                    Vector3d d = line.Direction;
                    d.Unitize();
                    edge.Dir = d;

                    edge.ZMin = Math.Min(line.FromZ, line.ToZ);
                    edge.ZMax = Math.Max(line.FromZ, line.ToZ);

                    // classify
                    var zAxis = Vector3d.ZAxis;
                    double cosToZ = Math.Abs(d * zAxis); // dot product magnitude (since both unit)
                    edge.IsVertical = cosToZ >= cosVertical;
                    //edge.IsHorizontal = cosToZ <= Math.Sin(RhinoMath.ToRadians(horizontalDeg)); // near perpendicular to Z
                    edge.IsAngled = !edge.IsVertical && !edge.IsHorizontal;

                    // low/high endpoint
                    if (Nodes[a].P.Z <= Nodes[b].P.Z)
                    {
                        edge.LowNode = a;
                        edge.HighNode = b;
                    }
                    else
                    {
                        edge.LowNode = b;
                        edge.HighNode = a;
                    }

                    Edges.Add(edge);
                }
            }

            private int GetOrCreateNode(Point3d p)
            {
                var key = BucketKey(p);

                if (nodeBuckets.TryGetValue(key, out var candidates))
                {
                    foreach (int id in candidates)
                    {
                        if (Nodes[id].P.DistanceToSquared(p) <= tol * tol)
                            return id;
                    }
                }

                // new node
                int newId = Nodes.Count;
                Nodes.Add(new Node { Id = newId, P = p });

                if (!nodeBuckets.ContainsKey(key))
                    nodeBuckets[key] = new List<int>();
                nodeBuckets[key].Add(newId);

                return newId;
            }

            private (int, int, int) BucketKey(Point3d p)
            {
                // grid hashing with cell size = tol
                int ix = (int)Math.Floor(p.X / tol);
                int iy = (int)Math.Floor(p.Y / tol);
                int iz = (int)Math.Floor(p.Z / tol);
                return (ix, iy, iz);
            }


        
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            double calc_time = 5.0; // Default timeout value
            // Retrieve the input data
            if (!DA.GetDataList(0, curves)) return;
            if (!DA.GetData(1, ref calc_time)) return;

            // Create the graph
            var graph = new GraphLongestTrail(curves);

            // Set a 5-second timeout
            TimeSpan timeout = TimeSpan.FromSeconds(calc_time);

            

            // Set the output data
            //DA.SetDataList(0, longestTrail);



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
            get { return new Guid("62b75d2c-a076-4dc3-bd68-4950444aad36"); }
        }
    }
}