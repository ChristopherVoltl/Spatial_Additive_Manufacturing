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
            // Graph representation
            public List<Point3d> Vertices { get; private set; }
            public List<Line> Edges { get; private set; }
            private Dictionary<Point3d, List<Line>> adjacencyList;

            public GraphLongestTrail(List<Curve> curves)
            {
                // Initialize the graph
                Vertices = new List<Point3d>();
                Edges = new List<Line>();
                adjacencyList = new Dictionary<Point3d, List<Line>>();

                // Build the graph from input curves
                BuildGraph(curves);
            }

            private void BuildGraph(List<Curve> curves)
            {
                foreach (var curve in curves)
                {
                    if (curve.TryGetPolyline(out Polyline polyline) && polyline.Count == 2)
                    {
                        Point3d start = polyline[0];
                        Point3d end = polyline[1];
                        Line edge = new Line(start, end);

                        // Add vertices
                        if (!Vertices.Contains(start)) Vertices.Add(start);
                        if (!Vertices.Contains(end)) Vertices.Add(end);

                        // Add edge
                        Edges.Add(edge);

                        // Update adjacency list
                        if (!adjacencyList.ContainsKey(start))
                            adjacencyList[start] = new List<Line>();
                        if (!adjacencyList.ContainsKey(end))
                            adjacencyList[end] = new List<Line>();

                        adjacencyList[start].Add(edge);
                        adjacencyList[end].Add(edge);
                    }
                }
            }

            // Method to find the longest trail
            public List<Line> FindLongestTrail(TimeSpan timeout)
            {
                // Create a CancellationTokenSource to enforce the timeout
                using (CancellationTokenSource cts = new CancellationTokenSource())
                {
                    cts.CancelAfter(timeout); // Set the time limit

                    try
                    {
                        return FindLongestTrailInternal(cts.Token);
                    }
                    catch (OperationCanceledException)
                    {
                        Rhino.RhinoApp.WriteLine("Operation was canceled due to timeout.");
                        return new List<Line>(); // Return an empty list if the operation times out
                    }
                }
            }
            private List<Line> FindLongestTrailInternal(CancellationToken token)
            {
                List<Line> longestTrail = new List<Line>();
                foreach (var vertex in Vertices)
                {
                    HashSet<Line> visitedEdges = new HashSet<Line>();
                    List<Line> currentTrail = new List<Line>();
                    DFS(vertex, visitedEdges, currentTrail, ref longestTrail, token);
                }
                return longestTrail;
            }
            private void DFS(Point3d vertex, HashSet<Line> visitedEdges, List<Line> currentTrail, ref List<Line> longestTrail, CancellationToken token)
            {
                token.ThrowIfCancellationRequested(); // Check if the operation has been canceled

                foreach (var edge in adjacencyList[vertex])
                {
                    if (!visitedEdges.Contains(edge))
                    {
                        visitedEdges.Add(edge);
                        currentTrail.Add(edge);

                        Point3d nextVertex = edge.From == vertex ? edge.To : edge.From;

                        DFS(nextVertex, visitedEdges, currentTrail, ref longestTrail, token);

                        visitedEdges.Remove(edge);
                        currentTrail.RemoveAt(currentTrail.Count - 1);
                    }
                }

                if (currentTrail.Count > longestTrail.Count)
                {
                    longestTrail = new List<Line>(currentTrail);
                }
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

            List<Line> longestTrail = graph.FindLongestTrail(timeout);

            // Set the output data
            DA.SetDataList(0, longestTrail);



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