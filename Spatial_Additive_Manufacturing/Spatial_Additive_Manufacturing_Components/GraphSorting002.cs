using System;
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
using System.Diagnostics;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class GraphSorting002 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public GraphSorting002()
          : base("Graph Sorting 0.02", "GS",
              "Generates a graph from the spatial toolpaths and sorts to find a collision free robotic toolpath",
              "FGAM", "Toolpathing")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("pathCurves", "pC", " an array of Curves", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Graph_Lines", "GL", "Graph:  Lines", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Graph_Weights", "GN", "Graph: Nodes", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Sorted_Graph_Node_Weights", "SGNW", "Graph: Sorted weight of the edge for sorting", GH_ParamAccess.tree);
            pManager.AddLineParameter("Sorted_Graph_Lines", "SGL", "Graph:  Sorted lines", GH_ParamAccess.tree);





        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public class LineConnectivityFinder
        {
            public static Dictionary<Line, (List<Line> startConnections, List<Line> endConnections)> FindConnectedLines(List<Line> lines)
            {
                Dictionary<Point3d, List<Line>> pointConnections = BuildPointConnections(lines);
                Dictionary<Line, (List<Line>, List<Line>)> connectedLines = new Dictionary<Line, (List<Line>, List<Line>)>();

                foreach (var line in lines)
                {
                    Point3d sp = line.From;
                    Point3d ep = line.To;

                    List<Line> startConnections = pointConnections.ContainsKey(sp)
                        ? pointConnections[sp].Where(l => l != line).ToList()
                        : new List<Line>();

                    List<Line> endConnections = pointConnections.ContainsKey(ep)
                        ? pointConnections[ep].Where(l => l != line).ToList()
                        : new List<Line>();

                    connectedLines[line] = (startConnections, endConnections);
                }

                return connectedLines;
            }

            private static Dictionary<Point3d, List<Line>> BuildPointConnections(List<Line> lines)
            {
                Dictionary<Point3d, List<Line>> connections = new Dictionary<Point3d, List<Line>>();

                foreach (var line in lines)
                {
                    if (!connections.ContainsKey(line.From))
                        connections[line.From] = new List<Line>();
                    if (!connections.ContainsKey(line.To))
                        connections[line.To] = new List<Line>();

                    connections[line.From].Add(line);
                    connections[line.To].Add(line);
                }

                return connections;
            }
        }
        public class LineProcessor
        {
            public static Dictionary<Line, double> AssignWeightsForSorting(List<Line> graphLines)
            {
                if (graphLines == null || graphLines.Count == 0)
                    throw new ArgumentException("Input list of lines is empty");

                Dictionary<Line, double> weights = new Dictionary<Line, double>();
                Dictionary<Point3d, List<Line>> pointConnections = BuildPointConnections(graphLines);

                foreach (var line in graphLines)
                {
                    Point3d sp = line.From;
                    Point3d ep = line.To;

                    if (sp.Z > ep.Z)
                        line.Flip(); // Ensure direction is consistent
                        sp = line.From;
                        ep = line.To;

                    double baseWeight = (ep.Z); // Base weight from Z-height
                    double weightAdjustment = 0;

                    bool isVertical = IsVertical(line);
                    bool isHorizontal = IsHorizontal(line);
                    bool isAngled = !isVertical && !isHorizontal;

                    if (isHorizontal)
                    {
                        // Only weight by Z-height, no connectivity adjustments
                        weights[line] = Math.Round(baseWeight, 3) + 0.9; 
                        continue;
                    }

                    if (isVertical)
                    {
                        if (IsConnectedAtEndToAngled(ep, line, pointConnections))
                            weightAdjustment -= 0.5; // Vertical prints first at endpoint

                        if (IsConnectedAtStartToAngled(sp, line, pointConnections))
                            weightAdjustment += 0.5; // Angled should print first at start
                    }
                    if (isAngled)
                    {
                        if (IsConnectedAtEndToVertical(ep, line, pointConnections))
                            weightAdjustment += 0.05; // Vertical prints first, delay angled

                        if (IsConnectedAtStartToVertical(sp, line, pointConnections))
                            weightAdjustment -= 0.05; // Angled prints first
                    }

                    double finalWeight = Math.Round(baseWeight + weightAdjustment, 3);
                    weights[line] = finalWeight;
                }

                // Step 2: Run Refinement Function to resolve conflicts
                RefineWeights(weights, pointConnections, 5);



                return weights;
            }
            private static void RefineWeights(Dictionary<Line, double> weights, Dictionary<Point3d, List<Line>> pointConnections,int limit)
            {
                if (limit <= 0)
                    return;

                // Group lines by weight value
                var groupedWeights = weights.GroupBy(kvp => kvp.Value).Where(g => g.Count() > 1);

                foreach (var group in groupedWeights)
                {
                    List<Line> conflictingLines = group.Select(kvp => kvp.Key).ToList();

                    var connections = LineConnectivityFinder.FindConnectedLines(conflictingLines);

                    foreach (var connection in connections)
                    {
                        bool isVertical = IsVertical(connection.Key);

                        if(isVertical)
                        {
                            //add weight to line connected to start point
                            int counter = 0;
                            foreach (var line in connection.Value.startConnections)
                            {
                                if (weights.ContainsKey(line))
                                    weights[line] -= 0.05;  
                                if (counter == 0)
                                {
                                    weights[connection.Key] += 0.05;
                                    counter++;
                                }
                            }
                            counter = 0;
                            //add weight to line connected to end point
                            foreach (var line in connection.Value.endConnections)
                            {
                                if (weights.ContainsKey(line))
                                    weights[line] += 0.05;
                                if (counter == 0)
                                {
                                    weights[connection.Key] -= 0.05;
                                    counter++;
                                }
                            }
                            counter = 0;
                        }
                    }
                }
                RefineWeights(weights, pointConnections, limit - 1);
            }
            private static Dictionary<Point3d, List<Line>> BuildPointConnections(List<Line> lines)
            {
                Dictionary<Point3d, List<Line>> connections = new Dictionary<Point3d, List<Line>>();
                foreach (var line in lines)
                {
                    if (!connections.ContainsKey(line.From))
                        connections[line.From] = new List<Line>();
                    if (!connections.ContainsKey(line.To))
                        connections[line.To] = new List<Line>();

                    connections[line.From].Add(line);
                    connections[line.To].Add(line);
                }
                return connections;
            }

            private static bool IsVertical(Line line)
            {
                return Math.Round(line.From.X, 2) == Math.Round(line.To.X, 2) &&
                       Math.Round(line.From.Y, 2) == Math.Round(line.To.Y, 2);
            }

            private static bool IsHorizontal(Line line)
            {
                return Math.Abs(line.From.Z - line.To.Z) < 0.01;
            }

            private static bool IsConnectedAtEndToAngled(Point3d end, Line line, Dictionary<Point3d, List<Line>> connections)
            {
                if (!connections.ContainsKey(end)) return false;

                foreach (var connectedLine in connections[end])
                {
                    if (connectedLine == line) continue;
                    if (!IsVertical(connectedLine) && !IsHorizontal(connectedLine)) return true; // Angled found
                }
                return false;
            }

            private static bool IsConnectedAtStartToAngled(Point3d start, Line line, Dictionary<Point3d, List<Line>> connections)
            {
                if (!connections.ContainsKey(start)) return false;

                foreach (var connectedLine in connections[start])
                {
                    if (connectedLine == line) continue;
                    if (!IsVertical(connectedLine) && !IsHorizontal(connectedLine)) return true; // Angled found
                }
                return false;
            }

            private static bool IsConnectedAtEndToVertical(Point3d end, Line line, Dictionary<Point3d, List<Line>> connections)
            {
                if (!connections.ContainsKey(end)) return false;

                foreach (var connectedLine in connections[end])
                {
                    if (connectedLine == line) continue;
                    if (IsVertical(connectedLine)) return true; // Vertical found
                }
                return false;
            }

            private static bool IsConnectedAtStartToVertical(Point3d start, Line line, Dictionary<Point3d, List<Line>> connections)
            {
                if (!connections.ContainsKey(start)) return false;

                foreach (var connectedLine in connections[start])
                {
                    if (connectedLine == line) continue;
                    if (IsVertical(connectedLine)) return true; // Vertical found
                }
                return false;
            }
        }

        public class LineProcessing
        {
            public static Vector3d GetLineVector(Line line)
            {
                // Create a Vector3d object from the start and end points of the line
                Point3d startPt = line.From;
                Point3d endPt = line.To;
                Vector3d lineVector = endPt - startPt;
                return lineVector;
            }

            public static bool ChainLinkingPt(Line lineA, Line lineB)
            {
                // This function will determine if the point is the connecting point between two adjacent curves.
                Point3d lineAEndPt = lineA.To;
                Point3d lineBStartPt = lineB.From;

                bool isConnected = false;

                if (Math.Round(lineAEndPt.X, 2) == Math.Round(lineBStartPt.X, 2) &&
                    Math.Round(lineAEndPt.Y, 2) == Math.Round(lineBStartPt.Y, 2) &&
                    Math.Round(lineAEndPt.Z, 2) == Math.Round(lineBStartPt.Z, 2))
                {
                    isConnected = true;
                }

                return isConnected;
            }

            public static bool HorizontalPtIntTest(Point3d pt, List<Line> lines)
            {
                // Test the point to see if it intersects with a point that is attached to a horizontal line in a set of curves  
                int count = 0;

                foreach (Line line in lines)
                {
                    Point3d start_pt = line.From;
                    Point3d end_pt = line.To;

                    // Testing if the start point of the curve is the same as pt
                    if (Math.Round(pt.X, 2) == Math.Round(start_pt.X, 2) &&
                        Math.Round(pt.Y, 2) == Math.Round(start_pt.Y, 2) &&
                        Math.Round(pt.Z, 2) == Math.Round(start_pt.Z, 2))
                    {
                        // If the points are equal, test if the curve that the point is attached to is horizontal
                        if (Math.Round(start_pt.Z, 2) == Math.Round(end_pt.Z, 2))
                        {
                            count += 1;
                        }
                    }
                    // Testing if the end point of the curve is the same as pt
                    else if (Math.Round(pt.X, 2) == Math.Round(end_pt.X, 2) &&
                             Math.Round(pt.Y, 2) == Math.Round(end_pt.Y, 2) &&
                             Math.Round(pt.Z, 2) == Math.Round(end_pt.Z, 2))
                    {
                        // If the points are equal, test if the curve that the point is attached to is horizontal
                        if (Math.Round(start_pt.Z, 2) == Math.Round(end_pt.Z, 2))
                        {
                            count += 1;
                        }
                    }
                }

                if (count >= 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }

            public static bool VectorsEqual(Line lineA, Line lineB)
            {
                // Check if the vectors of lineA and lineB are equal (parallel)
                Vector3d lineAVector = GetLineVector(lineA);
                Vector3d lineBVector = GetLineVector(lineB);
                lineAVector.Unitize();
                lineBVector.Unitize();

                if (Math.Round(lineAVector.X, 2) == Math.Round(lineBVector.X, 2) &&
                    Math.Round(lineAVector.Y, 2) == Math.Round(lineBVector.Y, 2) &&
                    Math.Round(lineAVector.Z, 2) == Math.Round(lineBVector.Z, 2))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            private bool ArePointsEqual(Point3d pt1, Point3d pt2)
            {
                if (Math.Round(pt1.X, 2) == Math.Round(pt2.X, 2) &&
                       Math.Round(pt1.Y, 2) == Math.Round(pt2.Y, 2) &&
                       Math.Round(pt1.Z, 2) == Math.Round(pt2.Z, 2))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            private void RemoveShortLine(Line lineB, List<Line> newLines, List<double> newWeights)
            {
                if (lineB.Length < 100)
                {
                    int index = newLines.IndexOf(lineB);
                    if (index != -1)
                    {
                        newLines.RemoveAt(index);
                        newWeights.RemoveAt(index);
                    }
                }
            }
            private bool IsNearEndpoint(double parameter, Curve curve)
            {
                const double tolerance = 1e-6; // Small tolerance to account for numerical precision
                return Math.Abs(parameter - curve.Domain.T0) < tolerance || Math.Abs(parameter - curve.Domain.T1) < tolerance;
            }

            public (List<Line>, List<double>, List<Line>) CombineCurves(List<Line> crvs, List<double> weights)
            {
                List<Line> newLines = new List<Line>();
                List<double> newWeights = new List<double>();
                List<Line> debugLines = new List<Line>();

                HashSet<int> mergedIndices = new HashSet<int>(); // Track indices of merged lines

                for (int i = 0; i < crvs.Count; i++)
                {
                    if (mergedIndices.Contains(i)) continue; // Skip lines that have already been merged

                    Line lineA = crvs[i];
                    Point3d lineAStartPt = lineA.From;
                    Point3d lineAEndPt = lineA.To;
                    bool addedLine = false;
                    double weight = weights[i];

                    if (lineAStartPt.Z > lineAEndPt.Z)
                    {
                        lineA.Flip();
                    }

                    for (int j = 0; j < crvs.Count; j++)
                    {
                        if (i == j || mergedIndices.Contains(j)) continue; // Skip already merged lines

                        Line lineB = crvs[j];
                        double altWeight = weights[j];

                        Point3d lineBStartPt = lineB.From;
                        Point3d lineBEndPt = lineB.To;

                        if (lineBStartPt.Z > lineBEndPt.Z)
                        {
                            lineB.Flip();
                        }

                        bool linkPt = ChainLinkingPt(lineA, lineB);
                        bool areVectorsEqual = VectorsEqual(lineA, lineB);

                        if (areVectorsEqual && linkPt)
                        {
                            double dist = Math.Abs(lineAStartPt.DistanceTo(lineBEndPt));
                            bool lineAHorizontalTest = HorizontalPtIntTest(lineAEndPt, crvs);
                            bool lineBHorizontalTest = HorizontalPtIntTest(lineBStartPt, crvs);

                            if (lineAHorizontalTest && lineBHorizontalTest && dist < 80)
                            {
                                Line newLine = new Line(lineAStartPt, lineBEndPt);
                                newLines.Add(newLine);

                                if (Math.Round(lineBStartPt.X, 2) == Math.Round(lineBEndPt.X, 2) &&
                                    Math.Round(lineBStartPt.Y, 2) == Math.Round(lineBEndPt.Y, 2))
                                {
                                    newWeights.Add(altWeight + 0.13);
                                }
                                else
                                {
                                    newWeights.Add(altWeight);
                                }

                                mergedIndices.Add(i); // Mark original lines as merged
                                mergedIndices.Add(j);
                                addedLine = true;
                                break; // Exit inner loop once merged
                            }
                        }
                    }

                    if (!addedLine)
                    {
                        newLines.Add(lineA);
                        newWeights.Add(weight);
                    }
                }



                // Now, delete the remaining curve that overlaps the new curves
                for (int i = 0; i < newLines.Count; i++)
                {
                    Line lineA = newLines[i];
                    Point3d lineAStartPt = lineA.From;
                    Point3d lineAEndPt = lineA.To;
                    Point3d lineAMidpoint = lineA.PointAtLength(lineA.Length / 2);

                    int intersectionCount = 0;


                    debugLines.Add(lineA);
                    List<int> weightToDelete = new List<int>();
                    List<int> edgeToDelete = new List<int>();


                    for (int j = 0; j < newLines.Count; j++)
                    {


                        Line lineB = newLines[j];
                        Point3d lineBStartPt = lineB.From;
                        Point3d lineBEndPt = lineB.To;
                        Point3d lineBMidpoint = lineB.PointAtLength(lineB.Length / 2);

                        // Skip self-intersection checks
                        if (ArePointsEqual(lineA.To, lineB.To) && ArePointsEqual(lineA.From, lineB.From))
                        {
                            continue;
                        }

                        if (lineBStartPt.Z == lineBEndPt.Z)
                        {
                            // Horizontal line, no action

                            continue;
                        }

                        else if (Math.Round(lineBStartPt.X, 2) == Math.Round(lineBEndPt.X, 2) &&
                            Math.Round(lineBStartPt.Y, 2) == Math.Round(lineBEndPt.Y, 2) &&
                            Math.Round(lineBStartPt.Z, 2) == Math.Round(lineBEndPt.Z, 2))
                        {
                            // Same start and end points
                            continue;
                        }
                        //vertical line
                        else if (Math.Round(lineBStartPt.X, 2) == Math.Round(lineBEndPt.X, 2) &&
                            Math.Round(lineBStartPt.Y, 2) == Math.Round(lineBEndPt.Y, 2))
                        {
                            // Vertical line, find overlapping
                            Point3d paramA = lineB.PointAt(0.2);
                            Point3d paramB = lineB.PointAt(0.8);

                            Point3d paramAA = lineA.PointAt(0.2);
                            Point3d paramBA = lineA.PointAt(0.8);


                            //test paramA & paramB for intersection with lineB
                            Point3d paramA_lineA = lineA.ClosestPoint(paramA, true);
                            Point3d paramB_lineA = lineA.ClosestPoint(paramB, true);

                            Point3d paramA_lineB = lineB.ClosestPoint(paramAA, true);
                            Point3d paramB_lineB = lineB.ClosestPoint(paramBA, true);



                            if (ArePointsEqual(paramA, paramA_lineA) && ArePointsEqual(paramB, paramB_lineA))
                            {
                                if (lineB.Length < 40)
                                {
                                    weightToDelete.Add(j);
                                    edgeToDelete.Add(j);
                                }
                            }
                            else if (ArePointsEqual(paramAA, paramA_lineB) && ArePointsEqual(paramBA, paramB_lineB))
                            {

                                if (lineA.Length < 40)
                                {
                                    weightToDelete.Add(i);
                                    edgeToDelete.Add(i);
                                }
                            }
                            else
                            {
                                continue;
                            }
                        }
                        else

                        {

                            bool areVectorsEqual = VectorsEqual(lineA, lineB);

                            if (areVectorsEqual)
                            {
                                Point3d paramA = lineB.PointAt(0.2);
                                Point3d paramB = lineB.PointAt(0.8);

                                //test paramA & paramB for intersection with lineB
                                Point3d paramA_lineA = lineA.ClosestPoint(paramA, true);
                                Point3d paramB_lineA = lineA.ClosestPoint(paramB, true);

                                if (ArePointsEqual(paramA, paramA_lineA) && ArePointsEqual(paramB, paramB_lineA))
                                {
                                    if (lineB.Length < 40)
                                    {
                                        weightToDelete.Add(j);
                                        edgeToDelete.Add(j);
                                    }
                                    else if (lineA.Length < 40)
                                    {
                                        weightToDelete.Add(i);
                                        edgeToDelete.Add(i);
                                    }
                                }
                            }

                        }
                        // Additional conditions

                        for (int k = 0; k < weightToDelete.Count; k++)
                        {
                            newWeights.RemoveAt(weightToDelete[k]);
                            newLines.RemoveAt(edgeToDelete[k]);
                        }
                        weightToDelete.Clear();
                        edgeToDelete.Clear();
                    }
                }
                return (newLines, newWeights, debugLines);
            }
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare List of curves
            List<Curve> curves = new List<Curve>();

            // Retrieve the input data
            if (!DA.GetDataList(0, curves)) return;

            // Initialize a GH_Structure to store grafted data
            GH_Structure<GH_Point> graftedTree = new GH_Structure<GH_Point>();

            // 3. Abort on invalid inputs.
            if (curves == null)
            {
                RhinoApp.WriteLine("The selected object is not a curve.");
            }
            //convert curves to lines
            List<Line> lines = new List<Line>();
            foreach (var curve in curves)
            {
                Polyline polyline;
                curve.TryGetPolyline(out polyline);
                for (int i = 0; i < polyline.Count - 1; i++)
                {
                    lines.Add(new Line(polyline[i], polyline[i + 1]));
                }
            }

            Dictionary<Line, double> weightsDict = LineProcessor.AssignWeightsForSorting(lines);



            var weights = new List<double>();
            var weightedEdges = new List<Line>();

            foreach (var kvp in weightsDict)
            {
                weights.Add(kvp.Value);
                weightedEdges.Add(kvp.Key);
            }

            var sortedWeights = new List<double>();
            var sortedEdges = new List<Line>();

            // Sort by weight (value) and create a new dictionary
            var sortedByWeights = weightedEdges.Zip(weights, (a, b) => new { ItemA = a, ItemB = b })
                .OrderBy(x => x.ItemB)  // Sort by ListB's values
                .ToList();

            // Separate the sorted pairs back into individual lists
            List<Line> sortedEdgesA = sortedByWeights.Select(x => x.ItemA).ToList(); //issues here
            List<double> sortedWeightsB = sortedByWeights.Select(x => x.ItemB).ToList(); //issues here

            foreach (var line in sortedEdgesA)
            {
                sortedEdges.Add(line);
            }
            foreach (var weight in sortedWeightsB)
            {
                sortedWeights.Add(weight);
            }


            // Set the output data
            DA.SetDataList(0, weightedEdges);
            DA.SetDataList(1, weights);
            DA.SetDataList(2, sortedWeights);
            DA.SetDataList(3, sortedEdges);
        }


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
            get { return new Guid("82c5f475-2e61-4262-8f63-6bf27d9a1caf"); }
        }
    }
}
