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

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class GraphSorting : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public GraphSorting()
          : base("Graph Sorting", "GS",
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
            pManager.AddPointParameter("Graph_Nodes", "GN", "Graph: Nodes", GH_ParamAccess.tree);
            pManager.AddPointParameter("Graph_Node_Connections", "GNC", "Graph: Graph Node Connectivity", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Graph_Node_Connections_Length", "GNCL", "Graph: Graph Node Connectivity Count Per Node", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Graph_Node_Weights", "GNW", "Graph: Weight of the edge for sorting", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Sorted_Graph_Node_Weights", "SGNW", "Graph: Sorted weight of the edge for sorting", GH_ParamAccess.tree);
            pManager.AddLineParameter("Sorted_Graph_Lines", "SGL", "Graph:  Sorted lines", GH_ParamAccess.tree);
            pManager.AddLineParameter("Debug Lines", "SGL", "Graph:  Sorted lines", GH_ParamAccess.tree);





        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public class Graph
        {
            // Dictionary to store nodes and their connections
            private Dictionary<Point3d, HashSet<Point3d>> nodes;

            public Graph()
            {
                nodes = new Dictionary<Point3d, HashSet<Point3d>>();
            }

            // Add a node to the graph
            public void AddNode(Point3d node)
            {
                if (!nodes.ContainsKey(node))
                {
                    nodes[node] = new HashSet<Point3d>(); // Initialize an empty set for connections
                }
            }

            // Add an edge between two nodes
            public void AddEdge(Point3d node1, Point3d node2)
            {
                AddNode(node1);
                AddNode(node2);
                nodes[node1].Add(node2);
                nodes[node2].Add(node1);
            }

            // Create graph from lines
            public void CreateGraphFromLines(IEnumerable<Line> lines)
            {
                foreach (var line in lines)
                {
                    Point3d startPoint = line.From;
                    Point3d endPoint = line.To;
                    AddEdge(startPoint, endPoint);
                }
            }

            // Get the nodes and their connections
            public Dictionary<Point3d, HashSet<Point3d>> GetNodes()
            {
                return nodes;
            }
        }

        public class LineProcessor
        {
            public static Dictionary<Line, double> AddWeightToLines(List<Line> graphLines)
            {
                if (graphLines == null || graphLines.Count == 0)
                    throw new ArgumentException("Input list of lines is empty");

                Dictionary<Line, double> weights = new Dictionary<Line, double>();


                foreach (var line in graphLines)
                {
                    Point3d sp = line.From;
                    Point3d ep = line.To;

                    // Ensure line direction is correct
                    if (sp.Z > ep.Z)
                        line.Flip();

                    // Calculate average Z height
                    double averageZ = (sp.Z + ep.Z) / 2;
                    double weight = 0;

                    if (Math.Abs(sp.Z - ep.Z) < 0.01)
                    {
                        // Horizontal line
                        // No weight changes for horizontal lines
                    }
                    else if (Math.Round(sp.X, 2) == Math.Round(ep.X, 2) &&
                             Math.Round(sp.Y, 2) == Math.Round(ep.Y, 2))
                    {
                        // Vertical line
                        weight += FindIntersectionVerticalAtEnd(line, graphLines);
                        weight += FindIntersectionVerticalAtStart(line, graphLines);
                        weight += VerticalNoAngleAtTop(line, graphLines);
                    }
                    else
                    {
                        // Angled line
                        weight += FindIntersectionAngledAtStart(line, graphLines);
                        weight += FindIntersectionAngledAtEnd(line, graphLines);
                        weight += VerticalsWithAngledAtStart(line, graphLines);
                    }

                    // Apply final weight adjustment
                    weight += CalculateWeight(averageZ) - 32.481;
                    weight = Math.Round(weight, 3);

                    weights[line] = weight;
                }

                return weights;
            }


            private static double FindIntersectionVerticalAtStart(Line inputLine, List<Line> graphLines)
            {
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                double weightVertical = 0;

                // Flip the input line if start Z > end Z
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                foreach (var line in graphLines)
                {
                    // Check if line is nearly horizontal
                    if (Math.Abs(line.To.Z - line.From.Z) <= 0.02)
                    {
                        weightVertical = 0;
                    }
                    // If both points of the line are identical
                    else if (line.From.X == line.To.X && line.From.Y == line.To.Y)
                    {
                        weightVertical = 0;
                    }
                    else
                    {
                        // Flip the line if From.Z > To.Z
                        if (line.From.Z > line.To.Z)
                        {
                            line.Flip();
                        }

                        // Check for intersection at the start point
                        if (Math.Round(line.From.X, 2) == Math.Round(startPoint.X, 2) &&
                            Math.Round(line.From.Y, 2) == Math.Round(startPoint.Y, 2) &&
                            Math.Round(line.From.Z, 2) == Math.Round(startPoint.Z, 2))
                        {
                            weightVertical = 0.2;
                            return weightVertical;
                        }
                    }
                }

                return weightVertical;
            }

            private static double FindIntersectionVerticalAtEnd(Line inputLine, List<Line> graphLines)
            {
                // Find the intersection for a vertical line
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                double weightVertical = 0;

                // Flip the input line if the start point's Z is greater than the end point's Z
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                foreach (var line in graphLines)
                {
                    // Check if the input line is approximately horizontal
                    if (Math.Abs(endPoint.Z - startPoint.Z) <= 0.02)
                    {
                        weightVertical = 0;
                    }
                    // Check if the graph line is vertical
                    else if (line.From.X == line.To.X && line.From.Y == line.To.Y)
                    {
                        weightVertical = 0;
                    }
                    else
                    {
                        // Flip the graph line if necessary
                        if (line.From.Z > line.To.Z)
                        {
                            line.Flip();
                        }

                        // Intersection at the endpoint
                        if (Math.Round(line.To.X, 2) == Math.Round(endPoint.X, 2) &&
                            Math.Round(line.To.Y, 2) == Math.Round(endPoint.Y, 2) &&
                            Math.Round(line.To.Z, 2) == Math.Round(endPoint.Z, 2))
                        {
                            weightVertical = 0.0;
                            return weightVertical;
                        }
                    }
                }

                return weightVertical;
            }

            private static double FindIntersectionAngledAtStart(Line inputLine, List<Line> graphLines)
            {
                // Find the intersection for a vertical line, assuming inputLine is vertical
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                double weightAngled = 0;

                // Flip the input line if the start point's Z is greater than the end point's Z
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                foreach (var line in graphLines)
                {
                    // Check if the input line is approximately horizontal
                    if (Math.Abs(endPoint.Z - startPoint.Z) <= 0.02)
                    {
                        weightAngled = 0;
                    }
                    // Check if the graph line is vertical
                    else if (Math.Round(line.From.X, 2) == Math.Round(line.To.X, 2) &&
                             Math.Round(line.From.Y, 2) == Math.Round(line.To.Y, 2))
                    {
                        // Flip the graph line if necessary
                        if (line.From.Z > line.To.Z)
                        {
                            line.Flip();
                        }

                        // Check for intersection at the start point
                        if (Math.Round(line.From.X, 2) == Math.Round(startPoint.X, 2) &&
                            Math.Round(line.From.Y, 2) == Math.Round(startPoint.Y, 2) &&
                            Math.Round(line.From.Z, 2) == Math.Round(startPoint.Z, 2))
                        {
                            weightAngled = -0.2;
                            return weightAngled;
                        }
                    }
                    else
                    {
                        weightAngled = 0;
                    }
                }

                return weightAngled;
            }

            private static double FindIntersectionAngledAtEnd(Line inputLine, List<Line> graphLines)
            {
                // Find the intersection for a vertical line
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                double weightAngled = 0;

                // Flip the input line if the start point's Z is greater than the end point's Z
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                foreach (var line in graphLines)
                {
                    // Check if the line is approximately horizontal
                    if (Math.Abs(line.To.Z - line.From.Z) <= 0.02)
                    {
                        weightAngled = 0;
                    }
                    // Check if the line is vertical
                    else if (line.From.X == line.To.X && line.From.Y == line.To.Y)
                    {
                        if (line.From.Z > line.To.Z)
                        {
                            line.Flip();
                        }

                        // Check for intersection at the end point
                        if (Math.Round(line.To.X, 2) == Math.Round(endPoint.X, 2) &&
                            Math.Round(line.To.Y, 2) == Math.Round(endPoint.Y, 2) &&
                            Math.Round(line.To.Z, 2) == Math.Round(endPoint.Z, 2))
                        {
                            weightAngled = 0.2;
                            return weightAngled;
                        }
                    }
                    else
                    {
                        weightAngled = 0;
                    }
                }

                return weightAngled;
            }

            private static double VerticalNoAngleAtTop(Line inputLine, List<Line> graphLines)
            {
                // Intersection logic for angled lines at the end
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                // Calculate the average Z of the input line
                double averageZInputLine = (startPoint.Z + endPoint.Z) / 2;
                double weightVertical = 0;

                // Flip the input line if needed
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                int count = 0;

                // Iterate through graph lines
                foreach (var line in graphLines)
                {
                    // Check if the line is horizontal
                    if (Math.Abs(line.To.Z - line.From.Z) <= 0.02)
                    {
                        weightVertical = 0;
                    }
                    // Check if the line is a vertical line
                    else if (line.From.X == line.To.X && line.From.Y == line.To.Y)
                    {
                        weightVertical = 0;
                    }
                    else
                    {
                        // Flip the line if needed
                        if (line.From.Z > line.To.Z)
                        {
                            line.Flip();
                        }

                        // Calculate the average Z of the line
                        double averageZLine = (line.From.Z + line.To.Z) / 2;

                        // Check for intersection at the endpoint point
                        if (averageZLine <= averageZInputLine)
                        {
                            if (Math.Round(line.To.X, 2) == Math.Round(endPoint.X, 2) &&
                                Math.Round(line.To.Y, 2) == Math.Round(endPoint.Y, 2) &&
                                Math.Round(line.To.Z, 2) == Math.Round(endPoint.Z, 2))
                            {
                                count++;
                            }
                        }
                    }
                }

                // Update the weight if no intersections were found
                if (count == 0)
                {
                    weightVertical = 0.2;
                }
                else
                {
                    weightVertical = -0.2;
                }

                return weightVertical;
            }

            private static double VerticalsWithAngledAtStart(Line inputLine, List<Line> graphLines)
            {
                // Find the intersection for a vertical line
                Point3d startPoint = inputLine.From;
                Point3d endPoint = inputLine.To;

                double weightAngled = 0;

                // Flip the input line if the start point Z is greater than the end point Z
                if (startPoint.Z > endPoint.Z)
                {
                    inputLine.Flip();
                }

                // Test if the line connected to the input line is vertical and attached to the end at both lines
                foreach (var line in graphLines)
                {
                    Point3d linePoint = line.PointAt(line.Length / 2);

                    // Flip the line if its start point Z is greater than its end point Z
                    if (line.From.Z > line.To.Z)
                    {
                        line.Flip();
                    }

                    // Test if the same line
                    if (Math.Round(line.From.X, 2) == Math.Round(startPoint.X, 2) &&
                        Math.Round(line.From.Y, 2) == Math.Round(startPoint.Y, 2) &&
                        Math.Round(line.From.Z, 2) == Math.Round(startPoint.Z, 2) &&
                        Math.Round(line.To.X, 2) == Math.Round(endPoint.X, 2) &&
                        Math.Round(line.To.Y, 2) == Math.Round(endPoint.Y, 2) &&
                        Math.Round(line.To.Z, 2) == Math.Round(endPoint.Z, 2))
                    {
                        continue; // Skip processing as it's the same line
                    }

                    // Test if the same endpoint
                    else if (Math.Round(endPoint.X, 2) == Math.Round(line.To.X, 2) &&
                             Math.Round(endPoint.Y, 2) == Math.Round(line.To.Y, 2) &&
                             Math.Round(endPoint.Z, 2) == Math.Round(line.To.Z, 2))
                    {
                        if (Math.Round(line.From.X, 2) == Math.Round(line.To.X, 2) &&
                            Math.Round(line.From.Y, 2) == Math.Round(line.To.Y, 2))
                        {
                            weightAngled = FindIntersectionVerticalAtStart(line, graphLines);
                            return weightAngled;
                        }
                    }
                }

                return weightAngled;
            }

            private static double CalculateWeight(double averageZ)
            {
                // Weight calculation logic
                return averageZ; // Placeholder logic
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
            foreach (var curve in curves) {
                Polyline polyline;
                curve.TryGetPolyline(out polyline);
                for (int i = 0; i < polyline.Count - 1; i++)
                {
                    lines.Add(new Line(polyline[i], polyline[i + 1]));
                }
            }

            // Initialize a spatial graph
            Graph spatialGraph = new Graph();


            // Create the graph from lines
            spatialGraph.CreateGraphFromLines(lines);

            // Retrieve nodes and their connections
            var nodes = spatialGraph.GetNodes();

            // Collect nodes and connections for further processing if needed
            List<Point3d> nodeList = new List<Point3d>();
            List<Point3d> nodeConnections = new List<Point3d>();
            List<int> connectionLengths = new List<int>();
            int counter = 0;

            foreach (var node in nodes)
            {
                nodeList.Add(node.Key);
                connectionLengths.Add(node.Value.Count);
                GH_Path path = new GH_Path(counter);

                foreach (var connection in node.Value)
                {
                    nodeConnections.Add(connection);
                    
                    graftedTree.Append(new GH_Point(connection), path);
                }
                counter++;
            }

            Dictionary<Line, double> weightsDict = LineProcessor.AddWeightToLines(lines);

            var weights = new List<double>();
            var weightedEdges = new List<Line>(); 
            var debugLines = new List<Line>();

            foreach (var kvp in weightsDict)
            {
                weights.Add(kvp.Value);
                weightedEdges.Add(kvp.Key);
            }

            //sort weights
            var spatialDict = new Dictionary<Line, double>(); // Replace 'Line' with the appropriate type for lines

            for (int i = 0; i < lines.Count; i++)
            {
                spatialDict[lines[i]] = weights[i];
            }

            

            var sortedWeights = new List<double>();
            var sortedEdges = new List<Line>();

            
            LineProcessing lineProcessing = new LineProcessing();
            (weightedEdges, weights, debugLines) = lineProcessing.CombineCurves(weightedEdges, weights);

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
            DA.SetDataList(0, lines);
            DA.SetDataList(1, nodeList);
            DA.SetDataTree(2, graftedTree);
            DA.SetDataList(3, connectionLengths);
            DA.SetDataList(4, weights);
            DA.SetDataList(5, sortedWeights);
            DA.SetDataList(6, sortedEdges);
            DA.SetDataList(7, debugLines);


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
            get { return new Guid("039d4497-4d98-4b3b-806f-5e11f7a71657"); }
        }
    }
}