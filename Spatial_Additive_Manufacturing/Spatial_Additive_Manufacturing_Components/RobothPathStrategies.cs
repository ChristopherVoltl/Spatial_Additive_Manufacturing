using Rhino.Geometry;
using SMT;
using System;
using System.Collections.Generic;
using static SMT.SMTUtilities;
using System.Diagnostics.Metrics;
using System.Reflection;


public interface IPathPointStrategy
{
    List<PathPointCommand> GetPathPoints(PathCurve pathCurve);
}


public struct PathPointCommand
{
    public Point3d Point { get; }
    public double E5Value { get; }
    public bool CoolingOn { get; }
    public bool ExtrudeOn { get; }
    public bool HeatOn { get; }
    public float VelRatio { get; }
    public bool CycleWait { get; }


    public PathPointCommand(Point3d point, double e5Value, bool coolingOn, bool extrudeOn, bool heatOn, float velRatio, bool cycleWait)
    {
        Point = point;
        E5Value = e5Value;
        CoolingOn = coolingOn;
        ExtrudeOn = extrudeOn;
        HeatOn = heatOn;
        VelRatio = velRatio;
        CycleWait = cycleWait;
    }
}
public static class PathPointStrategyFactory
{
    public static IPathPointStrategy GetStrategy(PathCurve pathCurve)
    {
        switch (pathCurve.Orientation)
        {
            case PathCurve.OrientationType.Horizontal:
                return new HorizontalStrategy();

            case PathCurve.OrientationType.AngledUp:
                return new AngledUpStrategy();

            case PathCurve.OrientationType.AngledDown:
                return new AngledDownStrategy();

            case PathCurve.OrientationType.Vertical:
                return new VerticalStrategy();

            default:
                throw new InvalidOperationException($"Unsupported orientation: {pathCurve.Orientation}");
        }
    }



    private class AngledDownStrategy : IPathPointStrategy
    {
        public List<PathPointCommand> GetPathPoints(PathCurve pathCurve)
        {
            List<PathPointCommand> sequence = new();

            Point3d pathStart = pathCurve.StartPoint;
            Point3d pathEnd = pathCurve.EndPoint;
            Point3d pullbackPointvert = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + pathCurve.Line.Length / 4);

            // Pullback point
            Point3d pullbackPoint = new Point3d(pathStart.X, pathStart.Y, pathStart.Z);
            Point3d pullbackDir = new Point3d(pathStart.X, pathStart.Y, pathEnd.Z);

            Vector3d pullbackVector = pullbackDir - pathEnd;
            pullbackVector.Unitize();
            pullbackVector *= pathCurve.Line.Length / 2.2;


            pullbackPoint.Transform(Transform.Translation(pullbackVector));

            // Modified end point
            double voxelHeight = pathStart.Z - pathEnd.Z;
            Point3d modifiedEndPoint = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + (voxelHeight * 2.0));

            // Build sequence:
            float velRatio = 0.05f;


            // Pullback point 
            sequence.Add(new PathPointCommand(
                pullbackPoint, 2.4, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path start
            sequence.Add(new PathPointCommand(
                pullbackPointvert, 2.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path start
            sequence.Add(new PathPointCommand(
                pullbackPointvert, 1.6, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Modified end 
            sequence.Add(new PathPointCommand(
                modifiedEndPoint, 1.6, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path end 
            if (pathCurve.Line.Length < 25.0)
            {
                // Adjust Z for short paths
                pathEnd = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 6.5);

                // Modified end  to change e5
                sequence.Add(new PathPointCommand(
                    modifiedEndPoint, 1.2, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

                sequence.Add(new PathPointCommand(
                    pathEnd, 1.2, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
            }
            else if (pathCurve.Line.Length > 50.0)
            {
                pathEnd = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z);

                // Modified end   to change e5
                sequence.Add(new PathPointCommand(
                    modifiedEndPoint, 0.8, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

                sequence.Add(new PathPointCommand(
                    pathEnd, 0.8, coolingOn: true, extrudeOn: false, heatOn: true, velRatio: velRatio, cycleWait: false));
            }
            else
            {
                pathEnd = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 4.5);

                // Modified end   to change e5
                sequence.Add(new PathPointCommand(
                    modifiedEndPoint, 1.2, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

                sequence.Add(new PathPointCommand(
                pathEnd, 1.2, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: velRatio, cycleWait: false));
            }
            

            return sequence;
        }
    }

    private class VerticalStrategy : IPathPointStrategy
    {
        public List<PathPointCommand> GetPathPoints(PathCurve pathCurve)
        {
            List<PathPointCommand> sequence = new();

            Point3d pathStart = pathCurve.StartPoint;
            Point3d pathEnd = pathCurve.EndPoint;

            // Parameters
            double startExtruding = 4.5;
            double startCooling = 1.0;
            double slowExtruding = 4.0;
            double e5 = 3.0;
            float velRatio = 0.05f;
            

            Point3d startExtruding_pt = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + startExtruding);
            Point3d slowExtruding_pt = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z - slowExtruding);

            LineCurve modifiedPath = new LineCurve(startExtruding_pt, pathEnd);
            modifiedPath.LengthParameter(startCooling, out double startCoolingParam);
            Point3d startCooling_pt = modifiedPath.PointAt(startCoolingParam);

            double lineLength = modifiedPath.Line.Length;
            double step = 15.0;



            // Build sequence:
            if (pathCurve.Line.Length  > 50.0)
            {
                sequence.Add(new PathPointCommand(startExtruding_pt, e5 * 2.0, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: velRatio, cycleWait: false));

                sequence.Add(new PathPointCommand(startCooling_pt, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
            }
            else
            {
                startExtruding_pt = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + startExtruding + 1.0);
                startCooling_pt = new Point3d(startCooling_pt.X, startCooling_pt.Y, startCooling_pt.Z + 1.0);

                sequence.Add(new PathPointCommand(startExtruding_pt, e5 * 2.0, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: velRatio, cycleWait: false));

                sequence.Add(new PathPointCommand(startCooling_pt, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            }
                
            if (lineLength > step)
            {
                int numSteps = (int)Math.Floor(lineLength / step);

                for (int i = 1; i <= numSteps; i++)
                {
                    double t = i * step / lineLength;
                    Point3d wait_pt = pathCurve.Line.PointAt(t);
                    Point3d wait_pt2 = new Point3d(wait_pt.X, wait_pt.Y, wait_pt.Z - 2.5);
                    // Decrease E5 value for each step 
                    velRatio = velRatio * 0.38f;   

                    sequence.Add(new PathPointCommand(wait_pt2, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
                    sequence.Add(new PathPointCommand(wait_pt, e5 / 1.2, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio / 3.2f, cycleWait: true));
                }
            }


            //sequence.Add(new PathPointCommand(slowExtruding_pt, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
            sequence.Add(new PathPointCommand(pathEnd, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio/2, cycleWait: true));

            return sequence;
        }
    }

    private class AngledUpStrategy : IPathPointStrategy
    {
        public List<PathPointCommand> GetPathPoints(PathCurve pathCurve)
        {
            List<PathPointCommand> sequence = new();

            Point3d pathStart = pathCurve.StartPoint;
            Point3d pathEnd = pathCurve.EndPoint;

            // Length based parameters:
            Curve curve = new LineCurve(pathStart, pathEnd);
            double crvLength = curve.GetLength();
            double stopExtruding = crvLength * 0.85;
            double startCooling = crvLength * 0.06;
            double startExtruding = 2.5;

            // Build startExtruding_pt
            Point3d startExtruding_pt = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + startExtruding);

            // Build modified path for parameter calculation
            Curve pathModified = new LineCurve(startExtruding_pt, pathEnd);

            pathModified.LengthParameter(startCooling, out double startCoolingParam);
            Point3d startCooling_pt = pathModified.PointAt(startCoolingParam);

            pathModified.LengthParameter(stopExtruding, out double stopExtrudeParam);
            Point3d stopExtruding_pt = pathModified.PointAt(stopExtrudeParam);

            // Build sequence:

            // StartExtrudingPlane → Extrude ON
            sequence.Add(new PathPointCommand(
                startExtruding_pt, 2.4, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: 0.05f, cycleWait: false));

            // StartCoolingPlane → Cooling ON, Extrude ON
            //sequence.Add(new PathPointCommand(
                //startCooling_pt, 2.4, coolingOn: true, extrudeOn: true, heatOn: false));

            // StopExtrudingPlane → Extrude OFF, Cooling ON
            //sequence.Add(new PathPointCommand(
                //stopExtruding_pt, 2.4, coolingOn: true, extrudeOn: false, heatOn: false));

            // StopExtrudingPlane again → CycleWait point → here modeled as Cooling OFF, Extrude OFF
            //sequence.Add(new PathPointCommand(
                //stopExtruding_pt, 2.4, coolingOn: false, extrudeOn: false, heatOn: false));

            // StopExtrudingPlane again → Extrude ON again (based on your code → strange but matches your logic)
            //sequence.Add(new PathPointCommand(
                //stopExtruding_pt, 2.4, coolingOn: false, extrudeOn: true, heatOn: false));

            // PathEnd → Final stopExtrude
            sequence.Add(new PathPointCommand(
                pathEnd, 2.4, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: 0.05f, cycleWait: false));

            return sequence;
        }
    }

    private class HorizontalStrategy : IPathPointStrategy
    {
        private const double ZHeightTest = 609.0;

        public List<PathPointCommand> GetPathPoints(PathCurve pathCurve)
        {
            List<PathPointCommand> sequence = new();

            Point3d pathStart = pathCurve.StartPoint;
            Point3d pathEnd = pathCurve.EndPoint;

            double t = 0.5;
            Curve curve = new LineCurve(pathStart, pathEnd);
            curve.Domain = new Interval(0, 1);
            Point3d pointOnCurve = curve.PointAt(t);


            Point3d pathStartEdit = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + 4);

            // move Z + 10
            Point3d pathEndMoveZ = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 15);
            Point3d pathEndMoveZFinal = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 4);

            Curve curveNewZ = new LineCurve(pathStart, pathEndMoveZ);

            
            // If Z > threshold
            if (pointOnCurve.Z > ZHeightTest)
            {
                

                // Path end 
                if (pathCurve.Line.Length < 25.0)
                {
                    // Adjust Z for short paths
                    Point3d pathStartEdit_short = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + 4.5);
                    Point3d pathEndMoveZFinal_short = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 5.0);
                    // Start point → Cooling ON, Extrude ON, E5 = 2.0
                    sequence.Add(new PathPointCommand(
                        pathStartEdit_short, 1.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: 0.05f, cycleWait: false));

                    sequence.Add(new PathPointCommand(
                    pathEndMoveZ, 1.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: 0.05f, cycleWait: false));

                    // pathEnd → stopExtrude
                    sequence.Add(new PathPointCommand(
                        pathEndMoveZFinal_short, 1.4, coolingOn: false, extrudeOn: false, heatOn: true, velRatio: 0.05f, cycleWait: false));

                }
                else
                {
                    // Start point → Cooling ON, Extrude ON, E5 = 2.0
                    sequence.Add(new PathPointCommand(
                        pathStartEdit, 1.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: 0.05f, cycleWait: false));

                    sequence.Add(new PathPointCommand(
                        pathEndMoveZ, 1.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: 0.05f, cycleWait: false));

                    // pathEnd → stopExtrude
                    sequence.Add(new PathPointCommand(
                        pathEndMoveZFinal, 1.4, coolingOn: false, extrudeOn: false, heatOn: true, velRatio: 0.05f, cycleWait: false));
                }
            }
            else // Z <= threshold
            {
                // Start point → Cooling OFF, Extrude ON, E5 = 1.6
                sequence.Add(new PathPointCommand(
                    pathStart, 1.6, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: 0.05f, cycleWait: false));


                // pathEnd → stopExtrude
                sequence.Add(new PathPointCommand(
                    pathEnd, 1.6, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: 0.05f, cycleWait: false));
            }

            return sequence;
        }
    }

    private class SimpleDebugStrategy : IPathPointStrategy
    {
        public List<PathPointCommand> GetPathPoints(PathCurve pathCurve)
        {
            List<PathPointCommand> sequence = new();

            // Add StartPoint → extrude ON, cooling OFF
            sequence.Add(new PathPointCommand(
                pathCurve.StartPoint, 1.0, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: 0.05f, cycleWait: false));

            // Add EndPoint → extrude OFF, cooling OFF
            sequence.Add(new PathPointCommand(
                pathCurve.EndPoint, 1.0, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: 0.05f, cycleWait: false));

            return sequence;
        }
    }

}