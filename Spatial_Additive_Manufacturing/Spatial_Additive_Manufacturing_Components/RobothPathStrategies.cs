using Rhino.Geometry;
using SMT;
using System;
using System.Collections.Generic;


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

    public PathPointCommand(Point3d point, double e5Value, bool coolingOn, bool extrudeOn, bool heatOn)
    {
        Point = point;
        E5Value = e5Value;
        CoolingOn = coolingOn;
        ExtrudeOn = extrudeOn;
        HeatOn = heatOn;
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

            // Pullback point
            Point3d pullbackPoint = new Point3d(pathStart.X, pathStart.Y, pathStart.Z);
            Point3d pullbackDir = new Point3d(pathStart.X, pathStart.Y, pathEnd.Z);

            Vector3d pullbackVector = pullbackDir - pathEnd;
            pullbackVector.Unitize();
            pullbackVector *= 10.0;

            pullbackPoint.Transform(Transform.Translation(pullbackVector));

            // Modified end point
            double voxelHeight = pathStart.Z - pathEnd.Z;
            Point3d modifiedEndPoint = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + (voxelHeight * 0.5));

            // Build sequence:

            // Pullback point → extrude ON
            sequence.Add(new PathPointCommand(
                pullbackPoint, 1.1, coolingOn: false, extrudeOn: true, heatOn: false));

            // Path start → extrude ON
            sequence.Add(new PathPointCommand(
                pathStart, 2.4, coolingOn: false, extrudeOn: true, heatOn: false));

            // Path start → cool ON (extrude stays ON → up to you to decide if it should flip)
            sequence.Add(new PathPointCommand(
                pathStart, 2.4, coolingOn: true, extrudeOn: true, heatOn: false));

            // Modified end → stopExtrude (extrude OFF, cooling ON)
            sequence.Add(new PathPointCommand(
                modifiedEndPoint, 1.2, coolingOn: true, extrudeOn: false, heatOn: false));

            // Path end → stopCooling (everything OFF)
            sequence.Add(new PathPointCommand(
                pathEnd, 1.2, coolingOn: false, extrudeOn: false, heatOn: false));

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
            double startExtruding = 2.5;
            double startCooling = 0.1;
            double slowExtruding = 4.0;

            Point3d startExtruding_pt = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + startExtruding);
            Point3d slowExtruding_pt = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z - slowExtruding);

            LineCurve modifiedPath = new LineCurve(startExtruding_pt, pathEnd);
            modifiedPath.LengthParameter(startCooling, out double startCoolingParam);
            Point3d startCooling_pt = modifiedPath.PointAt(startCoolingParam);

            // Build sequence:

            sequence.Add(new PathPointCommand(startExtruding_pt, 1.8, coolingOn: false, extrudeOn: true, heatOn: false));
            //sequence.Add(new PathPointCommand(pathStart, 1.8, coolingOn: false, extrudeOn: true, heatOn: false));
            //sequence.Add(new PathPointCommand(startCooling_pt, 1.8, coolingOn: true, extrudeOn: true, heatOn: false));
            //sequence.Add(new PathPointCommand(slowExtruding_pt, 1.8, coolingOn: true, extrudeOn: false, heatOn: false));
            sequence.Add(new PathPointCommand(pathEnd, 1.8, coolingOn: false, extrudeOn: false, heatOn: false));

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
                startExtruding_pt, 2.4, coolingOn: false, extrudeOn: true, heatOn: false));

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
                pathEnd, 2.4, coolingOn: false, extrudeOn: false, heatOn: false));

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

            // move Z + 10
            Point3d pathEndMoveZ = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 10);
            Curve curveNewZ = new LineCurve(pathStart, pathEndMoveZ);

            // Dummy plane division — you should replace with your actual DivideCurveIntoPlanes logic
            int numPlanes = 10;
            List<Point3d> pathZModifiedPoints = DivideCurveIntoPoints(curveNewZ, numPlanes);
            List<Point3d> crvPathPoints = DivideCurveIntoPoints(curve, numPlanes);

            // If Z > threshold
            if (pointOnCurve.Z > ZHeightTest)
            {
                // Start point → Cooling ON, Extrude ON, E5 = 2.0
                sequence.Add(new PathPointCommand(
                    pathStart, 2.0, coolingOn: true, extrudeOn: true, heatOn: false));


                // pathEnd → stopExtrude
                sequence.Add(new PathPointCommand(
                    pathEnd, 2.0, coolingOn: true, extrudeOn: false, heatOn: false));
            }
            else // Z <= threshold
            {
                // Start point → Cooling OFF, Extrude ON, E5 = 1.6
                sequence.Add(new PathPointCommand(
                    pathStart, 1.6, coolingOn: false, extrudeOn: true, heatOn: false));


                // pathEnd → stopExtrude
                sequence.Add(new PathPointCommand(
                    pathEnd, 2.0, coolingOn: false, extrudeOn: false, heatOn: false));
            }

            return sequence;
        }

        // Helper: divide curve into points — replace with your DivideCurveIntoPlanes
        private List<Point3d> DivideCurveIntoPoints(Curve curve, int numDivisions)
        {
            List<Point3d> pts = new List<Point3d>();
            double length = curve.GetLength();
            double step = length / (numDivisions - 1);

            for (int i = 0; i < numDivisions; i++)
            {
                double param;
                curve.LengthParameter(i * step, out param);
                pts.Add(curve.PointAt(param));
            }

            return pts;
        }
    }
}