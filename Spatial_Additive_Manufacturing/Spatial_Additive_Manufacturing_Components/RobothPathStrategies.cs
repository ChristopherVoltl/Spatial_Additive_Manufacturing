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
            Point3d pullbackPointvert = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + 6.0);

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
            float velRatio = 0.2f;


            // Pullback point → extrude ON
            sequence.Add(new PathPointCommand(
                pullbackPoint, 1.1, coolingOn: false, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path start → extrude ON
            sequence.Add(new PathPointCommand(
                pullbackPointvert, 1.2, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path start → cool ON (extrude stays ON → up to you to decide if it should flip)
            sequence.Add(new PathPointCommand(
                pullbackPointvert, 1.2, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Modified end → stopExtrude (extrude OFF, cooling ON)
            sequence.Add(new PathPointCommand(
                modifiedEndPoint, 2.4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));

            // Path end → stopCooling (everything OFF)
            sequence.Add(new PathPointCommand(
                pathEnd, 2.4, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: velRatio, cycleWait: false));

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
            float velRatio = 0.2f;

            Point3d startExtruding_pt = new Point3d(pathStart.X, pathStart.Y, pathStart.Z + startExtruding);
            Point3d slowExtruding_pt = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z - slowExtruding);

            LineCurve modifiedPath = new LineCurve(startExtruding_pt, pathEnd);
            modifiedPath.LengthParameter(startCooling, out double startCoolingParam);
            Point3d startCooling_pt = modifiedPath.PointAt(startCoolingParam);

            // Build sequence:

            sequence.Add(new PathPointCommand(startExtruding_pt, e5*1.4, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: velRatio, cycleWait: false));
            sequence.Add(new PathPointCommand(startCooling_pt, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
            sequence.Add(new PathPointCommand(slowExtruding_pt, e5, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio, cycleWait: false));
            sequence.Add(new PathPointCommand(pathEnd, e5/4, coolingOn: true, extrudeOn: true, heatOn: true, velRatio: velRatio/2, cycleWait: true));

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

            // move Z + 10
            Point3d pathEndMoveZ = new Point3d(pathEnd.X, pathEnd.Y, pathEnd.Z + 10);
            Curve curveNewZ = new LineCurve(pathStart, pathEndMoveZ);


            // If Z > threshold
            if (pointOnCurve.Z > ZHeightTest)
            {
                // Start point → Cooling ON, Extrude ON, E5 = 2.0
                sequence.Add(new PathPointCommand(
                    pathStart, 2.0, coolingOn: true, extrudeOn: true, heatOn: false, velRatio: 0.05f, cycleWait: false));


                // pathEnd → stopExtrude
                sequence.Add(new PathPointCommand(
                    pathEnd, 2.0, coolingOn: true, extrudeOn: false, heatOn: false, velRatio: 0.05f, cycleWait: false));
            }
            else // Z <= threshold
            {
                // Start point → Cooling OFF, Extrude ON, E5 = 1.6
                sequence.Add(new PathPointCommand(
                    pathStart, 1.6, coolingOn: false, extrudeOn: true, heatOn: false, velRatio: 0.05f, cycleWait: false));


                // pathEnd → stopExtrude
                sequence.Add(new PathPointCommand(
                    pathEnd, 2.0, coolingOn: false, extrudeOn: false, heatOn: false, velRatio: 0.05f, cycleWait: false));
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