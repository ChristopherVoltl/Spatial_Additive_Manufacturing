using Rhino.Geometry;
using System.Collections.Generic;

public interface IInnerPointStrategy
{
    List<Point3d> GetInnerPoints(PathCurve pathCurve);
}

public static class InnerPointStrategyFactory
{
    public static IInnerPointStrategy GetStrategy(PathCurve pathCurve)
    {
        switch (pathCurve.Orientation)
        {
            case PathCurve.OrientationType.Horizontal:
                return new UniformSpacingStrategy(5);

            case PathCurve.OrientationType.AngledUp:
                return new MidPointOnlyStrategy();

            case PathCurve.OrientationType.AngledDown:
                return new UniformSpacingStrategy(4);

            case PathCurve.OrientationType.Vertical:
                return new NoInnerPointsStrategy();

            default:
                return new UniformSpacingStrategy(pathCurve.PointCount);
        }
    }

    // Strategies defined here as nested classes:

    private class UniformSpacingStrategy : IInnerPointStrategy
    {
        private int numPoints;

        public UniformSpacingStrategy(int numPoints)
        {
            this.numPoints = numPoints;
        }

        public List<Point3d> GetInnerPoints(PathCurve pathCurve)
        {
            List<Point3d> points = new();

            for (int i = 1; i < numPoints - 1; i++)
            {
                double t = (double)i / (numPoints - 1);
                points.Add(pathCurve.Line.PointAt(t));
            }

            return points;
        }
    }

    private class MidPointOnlyStrategy : IInnerPointStrategy
    {
        public List<Point3d> GetInnerPoints(PathCurve pathCurve)
        {
            return new List<Point3d> { pathCurve.MidPoint };
        }
    }

    private class NoInnerPointsStrategy : IInnerPointStrategy
    {
        public List<Point3d> GetInnerPoints(PathCurve pathCurve)
        {
            return new List<Point3d>();
        }
    }
}