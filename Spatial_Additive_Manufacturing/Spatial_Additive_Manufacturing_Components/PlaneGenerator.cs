using Rhino;
using Rhino.Geometry;
using System;


public class AngledUpPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, Vector3d? optionalTangent = null)
    {
        Vector3d tangent = optionalTangent ?? pathCurve.Line.Direction;
        tangent.Unitize();

        Vector3d zAxis = -Vector3d.ZAxis;

        Vector3d yAxis = PlaneGenerationUtils.ComputeYAxisClosestToRoot(referencePoint, tangent);

        Vector3d xAxis = Vector3d.CrossProduct(yAxis, zAxis);
        xAxis.Unitize();

        return new Plane(referencePoint, xAxis, yAxis);
    }
}

public class AngledDownPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, Vector3d? optionalTangent = null)
    {
        // Use tangent at midpoint if not provided
        Vector3d yAxis = optionalTangent ?? pathCurve.Tangent;
        yAxis.Unitize();

        // Projected angle correction logic
        Point3d pathStart = pathCurve.StartPoint;
        Point3d pathEnd = pathCurve.EndPoint;

        Point3d projectedEndPt = new Point3d(pathStart.X, pathStart.Y, pathEnd.Z);
        Vector3d crvVec = pathStart - pathEnd;
        Vector3d projectedCrvVec = projectedEndPt - pathEnd;

        double angleToWorldZ = Vector3d.VectorAngle(crvVec, projectedCrvVec);
        angleToWorldZ = RhinoMath.ToDegrees(angleToWorldZ);

        if (angleToWorldZ > 45.0)
        {
            double rotationAngle = angleToWorldZ - 35;
            Transform rotation = Transform.Rotation(rotationAngle * (Math.PI / 180.0), Vector3d.CrossProduct(yAxis, Vector3d.ZAxis), pathStart);
            yAxis.Transform(rotation);
        }

        // X and Y orientation cleanup
        Vector3d xAxis = Vector3d.CrossProduct(Vector3d.ZAxis, yAxis);
        xAxis.Unitize();
        xAxis = -xAxis;
        yAxis = -yAxis;

        if (xAxis.IsZero)
        {
            xAxis = Vector3d.CrossProduct(Vector3d.YAxis, yAxis);
            xAxis.Unitize();
        }

        double xAxisDif = Vector3d.VectorAngle(xAxis, Vector3d.XAxis) * (180.0 / Math.PI);
        if (Math.Abs(xAxisDif) < 90)
        {
            xAxis = -xAxis;
            yAxis = -yAxis;
        }

        Vector3d zAxis = Vector3d.CrossProduct(yAxis, xAxis);
        zAxis.Unitize();

        // Final Plane
        return new Plane(referencePoint, xAxis, yAxis);
    }
}

    public class VerticalPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, Vector3d? optionalTangent = null)
    {
        Vector3d zAxis = -Vector3d.ZAxis;

        Vector3d toOrigin = new Vector3d(-referencePoint.X, -referencePoint.Y, -referencePoint.Z);
        Vector3d yAxis = toOrigin;
        yAxis.Z = 0;
        yAxis.Unitize();

        if (yAxis.IsZero)
        {
            yAxis = Vector3d.YAxis;
        }

        Vector3d xAxis = Vector3d.CrossProduct(yAxis, zAxis);
        xAxis.Unitize();

        return new Plane(referencePoint, xAxis, yAxis);
    }
}

public class HorizontalPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, Vector3d? optionalTangent = null)
    {
        Vector3d tangent = optionalTangent ?? pathCurve.Line.Direction;
        tangent.Unitize();

        Vector3d zAxis = -Vector3d.ZAxis;

        Vector3d yAxis = tangent;

        Vector3d xAxis = Vector3d.CrossProduct(yAxis, zAxis);
        xAxis.Unitize();

        return new Plane(referencePoint, xAxis, yAxis);
    }
}





