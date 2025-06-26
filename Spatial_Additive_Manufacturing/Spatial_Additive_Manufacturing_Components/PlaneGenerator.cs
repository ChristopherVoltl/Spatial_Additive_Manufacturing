using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;


public class AngledUpPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, out double xAxisDif, out double yAxisDif, Vector3d? optionalTangent = null)
    {
        Vector3d tangent = optionalTangent ?? pathCurve.Line.Direction;
        tangent.Unitize();

        Vector3d zAxis = -Vector3d.ZAxis;

        Vector3d yAxis = PlaneGenerationUtils.ComputeYAxisClosestToRoot(referencePoint, tangent);

        Vector3d xAxis = Vector3d.CrossProduct(yAxis, zAxis);
        xAxis.Unitize();

        xAxisDif = Vector3d.VectorAngle(xAxis, Vector3d.XAxis) * (180.0 / Math.PI);
        yAxisDif = Vector3d.VectorAngle(yAxis, Vector3d.YAxis) * (180.0 / Math.PI);


        return new Plane(referencePoint, xAxis, yAxis);
    }
}

public class AngledDownPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, out double xAxisDif, out double yAxisDif, Vector3d? optionalTangent = null)
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
        if (pathCurve.Line.Length <= 80.0)
        {
            IPlaneGenerator generator = new VerticalPlaneGenerator();
            Plane pathPlane = generator.GeneratePlane(pathCurve, referencePoint, out double xAxisDif_pathPlane, out double yAxisDif_pathPlane);

            xAxisDif = xAxisDif_pathPlane;
            yAxisDif = yAxisDif_pathPlane;

            return pathPlane;
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

        // Now calculate signed diffs:
        xAxisDif = SignedAngle(Vector3d.XAxis, xAxis, Vector3d.ZAxis);
        yAxisDif = SignedAngle(Vector3d.YAxis, yAxis, Vector3d.ZAxis);

        //  flip plane logic:
        if (xAxisDif <= 90 && xAxisDif >= 45 || xAxisDif == 0)
        {
            xAxis = -xAxis;
            yAxis = -yAxis;

            // Recompute diffs after flip:
            //xAxisDif = SignedAngle(Vector3d.XAxis, xAxis, Vector3d.ZAxis);
            //yAxisDif = SignedAngle(Vector3d.YAxis, yAxis, Vector3d.ZAxis);
        }

        Vector3d zAxis = Vector3d.CrossProduct(yAxis, xAxis);
        zAxis.Unitize();

        // Final Plane
        return new Plane(referencePoint, xAxis, yAxis);
    }

    // Helper function for signed angle:
    private static double SignedAngle(Vector3d v1, Vector3d v2, Vector3d aroundAxis)
    {
        Vector3d cross = Vector3d.CrossProduct(v1, v2);
        double dot = Vector3d.Multiply(cross, aroundAxis);

        double angle = Vector3d.VectorAngle(v1, v2);
        angle *= (dot >= 0.0) ? 1.0 : -1.0;

        return RhinoMath.ToDegrees(angle);
    }
}

public class VerticalPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, out double xAxisDif, out double yAxisDif, Vector3d? optionalTangent = null)
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

        xAxisDif = Vector3d.VectorAngle(xAxis, Vector3d.XAxis) * (180.0 / Math.PI);
        yAxisDif = Vector3d.VectorAngle(yAxis, Vector3d.YAxis) * (180.0 / Math.PI);


        return new Plane(referencePoint, xAxis, yAxis);
    }
}

public class HorizontalPlaneGenerator : IPlaneGenerator
{
    public Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, out double xAxisDif, out double yAxisDif, Vector3d? optionalTangent = null)
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

        xAxisDif = Vector3d.VectorAngle(xAxis, Vector3d.XAxis) * (180.0 / Math.PI);
        yAxisDif = Vector3d.VectorAngle(yAxis, Vector3d.YAxis) * (180.0 / Math.PI);


        return new Plane(referencePoint, xAxis, yAxis);
    }
}







