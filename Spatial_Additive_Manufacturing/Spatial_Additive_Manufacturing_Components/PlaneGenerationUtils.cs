using Rhino;
using Rhino.Geometry;
using System;

public static class PlaneGenerationUtils
{
    public const double DefaultRootFacingRollOffsetDegrees = 7.5;
    public const double DefaultXYPlaneYawOffsetDegrees = 0.0;
    public const double DefaultMaxXYDeviationDegrees = 30.0;
    public const double DefaultMaxRootFacingDeviationDegrees = DefaultMaxXYDeviationDegrees;
    public const double DefaultMaxToolTiltDegrees = 65.0;

    public static double RootFacingRollOffsetDegrees { get; set; } = DefaultRootFacingRollOffsetDegrees;
    public static double XYPlaneYawOffsetDegrees { get; set; } = DefaultXYPlaneYawOffsetDegrees;
    public static Plane RootReferencePlane { get; set; } = Plane.WorldXY;
    public static double MaxXYDeviationDegrees { get; set; } = DefaultMaxXYDeviationDegrees;
    public static double MaxRootFacingDeviationDegrees
    {
        get => MaxXYDeviationDegrees;
        set => MaxXYDeviationDegrees = value;
    }
    public static double MaxToolTiltDegrees { get; set; } = DefaultMaxToolTiltDegrees;

    public static Vector3d ComputeYAxisClosestToRoot(Point3d pointOnCurve, Vector3d tangent)
    {
        Plane rootPlane = GetValidRootReferencePlane();
        Vector3d toOrigin = rootPlane.Origin - pointOnCurve;

        Vector3d tangentDir = tangent;
        tangentDir.Unitize();

        Vector3d yAxis = toOrigin - (Vector3d.Multiply(toOrigin * tangentDir, tangentDir));
        yAxis.Unitize();

        if (yAxis.IsZero)
        {
            yAxis = tangentDir;
        }

        return yAxis;
    }

    public static Vector3d ComputeSafeRootFacingYAxis(Point3d pointOnCurve, Vector3d zAxis)
    {
        Vector3d zAxisUnit = zAxis;
        if (!zAxisUnit.Unitize())
        {
            zAxisUnit = GetValidRootReferencePlane().ZAxis;
        }

        Vector3d yAxis = ComputeProjectedRootFacingYAxis(pointOnCurve, zAxisUnit);
        RotateVectorAroundAxis(ref yAxis, zAxisUnit, RootFacingRollOffsetDegrees, pointOnCurve);
        if (!yAxis.Unitize())
        {
            yAxis = ComputeProjectedRootFacingYAxis(pointOnCurve, zAxisUnit);
        }

        return yAxis;
    }

    public static Plane ConstrainPlane(Plane plane)
    {
        Plane rootPlane = GetValidRootReferencePlane();
        Vector3d baseZAxis = -rootPlane.ZAxis;
        if (!baseZAxis.Unitize())
        {
            baseZAxis = -Vector3d.ZAxis;
        }

        Vector3d zAxis = plane.ZAxis;
        if (!zAxis.Unitize())
        {
            zAxis = baseZAxis;
        }

        double maxXYDeviationDegrees = ClampDegrees(MaxXYDeviationDegrees);
        double maxToolTiltDegrees = ClampDegrees(MaxToolTiltDegrees);
        zAxis = ClampDirectionToCone(baseZAxis, zAxis, Math.Min(maxXYDeviationDegrees, maxToolTiltDegrees), rootPlane.XAxis);

        Vector3d xyReferenceYAxis = ComputeXYReferenceYAxis(zAxis);
        Vector3d yAxis = plane.YAxis - (Vector3d.Multiply(plane.YAxis, zAxis) * zAxis);
        if (!yAxis.Unitize())
        {
            yAxis = xyReferenceYAxis;
        }

        yAxis = ClampDirectionAroundAxis(xyReferenceYAxis, yAxis, zAxis, maxXYDeviationDegrees);

        Vector3d xAxis = Vector3d.CrossProduct(yAxis, zAxis);
        if (!xAxis.Unitize())
        {
            xAxis = rootPlane.XAxis - (Vector3d.Multiply(rootPlane.XAxis, zAxis) * zAxis);
            if (!xAxis.Unitize())
            {
                xAxis = Vector3d.XAxis;
            }
        }

        yAxis = Vector3d.CrossProduct(zAxis, xAxis);
        yAxis.Unitize();

        if (Math.Abs(XYPlaneYawOffsetDegrees) > RhinoMath.SqrtEpsilon)
        {
            RotateVectorAroundAxis(ref xAxis, zAxis, XYPlaneYawOffsetDegrees, plane.Origin);
            RotateVectorAroundAxis(ref yAxis, zAxis, XYPlaneYawOffsetDegrees, plane.Origin);
            xAxis.Unitize();
            yAxis.Unitize();
        }

        return new Plane(plane.Origin, xAxis, yAxis);
    }

    public static Vector3d ComputeXYReferenceYAxis(Vector3d zAxis)
    {
        Plane rootPlane = GetValidRootReferencePlane();
        Vector3d zAxisUnit = zAxis;
        if (!zAxisUnit.Unitize())
        {
            zAxisUnit = -rootPlane.ZAxis;
        }

        Vector3d xAxis = rootPlane.XAxis - (Vector3d.Multiply(rootPlane.XAxis, zAxisUnit) * zAxisUnit);
        if (!xAxis.Unitize())
        {
            xAxis = rootPlane.YAxis - (Vector3d.Multiply(rootPlane.YAxis, zAxisUnit) * zAxisUnit);
            if (!xAxis.Unitize())
            {
                xAxis = Vector3d.XAxis;
            }
        }

        Vector3d yAxis = Vector3d.CrossProduct(zAxisUnit, xAxis);
        if (!yAxis.Unitize())
        {
            yAxis = rootPlane.YAxis - (Vector3d.Multiply(rootPlane.YAxis, zAxisUnit) * zAxisUnit);
            yAxis.Unitize();
        }

        return yAxis;
    }

    public static Plane GetValidRootReferencePlane()
    {
        Plane plane = RootReferencePlane;
        Vector3d xAxis = plane.XAxis;
        Vector3d yAxis = plane.YAxis;

        if (!xAxis.Unitize())
        {
            xAxis = Vector3d.XAxis;
        }

        if (!yAxis.Unitize())
        {
            yAxis = Vector3d.YAxis;
        }

        Vector3d zAxis = Vector3d.CrossProduct(xAxis, yAxis);
        if (!zAxis.Unitize())
        {
            return Plane.WorldXY;
        }

        yAxis = Vector3d.CrossProduct(zAxis, xAxis);
        yAxis.Unitize();

        return new Plane(plane.Origin, xAxis, yAxis);
    }

    public static void ApplyRootFacingRollOffset(ref Vector3d xAxis, ref Vector3d yAxis, Vector3d zAxis, Point3d origin)
    {
        RotateVectorAroundAxis(ref xAxis, zAxis, RootFacingRollOffsetDegrees, origin);
        RotateVectorAroundAxis(ref yAxis, zAxis, RootFacingRollOffsetDegrees, origin);
        xAxis.Unitize();
        yAxis.Unitize();
    }

    private static Vector3d ComputeProjectedRootFacingYAxis(Point3d pointOnCurve, Vector3d zAxis)
    {
        Plane rootPlane = GetValidRootReferencePlane();
        Vector3d toOrigin = rootPlane.Origin - pointOnCurve;

        Vector3d yAxis = toOrigin - (Vector3d.Multiply(toOrigin, zAxis) * zAxis);
        if (!yAxis.Unitize())
        {
            yAxis = rootPlane.YAxis - (Vector3d.Multiply(rootPlane.YAxis, zAxis) * zAxis);
            if (!yAxis.Unitize())
            {
                yAxis = rootPlane.XAxis - (Vector3d.Multiply(rootPlane.XAxis, zAxis) * zAxis);
                yAxis.Unitize();
            }
        }

        return yAxis;
    }

    private static void RotateVectorAroundAxis(ref Vector3d vector, Vector3d axis, double degrees, Point3d origin)
    {
        if (Math.Abs(degrees) < RhinoMath.SqrtEpsilon)
        {
            return;
        }

        Vector3d unitAxis = axis;
        if (!unitAxis.Unitize())
        {
            return;
        }

        Transform rotation = Transform.Rotation(RhinoMath.ToRadians(degrees), unitAxis, origin);
        vector.Transform(rotation);
    }

    private static Vector3d ClampDirectionToCone(Vector3d center, Vector3d direction, double maxDegrees, Vector3d fallbackAxis)
    {
        double clampedMaxDegrees = ClampDegrees(maxDegrees);
        if (clampedMaxDegrees >= 179.999)
        {
            return direction;
        }

        Vector3d centerUnit = center;
        Vector3d directionUnit = direction;
        if (!centerUnit.Unitize() || !directionUnit.Unitize())
        {
            return direction;
        }

        double angle = RhinoMath.ToDegrees(Vector3d.VectorAngle(centerUnit, directionUnit));
        if (angle <= clampedMaxDegrees)
        {
            return directionUnit;
        }

        Vector3d rotationAxis = Vector3d.CrossProduct(centerUnit, directionUnit);
        if (!rotationAxis.Unitize())
        {
            rotationAxis = fallbackAxis - (Vector3d.Multiply(fallbackAxis, centerUnit) * centerUnit);
            if (!rotationAxis.Unitize())
            {
                rotationAxis = Vector3d.YAxis;
            }
        }

        Vector3d result = centerUnit;
        Transform rotation = Transform.Rotation(RhinoMath.ToRadians(clampedMaxDegrees), rotationAxis, Point3d.Origin);
        result.Transform(rotation);
        result.Unitize();
        return result;
    }

    private static Vector3d ClampDirectionAroundAxis(Vector3d center, Vector3d direction, Vector3d axis, double maxDegrees)
    {
        double clampedMaxDegrees = ClampDegrees(maxDegrees);
        if (clampedMaxDegrees >= 179.999)
        {
            return direction;
        }

        Vector3d centerUnit = center;
        Vector3d directionUnit = direction;
        Vector3d axisUnit = axis;
        if (!centerUnit.Unitize() || !directionUnit.Unitize() || !axisUnit.Unitize())
        {
            return direction;
        }

        double angle = RhinoMath.ToDegrees(Vector3d.VectorAngle(centerUnit, directionUnit));
        if (angle <= clampedMaxDegrees)
        {
            return directionUnit;
        }

        double side = Vector3d.Multiply(Vector3d.CrossProduct(centerUnit, directionUnit), axisUnit) >= 0.0
            ? 1.0
            : -1.0;

        Vector3d result = centerUnit;
        Transform rotation = Transform.Rotation(RhinoMath.ToRadians(clampedMaxDegrees * side), axisUnit, Point3d.Origin);
        result.Transform(rotation);
        result.Unitize();
        return result;
    }

    private static double ClampDegrees(double degrees)
    {
        if (double.IsNaN(degrees) || double.IsInfinity(degrees))
        {
            return 180.0;
        }

        return Math.Max(0.0, Math.Min(180.0, Math.Abs(degrees)));
    }
}
