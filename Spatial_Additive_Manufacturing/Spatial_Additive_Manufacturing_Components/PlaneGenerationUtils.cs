using Rhino.Geometry;
using System;

public static class PlaneGenerationUtils
{
    public static Vector3d ComputeYAxisClosestToRoot(Point3d pointOnCurve, Vector3d tangent)
    {
        Vector3d toOrigin = new Vector3d(-pointOnCurve.X, -pointOnCurve.Y, -pointOnCurve.Z);

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
}


