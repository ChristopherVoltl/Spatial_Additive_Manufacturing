// PathCurve.cs
using System;
using System.Collections.Generic;
using Rhino.Geometry;

public class PathCurve
{
    public Guid Id { get; }
    public Line Line { get; }

    public Point3d preExtrusion => new Point3d(Line.From.X, Line.From.Y, Line.From.Z + 6.0);
    public Point3d StartPoint => Line.From;
    public Point3d EndPoint => Line.To;
    public Point3d MidPoint => Line.PointAt(0.5);
    public double MidpointZ => MidPoint.Z;
    public Vector3d Tangent => Line.Direction;
    public OrientationType Orientation { get; }

    public int PointCount { get; private set; }

    public List<Guid> StartConnections { get; set; } = new List<Guid>();
    public List<Guid> EndConnections { get; set; } = new List<Guid>();

    public PathCurve(Line line, double tolerance)
    {
        Id = Guid.NewGuid();
        Line = line;
        Orientation = ComputeOrientation(tolerance);
        PointCount = ComputePointCount(Orientation);
    }

    private int ComputePointCount(OrientationType orientation)
    {
        if (Orientation == OrientationType.Vertical)
        {
            return 5;
        }
        else if (Orientation == OrientationType.AngledUp)
        {
            return 6;
        }
        else if (Orientation == OrientationType.AngledDown)
        {
            return 6;
        }
        else
        {
            return 2;
        }
    }

    private OrientationType ComputeOrientation(double tolerance)
    {
        Vector3d dir = Line.Direction;
        dir.Unitize();

        if (Math.Abs(dir.X) < tolerance && Math.Abs(dir.Y) < tolerance && Math.Abs(dir.Z) > tolerance)
            return OrientationType.Vertical;

        if (Math.Abs(dir.Z) < tolerance)
            return OrientationType.Horizontal;

        if (dir.Z > 0)
            return OrientationType.AngledUp;
        else
            return OrientationType.AngledDown;

    }

    public enum OrientationType
    {
        Vertical,
        Horizontal,
        AngledUp,
        AngledDown
    }

    public interface IPlaneGenerator
    {
        Plane GeneratePlane(PathCurve pathCurve, double t);
    }
}

