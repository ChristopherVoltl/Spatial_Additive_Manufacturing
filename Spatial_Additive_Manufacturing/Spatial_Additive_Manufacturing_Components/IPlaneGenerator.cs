using Rhino.Geometry;

public interface IPlaneGenerator
{
    Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, out double xAxisDif, out double yAxisDif, Vector3d? optionalTangent = null);
}