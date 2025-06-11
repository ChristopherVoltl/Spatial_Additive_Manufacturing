using Rhino.Geometry;

public interface IPlaneGenerator
{
    Plane GeneratePlane(PathCurve pathCurve, Point3d referencePoint, Vector3d? optionalTangent = null);
}