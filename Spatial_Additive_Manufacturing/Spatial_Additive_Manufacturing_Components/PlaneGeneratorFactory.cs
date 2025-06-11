public static class PlaneGeneratorFactory
{
    public static IPlaneGenerator GetGenerator(PathCurve.OrientationType orientation)
    {
        switch (orientation)
        {
            case PathCurve.OrientationType.AngledUp:
                return new AngledUpPlaneGenerator();
            case PathCurve.OrientationType.AngledDown:
                return new AngledDownPlaneGenerator();
            case PathCurve.OrientationType.Vertical:
                return new VerticalPlaneGenerator();
            case PathCurve.OrientationType.Horizontal:
                return new HorizontalPlaneGenerator();
            default:
                throw new System.InvalidOperationException($"Unsupported orientation: {orientation}");
        }
    }
}
