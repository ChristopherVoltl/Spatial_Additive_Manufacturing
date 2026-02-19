using System.Collections.Generic;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;
using System;
using System.Data;

public class ToolpathPlaneConduit : DisplayConduit
{
    private readonly List<Plane> planes;
    private readonly List<double> e5Values;
    private readonly double axisSize;
    private readonly bool showPlaneIndex;
    private readonly bool useE5Gradient;
    private readonly List<double> xAxisDifValues;
    private readonly List<double> yAxisDifValues;
    private readonly List<double> planeRotationAngles;


    public ToolpathPlaneConduit(
        List<Plane> planes,
        List<double> e5Values,
        List<double> xAxisDifValues = null,
        List<double> yAxisDifValues = null,
        List<double> planeRotationAngles = null,
        double axisSize = 5.0,
        bool showPlaneIndex = true,
        bool useE5Gradient = true)
    {
        this.planes = planes;
        this.e5Values = e5Values;
        this.xAxisDifValues = xAxisDifValues;
        this.yAxisDifValues = yAxisDifValues;
        this.planeRotationAngles = planeRotationAngles;
        this.axisSize = axisSize;
        this.showPlaneIndex = showPlaneIndex;
        this.useE5Gradient = useE5Gradient;
    }

    protected override void DrawForeground(DrawEventArgs e)
    {
        if (planes == null || planes.Count == 0)
            return;

        // Determine E5 value range for gradient (if enabled)
        double minE5 = 0.0;
        double maxE5 = 1.0;

        if (useE5Gradient && e5Values.Count > 0)
        {
            minE5 = double.MaxValue;
            maxE5 = double.MinValue;
            foreach (double v in e5Values)
            {
                if (v < minE5) minE5 = v;
                if (v > maxE5) maxE5 = v;
            }

            // Protect against division by zero
            if (Math.Abs(maxE5 - minE5) < 1e-6)
            {
                maxE5 = minE5 + 1.0;
            }
        }

        for (int i = 0; i < planes.Count; i++)
        {
            Plane plane = planes[i];
            double e5 = (i < e5Values.Count) ? e5Values[i] : 0.0;

            // X axis - Red
            e.Display.DrawLine(new Line(plane.Origin, plane.Origin + plane.XAxis * axisSize), Color.Red, 2);

            // Y axis - Green
            e.Display.DrawLine(new Line(plane.Origin, plane.Origin + plane.YAxis * axisSize), Color.Green, 2);

            // Z axis - Blue
            e.Display.DrawLine(new Line(plane.Origin, plane.Origin + plane.ZAxis * axisSize), Color.Blue, 2);

            // Draw E5 text with optional color gradient
            //Color e5Color = Color.White;
            double t = (e5 - minE5) / (maxE5 - minE5); // Normalize 0-1
            Color e5Color = InterpolateColor(Color.LightBlue, Color.Red, t);

            if (useE5Gradient)
            {
                double te5 = (e5 - minE5) / (maxE5 - minE5); // Normalize 0-1
                e5Color = InterpolateColor(Color.LightBlue, Color.Red, te5);
            }

            string e5Text = $"E5: {e5:F2}";
            e.Display.Draw2dText(e5Text, e5Color, plane.Origin, true, 36);
            //e.Display.DrawDot(plane.Origin, e5Text, Color.Black, e5Color);

            // Draw plane index (optional)
            if (showPlaneIndex)
            {
                Point3d indexPt = plane.Origin + plane.ZAxis * (axisSize * 0.5);
                string indexText = $"Plane {i}";
                e.Display.Draw2dText(indexText, Color.Yellow, indexPt, true, 10);
                
            }

            // xAxisDif value
            if (xAxisDifValues != null && i < xAxisDifValues.Count)
            {
                double xAxisDif = xAxisDifValues[i];
                string xAxisDifText = $"xAxisDif: {xAxisDif:F1}°";

                Point3d offsetPt = plane.Origin + plane.YAxis * (axisSize * 0.5);
                e.Display.DrawDot(offsetPt, xAxisDifText, Color.Black, Color.LightGreen);
            }

            // yAxisDif value
            if (yAxisDifValues != null && i < yAxisDifValues.Count)
            {
                double yAxisDif = yAxisDifValues[i];
                string yAxisDifText = $"yAxisDif: {yAxisDif:F1}°";

                Point3d offsetPt = plane.Origin + plane.ZAxis * (axisSize * 0.5);
                e.Display.DrawDot(offsetPt, yAxisDifText, Color.Black, Color.Cyan);
            }
            // PlaneRotationAngle value
            if (planeRotationAngles != null && i < planeRotationAngles.Count)
            {
                double planeAngle = planeRotationAngles[i];
                string planeAngleText = $"RotAngle: {planeAngle:F1}°";

                Point3d offsetPt = plane.Origin + plane.XAxis * (axisSize * 0.5);

                // RED if reflection detected (angle < 0), otherwise ORANGE
                Color angleColor = (planeAngle < 0) ? Color.Red : Color.Orange;

                e.Display.DrawDot(offsetPt, planeAngleText, Color.Black, angleColor);
            }

        }
    }

    // Helper function: linear color interpolation
    private static Color InterpolateColor(Color start, Color end, double t)
    {
        t = Math.Max(0.0, Math.Min(1.0, t)); // Clamp to [0,1]

        int r = (int)(start.R + (end.R - start.R) * t);
        int g = (int)(start.G + (end.G - start.G) * t);
        int b = (int)(start.B + (end.B - start.B) * t);

        return Color.FromArgb(r, g, b);
    }

    //PlaneRotationAngle helper
    public static double GetPlaneRotationAngle(Plane referencePlane, Plane targetPlane)
    {
        Transform transform = Transform.PlaneToPlane(referencePlane, targetPlane);

        // Compute rotation angle from trace
        double trace = transform.M00 + transform.M11 + transform.M22;
        double angleRadians = Math.Acos((trace - 1.0) / 2.0);
        angleRadians = Math.Max(0.0, Math.Min(Math.PI, angleRadians));
        double angleDegrees = angleRadians * (180.0 / Math.PI);

        // Compute determinant → detects reflection
        double determinant =
            transform.M00 * (transform.M11 * transform.M22 - transform.M12 * transform.M21)
          - transform.M01 * (transform.M10 * transform.M22 - transform.M12 * transform.M20)
          + transform.M02 * (transform.M10 * transform.M21 - transform.M11 * transform.M20);

        bool isReflection = (determinant < 0);

        // If reflection → return NEGATIVE angle
        if (isReflection)
        {
            angleDegrees = -angleDegrees;
        }

        return angleDegrees;
    }

    public static (double xAxisDif, double yAxisDif) GetPlaneAxisDifferences(Plane referencePlane, Plane targetPlane)
    {
        double xAxisDif = Vector3d.VectorAngle(targetPlane.XAxis, referencePlane.XAxis) * (180.0 / Math.PI);
        double yAxisDif = Vector3d.VectorAngle(targetPlane.YAxis, referencePlane.YAxis) * (180.0 / Math.PI);

        return (xAxisDif, yAxisDif);
    }
}

