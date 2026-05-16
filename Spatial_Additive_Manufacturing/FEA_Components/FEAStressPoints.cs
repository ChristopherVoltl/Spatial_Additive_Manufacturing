using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Display;
using Rhino.Geometry;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class FEAStressPoints : GH_Component
    {
        private readonly List<Point3d> previewPoints = new List<Point3d>();
        private readonly List<Color> previewColors = new List<Color>();
        private BoundingBox previewBox = BoundingBox.Empty;
        private int previewPointSize = 5;

        public FEAStressPoints()
          : base("FEA Stress Points", "FEAStressPts",
              "Imports ANSYS equivalent stress points, converts meter coordinates to Rhino units, and maps stress values to adjustable point colors.",
              "FGAM", "FEA")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("CSV Path", "CSV", "Optional ANSYS CSV file path with NodeID, X, Y, Z, EquivalentStress columns.", GH_ParamAccess.item);
            pManager[0].Optional = true;

            pManager.AddPointParameter("Points", "P", "Point locations to color when no CSV path is provided.", GH_ParamAccess.list);
            pManager[1].Optional = true;

            pManager.AddNumberParameter("Stress Values", "S", "Stress value for each point when no CSV path is provided.", GH_ParamAccess.list);
            pManager[2].Optional = true;

            pManager.AddNumberParameter("Minimum Stress", "Min", "Stress value mapped to the low color. Leave empty for automatic minimum.", GH_ParamAccess.item);
            pManager[3].Optional = true;

            pManager.AddNumberParameter("Maximum Stress", "Max", "Stress value mapped to the high color. Leave empty for automatic maximum.", GH_ParamAccess.item);
            pManager[4].Optional = true;

            pManager.AddColourParameter("Low Color", "Low", "Color used for the minimum stress value.", GH_ParamAccess.item, Color.FromArgb(0, 105, 255));
            pManager.AddColourParameter("High Color", "High", "Color used for the maximum stress value.", GH_ParamAccess.item, Color.FromArgb(255, 35, 0));
            pManager.AddIntegerParameter("Point Size", "Size", "Viewport preview point size.", GH_ParamAccess.item, 5);
            pManager.AddBooleanParameter("Coordinates In Meters", "Meters", "Convert input point coordinates from meters to the current Rhino document units.", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Rhino point locations.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Stress Values", "S", "Equivalent stress values for each point.", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Color mapped from each stress value.", GH_ParamAccess.list);
            pManager.AddGenericParameter("Point Cloud", "PC", "Rhino point cloud with per-point colors.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Minimum Stress", "Min", "Stress value used for the low color.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Maximum Stress", "Max", "Stress value used for the high color.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Unit Scale", "Scale", "Coordinate multiplier used to convert meters to the current Rhino document units.", GH_ParamAccess.item);
            pManager.AddTextParameter("Rhino Units", "Units", "Current Rhino document unit system used for coordinate conversion.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            previewPoints.Clear();
            previewColors.Clear();
            previewBox = BoundingBox.Empty;

            string csvPath = null;
            List<Point3d> points = new List<Point3d>();
            List<double> values = new List<double>();
            double minStress = 0.0;
            double maxStress = 0.0;
            bool hasMinOverride = false;
            bool hasMaxOverride = false;
            Color lowColor = Color.FromArgb(0, 105, 255);
            Color highColor = Color.FromArgb(255, 35, 0);
            int pointSize = 5;
            bool coordinatesInMeters = true;

            DA.GetData(0, ref csvPath);

            if (!string.IsNullOrWhiteSpace(csvPath))
            {
                string cleanPath = NormalizePath(csvPath);
                if (!File.Exists(cleanPath))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CSV path does not exist: " + cleanPath);
                    return;
                }

                string error;
                int skippedRows;
                if (!TryReadStressCsv(cleanPath, points, values, out skippedRows, out error))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
                    return;
                }

                if (skippedRows > 0)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, string.Format(CultureInfo.InvariantCulture, "Skipped {0} CSV rows that could not be parsed.", skippedRows));
            }
            else
            {
                DA.GetDataList(1, points);
                DA.GetDataList(2, values);
            }

            if (points.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No points were provided. Connect a CSV Path or point/value lists.");
                return;
            }

            if (values.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No stress values were provided.");
                return;
            }

            if (points.Count != values.Count)
            {
                int count = Math.Min(points.Count, values.Count);
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, string.Format(CultureInfo.InvariantCulture, "Point count ({0}) and stress value count ({1}) differ. Using the first {2} items.", points.Count, values.Count, count));
                points = points.GetRange(0, count);
                values = values.GetRange(0, count);
            }

            hasMinOverride = DA.GetData(3, ref minStress);
            hasMaxOverride = DA.GetData(4, ref maxStress);
            DA.GetData(5, ref lowColor);
            DA.GetData(6, ref highColor);
            DA.GetData(7, ref pointSize);
            DA.GetData(8, ref coordinatesInMeters);

            string rhinoUnits = CurrentDocumentUnitName();
            double unitScale = 1.0;
            if (coordinatesInMeters)
            {
                unitScale = MeterToDocumentUnitScale(out rhinoUnits);
                for (int i = 0; i < points.Count; i++)
                {
                    Point3d point = points[i];
                    points[i] = new Point3d(point.X * unitScale, point.Y * unitScale, point.Z * unitScale);
                }
            }

            if (!hasMinOverride)
                minStress = Min(values);
            if (!hasMaxOverride)
                maxStress = Max(values);

            if (maxStress < minStress)
            {
                double temp = minStress;
                minStress = maxStress;
                maxStress = temp;
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Minimum Stress was greater than Maximum Stress, so the range was swapped.");
            }

            previewPointSize = Math.Max(1, pointSize);

            List<Color> colors = new List<Color>(values.Count);
            PointCloud pointCloud = new PointCloud();

            for (int i = 0; i < points.Count; i++)
            {
                Color color = ColorFromValue(values[i], minStress, maxStress, lowColor, highColor);
                colors.Add(color);
                pointCloud.Add(points[i], color);
            }

            previewPoints.AddRange(points);
            previewColors.AddRange(colors);
            previewBox = new BoundingBox(points);

            DA.SetDataList(0, points);
            DA.SetDataList(1, values);
            DA.SetDataList(2, colors);
            DA.SetData(3, pointCloud);
            DA.SetData(4, minStress);
            DA.SetData(5, maxStress);
            DA.SetData(6, unitScale);
            DA.SetData(7, rhinoUnits);
        }

        public override BoundingBox ClippingBox
        {
            get { return previewBox; }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

            if (previewPoints.Count == 0)
                return;

            for (int i = 0; i < previewPoints.Count; i++)
            {
                Color color = i < previewColors.Count ? previewColors[i] : Color.White;
                args.Display.DrawPoint(previewPoints[i], PointStyle.RoundControlPoint, previewPointSize, color);
            }
        }

        private static bool TryReadStressCsv(string path, List<Point3d> points, List<double> values, out int skippedRows, out string error)
        {
            skippedRows = 0;
            error = null;

            string[] lines = File.ReadAllLines(path);
            if (lines.Length == 0)
            {
                error = "CSV file is empty.";
                return false;
            }

            int xIndex = -1;
            int yIndex = -1;
            int zIndex = -1;
            int stressIndex = -1;
            int startLine = 0;

            List<string> firstColumns = SplitCsvLine(lines[0]);
            bool firstRowIsHeader = firstColumns.Count > 0 && !CanParseDouble(firstColumns[0]);

            if (firstRowIsHeader)
            {
                xIndex = FindColumn(firstColumns, "x");
                yIndex = FindColumn(firstColumns, "y");
                zIndex = FindColumn(firstColumns, "z");
                stressIndex = FindStressColumn(firstColumns);
                startLine = 1;

                if (xIndex < 0 || yIndex < 0 || zIndex < 0 || stressIndex < 0)
                {
                    error = "CSV header must include X, Y, Z, and an EquivalentStress or Stress column.";
                    return false;
                }
            }
            else
            {
                if (firstColumns.Count >= 5)
                {
                    xIndex = 1;
                    yIndex = 2;
                    zIndex = 3;
                    stressIndex = 4;
                }
                else if (firstColumns.Count >= 4)
                {
                    xIndex = 0;
                    yIndex = 1;
                    zIndex = 2;
                    stressIndex = 3;
                }
                else
                {
                    error = "CSV rows must contain either X,Y,Z,Stress or NodeID,X,Y,Z,Stress.";
                    return false;
                }
            }

            for (int i = startLine; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i]))
                    continue;

                List<string> columns = SplitCsvLine(lines[i]);
                int requiredCount = Math.Max(Math.Max(xIndex, yIndex), Math.Max(zIndex, stressIndex)) + 1;
                if (columns.Count < requiredCount)
                {
                    skippedRows++;
                    continue;
                }

                double x;
                double y;
                double z;
                double stress;

                if (!TryParseDouble(columns[xIndex], out x) ||
                    !TryParseDouble(columns[yIndex], out y) ||
                    !TryParseDouble(columns[zIndex], out z) ||
                    !TryParseDouble(columns[stressIndex], out stress))
                {
                    skippedRows++;
                    continue;
                }

                points.Add(new Point3d(x, y, z));
                values.Add(stress);
            }

            if (points.Count == 0)
            {
                error = "CSV did not contain any parseable stress point rows.";
                return false;
            }

            return true;
        }

        private static double MeterToDocumentUnitScale(out string unitName)
        {
            UnitSystem targetUnits = CurrentDocumentUnits();
            unitName = targetUnits.ToString();

            if (targetUnits == UnitSystem.None || targetUnits == UnitSystem.Unset)
                return 1.0;

            double scale = RhinoMath.UnitScale(UnitSystem.Meters, targetUnits);
            if (double.IsNaN(scale) || double.IsInfinity(scale) || scale <= 0.0)
                return 1.0;

            return scale;
        }

        private static UnitSystem CurrentDocumentUnits()
        {
            RhinoDoc doc = RhinoDoc.ActiveDoc;
            if (doc == null)
                return UnitSystem.None;

            return doc.ModelUnitSystem;
        }

        private static string CurrentDocumentUnitName()
        {
            return CurrentDocumentUnits().ToString();
        }

        private static string NormalizePath(string path)
        {
            return Environment.ExpandEnvironmentVariables(path.Trim().Trim('"'));
        }

        private static int FindColumn(List<string> header, string target)
        {
            string normalizedTarget = NormalizeHeader(target);
            for (int i = 0; i < header.Count; i++)
            {
                if (NormalizeHeader(header[i]) == normalizedTarget)
                    return i;
            }

            return -1;
        }

        private static int FindStressColumn(List<string> header)
        {
            for (int i = 0; i < header.Count; i++)
            {
                string name = NormalizeHeader(header[i]);
                if (name == "equivalentstress" ||
                    name == "equivalentstressmpa" ||
                    name == "stress" ||
                    name == "stressmpa" ||
                    name == "vonmises" ||
                    name == "vonmisesstress" ||
                    name == "value" ||
                    name == "values")
                {
                    return i;
                }
            }

            return -1;
        }

        private static string NormalizeHeader(string value)
        {
            return value.Trim().Replace(" ", "").Replace("_", "").Replace("-", "").ToLowerInvariant();
        }

        private static List<string> SplitCsvLine(string line)
        {
            List<string> columns = new List<string>();
            bool inQuotes = false;
            string current = string.Empty;

            for (int i = 0; i < line.Length; i++)
            {
                char c = line[i];
                if (c == '"')
                {
                    if (inQuotes && i + 1 < line.Length && line[i + 1] == '"')
                    {
                        current += '"';
                        i++;
                    }
                    else
                    {
                        inQuotes = !inQuotes;
                    }
                }
                else if (c == ',' && !inQuotes)
                {
                    columns.Add(current.Trim());
                    current = string.Empty;
                }
                else
                {
                    current += c;
                }
            }

            columns.Add(current.Trim());
            return columns;
        }

        private static bool CanParseDouble(string text)
        {
            double value;
            return TryParseDouble(text, out value);
        }

        private static bool TryParseDouble(string text, out double value)
        {
            return double.TryParse(text.Trim(), NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out value) ||
                   double.TryParse(text.Trim(), NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.CurrentCulture, out value);
        }

        private static double Min(List<double> values)
        {
            double min = double.MaxValue;
            foreach (double value in values)
                if (value < min)
                    min = value;
            return min;
        }

        private static double Max(List<double> values)
        {
            double max = double.MinValue;
            foreach (double value in values)
                if (value > max)
                    max = value;
            return max;
        }

        private static Color ColorFromValue(double value, double min, double max, Color low, Color high)
        {
            double t = Math.Abs(max - min) < 1e-12 ? 0.0 : (value - min) / (max - min);
            t = Math.Max(0.0, Math.Min(1.0, t));

            int r = (int)Math.Round(low.R + (high.R - low.R) * t);
            int g = (int)Math.Round(low.G + (high.G - low.G) * t);
            int b = (int)Math.Round(low.B + (high.B - low.B) * t);
            int a = (int)Math.Round(low.A + (high.A - low.A) * t);

            return Color.FromArgb(a, r, g, b);
        }

        protected override Bitmap Icon
        {
            get { return null; }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("93aa5125-9bf0-42c0-85d3-91e061164e3b"); }
        }
    }
}
