using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Commands;
using Rhino;
using Rhino.Geometry;
using SMT;
using static SMT.SMTUtilities;
using Grasshopper.Kernel;
using static Spatial_Additive_Manufacturing.Spatial_Additive_Manufacturing_Component;
using static Spatial_Additive_Manufacturing.Spatial_Printing_Components.N_Bracing;
using System.Runtime.CompilerServices;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Reflection.Metadata;
using System.Security.Cryptography;



namespace Spatial_Additive_Manufacturing
{
    public enum SMTMemberClassification
    {
        Geometric,
        Horizontal,
        Vertical,
        Angled,
        AngledDown
    }

    public sealed class SMTClassifiedCurve
    {
        public SMTClassifiedCurve(Curve curve, SMTMemberClassification classification)
        {
            Curve = curve;
            Classification = classification;
            SegmentClassifications = new[] { classification };
        }

        public SMTClassifiedCurve(Curve curve, IReadOnlyList<SMTMemberClassification> segmentClassifications)
        {
            Curve = curve;
            SegmentClassifications = segmentClassifications ?? Array.Empty<SMTMemberClassification>();
            Classification = SegmentClassifications.Count > 0
                ? SegmentClassifications[0]
                : SMTMemberClassification.Geometric;
        }

        public Curve Curve { get; }
        public SMTMemberClassification Classification { get; }
        public IReadOnlyList<SMTMemberClassification> SegmentClassifications { get; }

        public SMTMemberClassification GetClassificationForSegment(int segmentIndex)
        {
            if (SegmentClassifications != null &&
                segmentIndex >= 0 &&
                segmentIndex < SegmentClassifications.Count)
            {
                return SegmentClassifications[segmentIndex];
            }

            return Classification;
        }

        public override string ToString()
        {
            if (SegmentClassifications == null || SegmentClassifications.Count == 0)
            {
                return $"{Classification} member";
            }

            return string.Join(" -> ", SegmentClassifications);
        }
    }

    public class SMTConnection_Component : GH_Component
    {
        static SuperMatterToolsPlugin smtPlugin => SuperMatterToolsPlugin.Instance;
        private static ToolpathPlaneConduit _toolpathConduit;


        public SMTConnection_Component() : base("SMT Connection", "SMT C", "Spatial printing curves to SMT", "FGAM", "SpatialPrinting")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddPlaneParameter("pathPlanes ", "pP", " an array of Planes", GH_ParamAccess.list);
            pManager.AddCurveParameter("pathLines", "pC", " an array of Curves", GH_ParamAccess.list);
            pManager.AddNumberParameter("Vertical_E5", "V_E5", "Parameter to split the curves at (between 0 and 1)", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Angled_E5", "A_E5", "Parameter to split the curves at (between 0 and 1)", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Horizontal_E5", "H_E5", "Parameter to split the curves at (between 0 and 1)", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Velocity Ratio Multiplier", "VRx", "Parameter to split the curves at (between 0 and 1)", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Root Roll Offset", "RRO", "Local Z roll in degrees applied to root-facing path planes to move A6 away from singularity. Use 0 for old behavior.", GH_ParamAccess.item, PlaneGenerationUtils.DefaultRootFacingRollOffsetDegrees);
            pManager.AddPlaneParameter("XY Plane", "XY", "Machine/root XY plane used for root-facing path plane orientation.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Max XY Deviation", "MXD", "Maximum plane rotation away from the input XY plane. Smaller keeps path planes closer to the root/reference XY plane.", GH_ParamAccess.item, PlaneGenerationUtils.DefaultMaxXYDeviationDegrees);
            pManager.AddNumberParameter("Max Tool Tilt", "MT", "Additional maximum plane Z tilt away from the root plane down direction.", GH_ParamAccess.item, PlaneGenerationUtils.DefaultMaxToolTiltDegrees);
            pManager.AddNumberParameter("A4 Plane Offset", "A4O", "Constant yaw offset in degrees around the input XY plane Z axis. Use small positive or negative values to bias A4 away from its limit.", GH_ParamAccess.item, PlaneGenerationUtils.DefaultXYPlaneYawOffsetDegrees);
            pManager[6].Optional = true;
            pManager[7].Optional = true;
            pManager[8].Optional = true;
            pManager[9].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddCurveParameter("PolyLines", "L", "Output Curves", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Planes", "PL", "Planes created at division points", GH_ParamAccess.list);
            pManager.AddCurveParameter("Curves", "C", "Output Curves", GH_ParamAccess.list);
            //pManager.AddPointParameter("SuperShape", "SS", "SuperShape for each plane", GH_ParamAccess.list);
        }

        private static void AddTraversalSequence(
     List<SMTPData> pDataList,
     Plane pathStart,
     Curve prevCurve,
     ref int counter,
     SuperEvent stopExtrude,
     SuperEvent stopCooling,
     SuperEvent stopHeat,
     SuperEvent extrude,
     List<Plane> allPlanes)
        {
            try
            {
                Point3d prevEnd = prevCurve.PointAtEnd;

                // Use the same X and Y axes as the pathStart plane
                Vector3d xAxis = pathStart.XAxis;
                Vector3d yAxis = pathStart.YAxis;

                float traverseVelRatio = 2.0f;

                if (prevEnd.DistanceTo(pathStart.Origin) > 10.0)
                {
                    // 1. End current path

                    Point3d prevEndlift = new Point3d(prevCurve.PointAtEnd.X, prevCurve.PointAtEnd.Y, prevCurve.PointAtEnd.Z + 4.5);

                    Plane endPlane = new Plane(prevEnd, xAxis, yAxis);
                    var stopData = new SMTPData(counter++, MoveType.Lin, endPlane, traverseVelRatio);
                    stopData.Events["NozzleCooling"] = stopHeat;
                    stopData.Events["NozzleCooling2"] = stopCooling;

                    //stopData.Events["Extrude"] = stopExtrude;
                    pDataList.Add(stopData);
                    allPlanes.Add(endPlane);

                    // 2. Lift vertically
                    Point3d liftPt = new Point3d(prevEnd.X, prevEnd.Y, prevEnd.Z + 70);
                    Plane liftPlane = new Plane(liftPt, xAxis, yAxis);
                    var liftData = new SMTPData(counter++, MoveType.Lin, liftPlane, traverseVelRatio);
                    pDataList.Add(liftData);
                    allPlanes.Add(liftPlane);

                    // 3. Move horizontally to next start point at same Z as lift
                    Point3d traversePt = new Point3d(pathStart.Origin.X, pathStart.Origin.Y, liftPt.Z);
                    Plane traversePlane = new Plane(traversePt, xAxis, yAxis);
                    var traverseData = new SMTPData(counter++, MoveType.Lin, traversePlane, traverseVelRatio * 0.8f);
                    traverseData.Events["Extrude"] = stopExtrude;
                    //traverseData.AxisValues["E5"] = 0.5;
                    pDataList.Add(traverseData);
                    allPlanes.Add(traversePlane);

                    // 4. Preextreude plane at start point before descending
                    Point3d preExtrudePt = new Point3d(pathStart.Origin.X, pathStart.Origin.Y, pathStart.Origin.Z + 5.0);
                    Plane preExtrudePlane = new Plane(preExtrudePt, xAxis, yAxis);
                    var preExtrudeData = new SMTPData(counter++, MoveType.Lin, preExtrudePlane, traverseVelRatio * 0.8f);
                    //traverseData.Events["Extrude"] = extrude;
                    //preExtrudeData.AxisValues["E5"] = 0.5;
                    pDataList.Add(preExtrudeData);
                    allPlanes.Add(preExtrudePlane);
                }
                
            }
            catch (ArgumentOutOfRangeException)
            {
                var recovery = new SMTPData(counter++, MoveType.Lin, pathStart, stopCooling, 2.0f);
                recovery.Events["NozzleCooling"] = stopHeat;
                recovery.Events["NozzleCooling2"] = stopCooling;
                pDataList.Add(recovery);
            }
        }

        public void WriteAllToSMT(
            List<Curve> AllFGAMPData,
            double Vertical_E5,
            double Angled_E5,
            double Horizontal_E5,
            double velocity_ratio_multiplier,
            double rootRollOffsetDegrees = PlaneGenerationUtils.DefaultRootFacingRollOffsetDegrees,
            Plane? rootReferencePlane = null,
            double maxRootFacingDeviationDegrees = PlaneGenerationUtils.DefaultMaxRootFacingDeviationDegrees,
            double maxToolTiltDegrees = PlaneGenerationUtils.DefaultMaxToolTiltDegrees,
            double xyPlaneYawOffsetDegrees = PlaneGenerationUtils.DefaultXYPlaneYawOffsetDegrees)
        {
            List<SMTClassifiedCurve> classifiedCurves = AllFGAMPData
                .Select(curve => new SMTClassifiedCurve(curve, SMTMemberClassification.Geometric))
                .ToList();

            WriteClassifiedToSMT(
                classifiedCurves,
                Vertical_E5,
                Angled_E5,
                Horizontal_E5,
                velocity_ratio_multiplier,
                rootRollOffsetDegrees,
                rootReferencePlane,
                maxRootFacingDeviationDegrees,
                maxToolTiltDegrees,
                xyPlaneYawOffsetDegrees);
        }

        private static PathCurve CreatePathCurve(Line line, SMTMemberClassification classification)
        {
            if (classification == SMTMemberClassification.Horizontal)
            {
                return new PathCurve(line, PathCurve.OrientationType.Horizontal);
            }

            if (classification == SMTMemberClassification.Vertical)
            {
                return new PathCurve(line, PathCurve.OrientationType.Vertical);
            }

            if (classification == SMTMemberClassification.Angled ||
                classification == SMTMemberClassification.AngledDown)
            {
                return new PathCurve(line, PathCurve.OrientationType.AngledDown);
            }

            return new PathCurve(line, 0.01);
        }

        private static Plane GenerateInheritedPlane(PathCurve pathCurve, Point3d origin, Plane inheritedPlane, out double xAxisDif, out double yAxisDif)
        {
            Plane plane = PlaneGenerationUtils.ConstrainPlane(FlipXAxisForNegativeXYCurveTiltIfNeeded(pathCurve, new Plane(origin, inheritedPlane.XAxis, inheritedPlane.YAxis)));
            xAxisDif = Vector3d.VectorAngle(plane.XAxis, Vector3d.XAxis) * (180.0 / Math.PI);
            yAxisDif = Vector3d.VectorAngle(plane.YAxis, Vector3d.YAxis) * (180.0 / Math.PI);
            return plane;
        }

        private static Plane OptimizeRollAroundZ(
            PathCurve pathCurve,
            Plane seedPlane,
            Plane referencePlane,
            bool hasReferencePlane,
            out double xAxisDif,
            out double yAxisDif)
        {
            Plane bestPlane = seedPlane;
            double bestScore = double.MaxValue;

            foreach (double angle in BuildRollCandidateAngles())
            {
                Plane candidate = PlaneGenerationUtils.ConstrainPlane(RotatePlaneAroundLocalZ(seedPlane, angle));
                double score = ScoreAgainstRootFacingPosture(candidate);
                if (hasReferencePlane)
                {
                    score += 0.2 * ScoreAgainstReferencePlane(candidate, referencePlane);
                }

                if (score < bestScore)
                {
                    bestScore = score;
                    bestPlane = candidate;
                }
            }

            Plane adjustedPlane = PlaneGenerationUtils.ConstrainPlane(FlipXAxisForNegativeXYCurveTiltIfNeeded(pathCurve, bestPlane));
            xAxisDif = Vector3d.VectorAngle(adjustedPlane.XAxis, Vector3d.XAxis) * (180.0 / Math.PI);
            yAxisDif = Vector3d.VectorAngle(adjustedPlane.YAxis, Vector3d.YAxis) * (180.0 / Math.PI);
            return adjustedPlane;
        }

        private static Plane FlipXAxisForNegativeXYCurveTiltIfNeeded(PathCurve pathCurve, Plane plane)
        {
            const double flipThresholdDegrees = 20.0;
            const double directionTolerance = 1e-6;

            if (pathCurve == null)
            {
                return plane;
            }

            Vector3d lowerToHigher = GetLowerToHigherCurveVector(pathCurve);
            if (!lowerToHigher.Unitize())
            {
                return plane;
            }

            double verticalDeviation = RhinoMath.ToDegrees(Vector3d.VectorAngle(lowerToHigher, Vector3d.ZAxis));
            bool tiltsTowardNegativeXY =
                lowerToHigher.X < -directionTolerance &&
                lowerToHigher.Y < -directionTolerance;

            if (verticalDeviation <= flipThresholdDegrees || !tiltsTowardNegativeXY)
            {
                return plane;
            }

            // Negate both axes so the plane's X flips while the tool Z direction stays unchanged.
            return new Plane(plane.Origin, -plane.XAxis, -plane.YAxis);
        }

        private static Vector3d GetLowerToHigherCurveVector(PathCurve pathCurve)
        {
            Point3d start = pathCurve.StartPoint;
            Point3d end = pathCurve.EndPoint;

            if (start.Z <= end.Z)
            {
                return end - start;
            }

            return start - end;
        }

        private static IEnumerable<double> BuildRollCandidateAngles()
        {
            yield return 0.0;

            for (int degrees = 15; degrees <= 180; degrees += 15)
            {
                double radians = RhinoMath.ToRadians(degrees);
                yield return radians;
                yield return -radians;
            }
        }

        private static Plane RotatePlaneAroundLocalZ(Plane plane, double angle)
        {
            Vector3d xAxis = plane.XAxis;
            Vector3d yAxis = plane.YAxis;
            Transform rotation = Transform.Rotation(angle, plane.ZAxis, plane.Origin);
            xAxis.Transform(rotation);
            yAxis.Transform(rotation);
            xAxis.Unitize();
            yAxis.Unitize();
            return new Plane(plane.Origin, xAxis, yAxis);
        }

        private static double ScoreAgainstReferencePlane(Plane candidate, Plane referencePlane)
        {
            return Vector3d.VectorAngle(candidate.XAxis, referencePlane.XAxis) +
                   Vector3d.VectorAngle(candidate.YAxis, referencePlane.YAxis);
        }

        private static double ScoreAgainstRootFacingPosture(Plane candidate)
        {
            Vector3d desiredYAxis = PlaneGenerationUtils.ComputeXYReferenceYAxis(candidate.ZAxis);
            if (!desiredYAxis.Unitize())
            {
                return 0.0;
            }

            return Vector3d.VectorAngle(candidate.YAxis, desiredYAxis);
        }

        private static Plane GeneratePathPlane(
            IPlaneGenerator planeGenerator,
            PathCurve eachCurve,
            Point3d referencePoint,
            bool inheritPreviousVerticalPlane,
            Plane previousVerticalPlane,
            bool hasPreviousVerticalPlane,
            out double xAxisDif,
            out double yAxisDif)
        {
            if (inheritPreviousVerticalPlane)
            {
                return GenerateInheritedPlane(eachCurve, referencePoint, previousVerticalPlane, out xAxisDif, out yAxisDif);
            }

            Plane generatedPlane = planeGenerator.GeneratePlane(eachCurve, referencePoint, out xAxisDif, out yAxisDif);
            if (eachCurve.Orientation == PathCurve.OrientationType.Vertical)
            {
                return OptimizeRollAroundZ(
                    eachCurve,
                    generatedPlane,
                    previousVerticalPlane,
                    hasPreviousVerticalPlane,
                    out xAxisDif,
                    out yAxisDif);
            }

            Plane adjustedPlane = PlaneGenerationUtils.ConstrainPlane(FlipXAxisForNegativeXYCurveTiltIfNeeded(eachCurve, generatedPlane));
            xAxisDif = Vector3d.VectorAngle(adjustedPlane.XAxis, Vector3d.XAxis) * (180.0 / Math.PI);
            yAxisDif = Vector3d.VectorAngle(adjustedPlane.YAxis, Vector3d.YAxis) * (180.0 / Math.PI);
            return adjustedPlane;
        }

        public static void WriteClassifiedToSMT(
            List<SMTClassifiedCurve> AllFGAMPData,
            double Vertical_E5,
            double Angled_E5,
            double Horizontal_E5,
            double velocity_ratio_multiplier,
            double rootRollOffsetDegrees = PlaneGenerationUtils.DefaultRootFacingRollOffsetDegrees,
            Plane? rootReferencePlane = null,
            double maxRootFacingDeviationDegrees = PlaneGenerationUtils.DefaultMaxRootFacingDeviationDegrees,
            double maxToolTiltDegrees = PlaneGenerationUtils.DefaultMaxToolTiltDegrees,
            double xyPlaneYawOffsetDegrees = PlaneGenerationUtils.DefaultXYPlaneYawOffsetDegrees)
        {
            PlaneGenerationUtils.RootFacingRollOffsetDegrees = rootRollOffsetDegrees;
            PlaneGenerationUtils.XYPlaneYawOffsetDegrees = xyPlaneYawOffsetDegrees;
            PlaneGenerationUtils.RootReferencePlane = rootReferencePlane ?? Plane.WorldXY;
            PlaneGenerationUtils.MaxXYDeviationDegrees = maxRootFacingDeviationDegrees;
            PlaneGenerationUtils.MaxToolTiltDegrees = maxToolTiltDegrees;

            //get the operation UI!
            int progIndex = smtPlugin.UIData.ProgramIndex;
            int opIndex = smtPlugin.UIData.OperationIndex;
            if (progIndex > -1 && opIndex > -1)
            {
                OperationUI opUI = smtPlugin.UIData.TreeRootUI.WC.ChildNodes[progIndex].ChildNodes[opIndex];
                if (opUI != null)
                {

                    opUI.DivStyle = DivisionStyle.PointData;
                    opUI.FeedMode = FeedMapping.PointData;
                    opUI.ZOrientationStyle = ZOrientStyle.PointData;
                    opUI.YOrientationStyle = YOrientStyle.PointData;
                    opUI.LIStyle = InOutStyle.Inactive;
                    opUI.LOStyle = InOutStyle.Inactive;
                    //opUI.ApproxDist = 0.0f;
                    //opUI.PTP_Traverse = false;

                    //actionstates of the extrusion operation
                    ActionState extrudeAct = opUI.SuperOperationRef.GetActionState("Extrude");
                    SuperActionUI actionUI = opUI.ActionControls["Extrude"];
                    actionUI.ActivationMode = ActivationStyle.PointData;


                    ActionState nozzleHeatingAct = opUI.SuperOperationRef.GetActionState("NozzleCooling");
                    SuperActionUI actionHeatingUI = opUI.ActionControls["NozzleCooling"];
                    actionHeatingUI.ActivationMode = ActivationStyle.PointData;

                    ActionState nozzleCoolingAct = opUI.SuperOperationRef.GetActionState("NozzleCooling2");
                    SuperActionUI actionCoolingUI = opUI.ActionControls["NozzleCooling2"];
                    actionCoolingUI.ActivationMode = ActivationStyle.PointData;

                    ActionState PauseAct = opUI.SuperOperationRef.GetActionState("CycleWait");
                    SuperActionUI actionPauseUI = opUI.ActionControls["CycleWait"];
                    actionPauseUI.StartValue = "3.0";
                    actionPauseUI.ActivationMode = ActivationStyle.PointData;



                    //extrude actionstates
                    SuperEvent extrude = new SuperEvent(extrudeAct, 0.0, EventType.Activate, true);
                    SuperEvent stopExtrude = new SuperEvent(extrudeAct, 0.0, EventType.Deactivate, true);



                    //SuperEvent extrusionE1 = new SuperEvent(extrudeAct, 0.0, EventType.Activate, true);


                    //Nozzle Heating actionstates
                    SuperEvent heat = new SuperEvent(nozzleHeatingAct, 0.0, EventType.Activate, true);
                    SuperEvent stopHeat = new SuperEvent(nozzleHeatingAct, 0.0, EventType.Deactivate, true);
                    
                    //nozzle cooling actionstates
                    SuperEvent cool = new SuperEvent(nozzleCoolingAct, 0.0, EventType.Activate, true);
                    SuperEvent stopCooling = new SuperEvent(nozzleCoolingAct, 0.0, EventType.Deactivate, true);

                    //cycle wait actionstates
                    SuperEvent cycleWait = new SuperEvent(PauseAct, 0.0, EventType.Activate, true);



                    int counter = 0;

                    //SMT.AxisState axisStateE5 = new SMT.AxisState();

                    //create the extrusion data
                    //SMT.AxisState extrusionAxisStateE1 = new SMT.AxisState();
                    //extrusionAxisStateE1.Value = minAlpha;

                    //SMT.AxisState extrusionAxisStateE2 = new SMT.AxisState();
                    //extrusionAxisStateE2.Value = maxAlpha;

                    List<Plane> allPlanes = new List<Plane>();
                    List<double> allE5Values = new List<double>();
                    List<double> allXAxisDifValues = new List<double>();
                    List<double> allYAxisDifValues = new List<double>();
                    List<double> allPlaneRotationAngles = new List<double>();

                    //loop through each path polyline 
                    List<List<SMTPData>> allSMTPData = new();
                    Plane previousVerticalPlane = Plane.Unset;
                    bool hasPreviousVerticalPlane = false;

                    for (int i = 0; i < AllFGAMPData.Count; i++)
                    {
                        SMTClassifiedCurve classifiedCurve = AllFGAMPData[i];
                         Polyline polyline;
                        if (!classifiedCurve.Curve.TryGetPolyline(out polyline))
                        {
                            Line line = new Line(classifiedCurve.Curve.PointAtStart, classifiedCurve.Curve.PointAtEnd);
                            polyline = new Polyline(new[] { line.From, line.To });
                        }
                        int segmentCount = polyline.SegmentCount;
                        int crv_index = 0;
                        List<SMTPData> pData = new();

                        for (int j = 0; j < segmentCount; j++)
                        {
                            Line line = polyline.SegmentAt(j);  
                            SMTMemberClassification segmentClassification = classifiedCurve.GetClassificationForSegment(j);

                            PathCurve eachCurve = CreatePathCurve(line, segmentClassification);

                            IPlaneGenerator planeGenerator = PlaneGeneratorFactory.GetGenerator(eachCurve.Orientation);
                            IPathPointStrategy pointStrategy = PathPointStrategyFactory.GetStrategy(eachCurve, segmentCount, crv_index, Vertical_E5, Angled_E5, Horizontal_E5);
                            bool inheritPreviousVerticalPlane =
                                eachCurve.Orientation == PathCurve.OrientationType.AngledDown &&
                                hasPreviousVerticalPlane;

                            double E5Val = 2.0;
                            float velRatio = 1.0f;



                            //Pre-extrusion
                            Plane prePlane = GeneratePathPlane(
                                planeGenerator,
                                eachCurve,
                                eachCurve.preExtrusion,
                                inheritPreviousVerticalPlane,
                                previousVerticalPlane,
                                hasPreviousVerticalPlane,
                                out double xAxisDif_prePlane,
                                out double yAxisDif_prePlane);
                            
                            //temp rotation of plane 

                            //Traversal 
                            if (j == 0 && i > 0)
                            {
                                Curve prevCurve = AllFGAMPData[i - 1].Curve;
                                AddTraversalSequence(pData, prePlane, prevCurve, ref counter, stopExtrude, stopCooling, stopHeat, extrude, allPlanes);
                            }


                            //SMTPData preExtrudeData = new SMTPData(counter, 0, 0, MoveType.Lin, prePlane, extrude, velRatio);
                            //preExtrudeData.Events["NozzleCooling2"] = stopCooling;
                            //preExtrudeData.Events["NozzleCooling"] = stopHeat;
                            

                            if (eachCurve.Orientation == PathCurve.OrientationType.Vertical)
                            {
                                E5Val = Vertical_E5;
                            }
                            else if (eachCurve.Orientation == PathCurve.OrientationType.AngledUp)
                            {
                                E5Val = Angled_E5;
                            }
                            else if (eachCurve.Orientation == PathCurve.OrientationType.AngledDown)
                            {
                                E5Val = Angled_E5;
                            }
                            else if (eachCurve.Orientation == PathCurve.OrientationType.Horizontal)
                            {
                                E5Val = Horizontal_E5;
                            }
                            //preExtrudeData.AxisValues["E5"] = E5Val;
                            //pData.Add(preExtrudeData);
                            //counter++;
                            // Store for visualization:
                            //allPlanes.Add(prePlane);
                            //allE5Values.Add(E5Val);




                            //Path points that are generated per each curve type
                            var sequence = pointStrategy.GetPathPoints(eachCurve, polyline.SegmentCount, j, Vertical_E5, Angled_E5, Horizontal_E5);
                            

                            foreach (var step in sequence)
                            {
                                Plane pathPlane = GeneratePathPlane(
                                    planeGenerator,
                                    eachCurve,
                                    step.Point,
                                    inheritPreviousVerticalPlane,
                                    previousVerticalPlane,
                                    hasPreviousVerticalPlane,
                                    out double xAxisDif_pathPlane,
                                    out double yAxisDif_pathPlane);

                                velRatio = step.VelRatio;

                                RhinoApp.WriteLine(
                                $"i={i} j={j} idx={crv_index} count={segmentCount} orient={eachCurve.Orientation} stepVR={step.VelRatio}");
                                var smtpData = new SMTPData(counter, MoveType.Lin, pathPlane, velRatio);
                                smtpData.LengthParam = counter;
                                smtpData.MType = MoveType.Lin;
                                
                                smtpData.AxisValues["E5"] = step.E5Value;

                                // Activate/deactivate cooling:
                                if (step.CoolingOn) smtpData.Events["NozzleCooling2"] = cool;
                                else smtpData.Events["NozzleCooling2"] = stopCooling;

                                // Activate/deactivate extrusion:
                                if (step.ExtrudeOn) smtpData.Events["Extrude"] = extrude;
                                else smtpData.Events["Extrude"] = stopExtrude;

                                // Activate/deactivate heat:
                                if (step.HeatOn) smtpData.Events["NozzleCooling"] = heat;  
                                else smtpData.Events["NozzleCooling"] = stopHeat;


                                if (step.CycleWait) smtpData.Events["CycleWait"] = cycleWait;
                                


                                pData.Add(smtpData);
                                counter++;


                                // Store for visualization:
                                allPlanes.Add(pathPlane);
                                allE5Values.Add(step.E5Value);
                                Plane rootReference = PlaneGenerationUtils.GetValidRootReferencePlane();
                                var diffs_pathData = ToolpathPlaneConduit.GetPlaneAxisDifferences(rootReference, pathPlane);
                                double xAxisDif_pathData = diffs_pathData.xAxisDif;
                                double yAxisDif_pathData = diffs_pathData.yAxisDif;

                                double planeRotAngle = ToolpathPlaneConduit.GetPlaneRotationAngle(rootReference, pathPlane);

                                allXAxisDifValues.Add(xAxisDif_pathPlane);
                                allYAxisDifValues.Add(yAxisDif_pathPlane);
                                allPlaneRotationAngles.Add(planeRotAngle);

                                if (eachCurve.Orientation == PathCurve.OrientationType.Vertical)
                                {
                                    previousVerticalPlane = pathPlane;
                                    hasPreviousVerticalPlane = true;
                                }
                                
                            }

                            crv_index++;

                            Plane stopPlane;

                            //Stop-extrusion
                            if (eachCurve.Line.Length < 25.0)
                            {
                                Point3d point3D = new Point3d(eachCurve.EndPoint.X, eachCurve.EndPoint.Y, eachCurve.EndPoint.Z + 5.5);
                                stopPlane = GeneratePathPlane(
                                    planeGenerator,
                                    eachCurve,
                                    point3D,
                                    inheritPreviousVerticalPlane,
                                    previousVerticalPlane,
                                    hasPreviousVerticalPlane,
                                    out double xAxisDif_stopPlane,
                                    out double yAxisDif_stopPlane);
                            }
                            else
                            {
                                Point3d point3D = new Point3d(eachCurve.EndPoint.X, eachCurve.EndPoint.Y, eachCurve.EndPoint.Z + 2.5);
                                stopPlane = GeneratePathPlane(
                                    planeGenerator,
                                    eachCurve,
                                    eachCurve.EndPoint,
                                    inheritPreviousVerticalPlane,
                                    previousVerticalPlane,
                                    hasPreviousVerticalPlane,
                                    out double xAxisDif_stopPlane,
                                    out double yAxisDif_stopPlane);
                            }
                                
                            //SMTPData stopExtrudeData = new SMTPData(counter, 0, 0, MoveType.Lin, stopPlane, stopExtrude, 0.2f);
                            //stopExtrudeData.Events["NozzleCooling2"] = stopCooling;
                            //stopExtrudeData.Events["Extrude"] = stopExtrude;
                            //stopExtrudeData.Events["NozzleCooling"] = stopHeat;
                            //stopExtrudeData.AxisValues["E5"] = E5Val;
                            //pData.Add(stopExtrudeData);
                            //counter++;

                            // Store for visualization:
                            //allPlanes.Add(stopPlane);
                            allE5Values.Add(E5Val);
                        }

                        if (pData.Count > 0)
                        {
                            allSMTPData.Add(pData);
                        }

                        
                    }

                    /*if (_toolpathConduit != null)
                    {
                        _toolpathConduit.Enabled = false;
                        _toolpathConduit = null;
                        Rhino.RhinoDoc.ActiveDoc.Views.Redraw();
                    }*/

                    /*// Create and enable new conduit:
                    _toolpathConduit = new ToolpathPlaneConduit(
                        allPlanes,
                        allE5Values,
                        axisSize: 1.0,
                        showPlaneIndex: false,
                        useE5Gradient: true
                    );*/

                    //_toolpathConduit.Enabled = true;
                    //Rhino.RhinoDoc.ActiveDoc.Views.Redraw();

                    //_toolpathConduit.Enabled = false;
                    //_toolpathConduit = null;
                    //Rhino.RhinoDoc.ActiveDoc.Views.Redraw();



                    //store all the pointdata and then instantiate the shape outside of the loop
                    Guid guid = Guid.NewGuid();



                    List<SuperShape> shapes = new List<SuperShape>();

                    for (int i = 0; i < allSMTPData.Count; i++)
                    {
                        List<SMTPData> pData = allSMTPData[i];
                        List<SMTPData> pDataList = new ();
                        pDataList.AddRange(pData);
                        smtPlugin.UserData[guid] = pDataList.ToArray();
                        SuperShape shape = SuperShape.SuperShapeFactory(guid, null, DivisionStyle.PointData, ZOrientStyle.PointData, VectorStyle.ByParam, YOrientStyle.PointData, false, 0.0, Rhino.Geometry.Plane.WorldXY);
                        shapes.Add(shape);

                    }


                    if (shapes.Count > 0)
                    {
                        var spbs = opUI.ReadFromGH(shapes.ToArray());
                        if (spbs != null)
                        {
                            spbs.Last().IsSelected = true;
                            opUI.IsSelected = true;
                        }
                    }


                }
                else
                {
                    RhinoApp.WriteLine("You must select an Operation");
                }

            }

            else
            {
                RhinoApp.WriteLine("You must select an Operation");
            }
        }
        


        public static Result SMTSetup(RhinoDoc doc)
        {
            //Startup
            if (doc.Name == "")
            {
                SaveRhinoFile(doc);
                doc = RhinoDoc.ActiveDoc;
            }
            if (doc.Name == "")
            {
                //Rhino.UI.Dialogs.ShowMessageBox("You must have a named (saved) file to work in SMT.\nPlease save your file and try again.", "SuperMatterTools");
                return Result.Cancel;
            }
            SuperMatterToolsPlugin plugIn = SuperMatterToolsPlugin.Instance;
            UIData uiData;
            if (plugIn.UIData == null)
            {
                uiData = new UIData();
                uiData.Startup();
                plugIn.UIData = uiData;
                plugIn.UIData.PresentUI();
            }
            else
            {
                uiData = plugIn.UIData;
            }

            //WorkCell
            if (uiData.TreeRootUI == null)
            {

                //uiData.AddTreeRootUI("KR120_Solo");
                uiData.AddTreeRootUI("KR120_Solo");
            }
            else
            {
                if (uiData.TreeRootUI.WC.Name != "KR120_Solo")
                {
                    uiData.TreeRootUI.WC.Name = "KR120_Solo";
                }
            }
            //Program
            if (uiData.TreeRootUI.WC.ChildNodes.Count > 0)
            {
                uiData.TreeRootUI.WC.ChildNodes[0].Name = "Main";
            }
            else
            {
                uiData.TreeRootUI.WC.AddProgram("Main", 0);
            }
            //Operation
            ProgramUI progUI = uiData.TreeRootUI.WC.ChildNodes[0];
            if (progUI.ChildNodes.Count > 0)
            {
                //progUI.ChildNodes[0].ProcessName = "DualExtruder_14mm_UofM";
                progUI.ChildNodes[0].ProcessName = "Extruder_14mm22";

            }
            else
            {
                progUI.AddOperation(0, "StarExtruder_14mm_UofM");
                //progUI.ChildNodes[0].ProcessName = "Extruder_14mm22";
                //progUI.AddOperation(0, "Extruder_14mm22");

            }
            uiData.PurgeUnusedUserData();
            progUI.SuperProgramRef.UpdateProgramData();
            uiData.UpdateLight();

            //select the first op in first program
            smtPlugin.UIData.TreeRootUI.WC.ChildNodes.First().IsSelected = true;
            smtPlugin.UIData.TreeRootUI.WC.ChildNodes.First().ChildNodes.First().IsSelected = true;

            return Result.Success;
        }




        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> pathCurves = new();
            double Vertical_E5 = new();
            double Angled_E5 = new();
            double Horizontal_E5 = new();
            double Velocity_Ratio_Multiplier = new();
            double rootRollOffsetDegrees = PlaneGenerationUtils.DefaultRootFacingRollOffsetDegrees;
            Plane rootReferencePlane = Plane.WorldXY;
            double maxRootFacingDeviationDegrees = PlaneGenerationUtils.DefaultMaxXYDeviationDegrees;
            double maxToolTiltDegrees = PlaneGenerationUtils.DefaultMaxToolTiltDegrees;
            double xyPlaneYawOffsetDegrees = PlaneGenerationUtils.DefaultXYPlaneYawOffsetDegrees;

            if (!DA.GetDataList(0, pathCurves)) { return; }
            if (!DA.GetData(1, ref Vertical_E5)) return;
            if (!DA.GetData(2, ref Angled_E5)) return;
            if (!DA.GetData(3, ref Horizontal_E5)) return;
            if (!DA.GetData(4, ref Velocity_Ratio_Multiplier)) return;
            DA.GetData(5, ref rootRollOffsetDegrees);
            DA.GetData(6, ref rootReferencePlane);
            DA.GetData(7, ref maxRootFacingDeviationDegrees);
            DA.GetData(8, ref maxToolTiltDegrees);
            DA.GetData(9, ref xyPlaneYawOffsetDegrees);
            // 1. Setup the SMT environment.

            // 3. Abort on invalid inputs.
            if (pathCurves == null)
            {
                RhinoApp.WriteLine("The selected object is not a Curve.");
            }
          

            WriteAllToSMT(
                pathCurves,
                Vertical_E5,
                Angled_E5,
                Horizontal_E5,
                Velocity_Ratio_Multiplier,
                rootRollOffsetDegrees,
                rootReferencePlane,
                maxRootFacingDeviationDegrees,
                maxToolTiltDegrees,
                xyPlaneYawOffsetDegrees);

            
        }


        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("6d935d9f-075a-48d1-a345-888964696dd6");
    }

}
