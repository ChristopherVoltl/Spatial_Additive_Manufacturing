using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;

namespace Spatial_Additive_Manufacturing
{
    public class SMTConnection_Classified_Component : GH_Component
    {
        public SMTConnection_Classified_Component()
          : base("SMT Connection Classified", "SMT CTM",
              "Spatial printing curves to SMT using explicit horizontal, vertical, and angled member classifications.",
              "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Sorted Classified Curves", "SC", "Sorted classified curve stream from the Classified Member Weave component.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Vertical_E5", "V_E5", "E5 extrusion value for vertical members.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Angled_E5", "A_E5", "E5 extrusion value for angled members.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Horizontal_E5", "H_E5", "E5 extrusion value for horizontal members.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Velocity Ratio Multiplier", "VRx", "Velocity ratio multiplier for generated SMT path points.", GH_ParamAccess.item, 1.4);
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

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "PL", "Planes created at division points by the SMT writer.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Curves", "C", "Classified member curves passed into SMT.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double verticalE5 = 1.4;
            double angledE5 = 1.4;
            double horizontalE5 = 1.4;
            double velocityRatioMultiplier = 1.4;
            double rootRollOffsetDegrees = PlaneGenerationUtils.DefaultRootFacingRollOffsetDegrees;
            Plane rootReferencePlane = Plane.WorldXY;
            double maxRootFacingDeviationDegrees = PlaneGenerationUtils.DefaultMaxXYDeviationDegrees;
            double maxToolTiltDegrees = PlaneGenerationUtils.DefaultMaxToolTiltDegrees;
            double xyPlaneYawOffsetDegrees = PlaneGenerationUtils.DefaultXYPlaneYawOffsetDegrees;

            if (!DA.GetData(1, ref verticalE5)) return;
            if (!DA.GetData(2, ref angledE5)) return;
            if (!DA.GetData(3, ref horizontalE5)) return;
            if (!DA.GetData(4, ref velocityRatioMultiplier)) return;
            DA.GetData(5, ref rootRollOffsetDegrees);
            DA.GetData(6, ref rootReferencePlane);
            DA.GetData(7, ref maxRootFacingDeviationDegrees);
            DA.GetData(8, ref maxToolTiltDegrees);
            DA.GetData(9, ref xyPlaneYawOffsetDegrees);

            var sortedClassifiedMembers = new List<GH_ObjectWrapper>();
            var classifiedCurves = new List<SMTClassifiedCurve>();

            if (!DA.GetDataList(0, sortedClassifiedMembers))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Connect the Classified Member Weave output to Sorted Classified Curves.");
                return;
            }

            AddSortedClassifiedMembers(classifiedCurves, sortedClassifiedMembers);

            if (classifiedCurves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No valid classified curves were supplied. Use the Classified Member Weave classified output, not the preview curve output.");
                return;
            }

            DA.SetDataList(1, classifiedCurves.Select(item => item.Curve));

            SMTConnection_Component.WriteClassifiedToSMT(
                classifiedCurves,
                verticalE5,
                angledE5,
                horizontalE5,
                velocityRatioMultiplier,
                rootRollOffsetDegrees,
                rootReferencePlane,
                maxRootFacingDeviationDegrees,
                maxToolTiltDegrees,
                xyPlaneYawOffsetDegrees);
        }

        private static void AddSortedClassifiedMembers(
            List<SMTClassifiedCurve> classifiedCurves,
            IEnumerable<GH_ObjectWrapper> classifiedMembers)
        {
            foreach (GH_ObjectWrapper memberGoo in classifiedMembers)
            {
                if (TryGetClassifiedCurve(memberGoo, out SMTClassifiedCurve classifiedCurve))
                {
                    classifiedCurves.Add(classifiedCurve);
                }
            }
        }

        private static bool TryGetClassifiedCurve(object value, out SMTClassifiedCurve classifiedCurve)
        {
            classifiedCurve = null;
            if (value == null)
            {
                return false;
            }

            if (value is SMTClassifiedCurve directClassifiedCurve)
            {
                classifiedCurve = directClassifiedCurve;
                return classifiedCurve.Curve != null && classifiedCurve.Curve.IsValid;
            }

            if (value is GH_ObjectWrapper nestedWrapper)
            {
                return TryGetClassifiedCurve(nestedWrapper.Value, out classifiedCurve);
            }

            return false;
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("1a5d9bd2-6786-4e99-8ad4-930827d7f3f1");
    }
}
