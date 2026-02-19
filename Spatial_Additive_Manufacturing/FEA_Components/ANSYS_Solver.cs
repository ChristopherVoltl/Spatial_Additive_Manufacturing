using System;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using Rhino;
using Rhino.DocObjects;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System.Linq;
using Rhino.Render.ChangeQueue;
using System.Collections;
using Rhino.Geometry.Intersect;
using Rhino.UI;
using System.Diagnostics.Eventing.Reader;
using Compas.Robot.Link;
using System.Security.Cryptography;
using Rhino.FileIO;
using System.Diagnostics;
using System.IO;
using System.Text;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class FEASolver : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public FEASolver()
          : base("FEA Solver", "FEAS",
              "Solves loading conditions in ANSYS with input mesh and loading conditions",
              "FGAM", "FEA")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "inputBrep", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Solve", "S", "True: Solve solution", GH_ParamAccess.item);
            pManager.AddVectorParameter("Load Paths", "lP", "input load paths as a list", GH_ParamAccess.list);
            pManager.AddPathParameter("Working Directory", "WorkDir", "(Optional) File path location", GH_ParamAccess.item);
            pManager.AddTextParameter("Job Name", "jN", "(Optional) Name of the job", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Status", "S", "Is job complete", GH_ParamAccess.item);
            pManager.AddPathParameter("JobFolder", "jF", "JobFolder", GH_ParamAccess.item);
            pManager.AddTextParameter("OutPath", "oP", "OutPath", GH_ParamAccess.item);
            pManager.AddTextParameter("RstPath", "rP", "RstPath", GH_ParamAccess.item);
            pManager.AddTextParameter("Log", "L", "Log", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>

        private void RunScript(Brep G, bool Solve, string MapdlExe, string WorkDir, string JobName,
        ref object Status, ref object JobFolder, ref object OutPath, ref object RstPath, ref object Log)
        {
            // 0) Guard rails
            if (!Solve)
            {
                Status = "Idle (Solve=false)";
                return;
            }
            if (string.IsNullOrWhiteSpace(MapdlExe) || !File.Exists(MapdlExe))
            {
                Status = "ERROR: MapdlExe path is missing or invalid.";
                return;
            }

            // 1) Default work directory (override if user provided)
            string baseDir = string.IsNullOrWhiteSpace(WorkDir)
              ? Path.Combine(Path.GetTempPath(), "GH_MAPDL")
              : WorkDir;

            Directory.CreateDirectory(baseDir);

            // 2) Normalize job name
            string safeJobName = string.IsNullOrWhiteSpace(JobName) ? "job" : SanitizeFileStem(JobName);

            // 3) Compute a stable hash (so GH recompute doesn't always rerun)
            string geomSig = ComputeGeometrySignature(G);
            string jobId = $"{safeJobName}_{geomSig.Substring(0, 10)}";
            string jobDir = Path.Combine(baseDir, jobId);
            Directory.CreateDirectory(jobDir);

            string stepPath = Path.Combine(jobDir, "geom.step");
            string inpPath = Path.Combine(jobDir, "job.inp");
            string outPath = Path.Combine(jobDir, "job.out");
            string rstPath = Path.Combine(jobDir, $"{safeJobName}.rst"); // MAPDL writes <jobname>.rst when /FILNAME is set

            // 4) Export STEP (for later pipeline steps)
            //    This is safe even if APDL doesn't import it yet.
            try
            {
                ExportStepFromBrep(G, stepPath);
            }
            catch (Exception ex)
            {
                // Not fatal for MVP
            }

            // 5) Write APDL input deck (demo analysis to validate the pipeline)
            //    Replace the "DEMO MODEL" section later with your real model creation/import.
            File.WriteAllText(inpPath, BuildApdlDeck(safeJobName), Encoding.ASCII);

            // 6) Run MAPDL in batch
            // Typical pattern: -b -i <input> -o <output>
            var psi = new ProcessStartInfo
            {
                FileName = MapdlExe,
                Arguments = $"-b -i \"{inpPath}\" -o \"{outPath}\"",
                WorkingDirectory = jobDir,
                CreateNoWindow = true,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true
            };

            string stdOut, stdErr;
            int exitCode;

            using (var p = new Process { StartInfo = psi })
            {
                p.Start();
                stdOut = p.StandardOutput.ReadToEnd();
                stdErr = p.StandardError.ReadToEnd();
                p.WaitForExit();
                exitCode = p.ExitCode;
            }

            // 7) Summarize status (very simple heuristic)
            bool outExists = File.Exists(outPath);
            string outText = outExists ? File.ReadAllText(outPath) : "";
            bool looksSolved = outText.IndexOf("S O L U T I O N", StringComparison.OrdinalIgnoreCase) >= 0
                            || outText.IndexOf("SOLVE", StringComparison.OrdinalIgnoreCase) >= 0;

            Status = (exitCode == 0 && outExists && looksSolved)
              ? "OK: MAPDL completed (heuristic: solved)."
              : $"WARN/FAIL: MAPDL exitCode={exitCode}. Check job.out / stderr.";

            JobFolder = jobDir;
            OutPath = outPath;
            RstPath = rstPath;
            Log = $"--- STDOUT ---\n{stdOut}\n\n--- STDERR ---\n{stdErr}\n";
        }

        private static string BuildApdlDeck(string jobname)
        {
            // Minimal demo solve: a small block cantilever-ish setup just to prove orchestration
            // You’ll replace this later with: geometry import / meshing / loads from GH inputs.
            var sb = new StringBuilder();

            sb.AppendLine("/CLEAR");
            sb.AppendLine($"/FILNAME,{jobname},1");
            sb.AppendLine("/PREP7");

            // Element type + material (structural solid demo)
            sb.AppendLine("ET,1,SOLID185");
            sb.AppendLine("MP,EX,1,2.0e11");
            sb.AppendLine("MP,PRXY,1,0.30");

            // DEMO MODEL: simple block 0.1 x 0.02 x 0.02 (meters)
            sb.AppendLine("BLC4,0,0,0.1,0.02,0.02");

            // Mesh
            sb.AppendLine("ESIZE,0.005");
            sb.AppendLine("VMESH,ALL");

            // Fix one end (x=0 face)
            sb.AppendLine("NSEL,S,LOC,X,0");
            sb.AppendLine("D,ALL,ALL,0");
            sb.AppendLine("ALLSEL,ALL");

            // Apply a downward force at the free end nodes (x=0.1)
            sb.AppendLine("NSEL,S,LOC,X,0.1");
            sb.AppendLine("F,ALL,FY,-50"); // demo load
            sb.AppendLine("ALLSEL,ALL");

            sb.AppendLine("/SOLU");
            sb.AppendLine("ANTYPE,0");
            sb.AppendLine("SOLVE");
            sb.AppendLine("FINISH");

            sb.AppendLine("/POST1");
            sb.AppendLine("SET,LAST");
            // You can print summary values to job.out to parse later:
            sb.AppendLine("/COM, POST: ready for DPF extraction from .rst");
            sb.AppendLine("FINISH");

            sb.AppendLine("/EXIT,NOSAVE");

            return sb.ToString();
        }


        private static bool ExportStepFromBrep(Brep brep, string stepPath)
        {
            if (brep == null) return false;

            // Create a temporary "headless" document (lifetime is yours)
            var tempDoc = RhinoDoc.CreateHeadless(null); // Since Rhino 7+ :contentReference[oaicite:1]{index=1}
            try
            {
                // Add geometry into that document
                tempDoc.Objects.AddBrep(brep);
                tempDoc.Views.Redraw();

                var opts = new FileStpWriteOptions();
                // Write STEP from the document
                return FileStp.Write(stepPath, tempDoc, opts); // doc-based :contentReference[oaicite:2]{index=2}
            }
            finally
            {
                tempDoc.Dispose();
            }
        }

        public static class StepExport
        {
            public static bool Export(string stepPath, object brepOrBreps)
            {
                var breps = CoerceBreps(brepOrBreps);
                if (breps.Count == 0) return false;

                var doc = RhinoDoc.CreateHeadless(null);
                try
                {
                    foreach (var b in breps)
                        doc.Objects.AddBrep(b);

                    var opts = new FileStpWriteOptions();
                    // Optional: set schema/units/etc here if needed (depends on Rhino version)

                    return FileStp.Write(stepPath, doc, opts);
                }
                finally
                {
                    doc.Dispose();
                }
            }

            private static List<Brep> CoerceBreps(object input)
            {
                var list = new List<Brep>();
                if (input == null) return list;

                // Single Brep
                if (input is Brep b)
                {
                    list.Add(b);
                    return list;
                }

                // GH might pass lists as IEnumerable
                if (input is IEnumerable enumerable)
                {
                    foreach (var item in enumerable)
                    {
                        if (item is Brep bb) list.Add(bb);
                    }
                }

                return list;
            }
        }

        private static string ComputeGeometrySignature(object G)
        {
            // Keep it simple: use bounding box + type + (if mesh) vertex count.
            // You can strengthen this later (e.g., compute a hash of serialized geometry).
            string sig;

            if (G is Brep b)
            {
                var bb = b.GetBoundingBox(true);
                sig = $"BREP|{bb.Min}|{bb.Max}";
            }
            else
            {
                sig = $"OBJ|{G?.GetType().FullName ?? "null"}";
            }

            using (var sha = SHA256.Create())
            {
                var bytes = Encoding.UTF8.GetBytes(sig);
                var hash = sha.ComputeHash(bytes);
                return BitConverter.ToString(hash).Replace("-", "").ToLowerInvariant();
            }
        }

        private static string SanitizeFileStem(string s)
        {
            foreach (char c in Path.GetInvalidFileNameChars())
                s = s.Replace(c, '_');
            return s.Trim();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep brep = null;
            bool runJob = false;
            List<Vector3d> loadPaths = new List<Vector3d>();
            String jobDir = null;
            String jobName = "Test Job";


            /*
            pManager.AddBrepParameter("Brep", "B", "inputBrep", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Solve", "S", "True: Solve solution", GH_ParamAccess.item);
            pManager.AddVectorParameter("Load Paths", "lP", "input load paths as a list", GH_ParamAccess.list);
            pManager.AddPathParameter("Working Directory", "WorkDir", "(Optional) File path location", GH_ParamAccess.item);
            pManager.AddTextParameter("Job Name", "jN", "(Optional) Name of the job", GH_ParamAccess.item); 
            */


            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref runJob)) return;
            if (!DA.GetData(2, ref loadPaths)) return;
            if (!DA.GetData(3, ref jobDir)) return;
            if (!DA.GetData(4, ref jobName)) return;


            string stepPath = Path.Combine(jobDir, "geom.step");
            bool ok = StepExport.Export(stepPath, brep); 
            if (!ok) throw new Exception("STEP export failed (no valid Breps).");

            /*
            pManager.AddTextParameter("Status", "S", "Is job complete", GH_ParamAccess.item);
            pManager.AddPathParameter("JobFolder", "jF", "JobFolder", GH_ParamAccess.item);
            pManager.AddTextParameter("OutPath", "oP", "OutPath", GH_ParamAccess.item);
            pManager.AddTextParameter("RstPath", "rP", "RstPath", GH_ParamAccess.item);
            pManager.AddTextParameter("Log", "L", "Log", GH_ParamAccess.item);
            */



            // Set the output data
            //DA.SetData(0, status);



        }



        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("27c4df1e-43a0-4054-a32b-a2b38a5597fe"); }
        }
    }
}