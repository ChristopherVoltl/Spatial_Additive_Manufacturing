using System;
using System.Collections.Generic;
using Rhino;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Linq;


namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class LatticeStructuralAnalysis : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public LatticeStructuralAnalysis()
          : base("Lattice Structural Analysis", "LSA",
              "Analysis of the Lattice Structure",
              "FGAM", "Analysis")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("pathCurves", "pC", " an array of Curves", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Model", "LT", "Graph:   Longest Trail Algorithm", GH_ParamAccess.tree);
            pManager.AddPointParameter("Deformation", "GN", "Graph: Nodes", GH_ParamAccess.tree);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public class KarambaTensionCompressionOptimizer
        {
            //add 
            
            
           
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            // Retrieve the input data
            if (!DA.GetDataList(0, curves)) return;

            // Create the graph


            // Set a 5-second timeout
            TimeSpan timeout = TimeSpan.FromSeconds(20);

               

            // Set the output data
            //DA.SetDataList(0, longestTrail);



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
            get { return new Guid("aa6a0a2c-ed7b-4f5e-a69a-271c295a02f9"); }
        }
    }
}