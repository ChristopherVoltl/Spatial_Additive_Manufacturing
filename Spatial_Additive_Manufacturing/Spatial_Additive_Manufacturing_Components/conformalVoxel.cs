using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Grasshopper;



namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class ConformalLattice : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CurvePlaneGenerator class.
        /// </summary>
        public ConformalLattice()
          : base("Conformal Lattice Generatrion", "CLG",
              "Gerneration of the Conformal Lattice Structure",
              "FGAM", "Generation")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "Input Brep", GH_ParamAccess.item);
            pManager.AddIntegerParameter("U Divisions", "U", "Number of divisions in the U direction", GH_ParamAccess.item, 10);
            pManager.AddIntegerParameter("V Divisions", "V", "Number of divisions in the V direction", GH_ParamAccess.item, 10);
            pManager.AddIntegerParameter("W Divisions", "W", "Number of divisions in the W direction", GH_ParamAccess.item, 10);
            pManager.AddIntegerParameter("Top Surface", "TS", "Top Surface Face Number", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Bottom Surface", "BS", "Bottom Surface Face Number", GH_ParamAccess.item, 1);


        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Point Grid", "P", "Generated 3D point grid", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Cuboids", "C", "Generated 3D Cuboids", GH_ParamAccess.tree);


        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public DataTree<GH_Integer> GenerateConformalVoxels(int uDivisions, int vDivisions,int wDivisions)
        {
            // Create a data tree for cuboid faces
            DataTree<GH_Integer> cuboidFaces = new DataTree<GH_Integer>();

            // Iterate through the grid to construct cuboids
            for (int w = 0; w<wDivisions; w++)
            {
                for (int v = 0; v<vDivisions; v++)
                {
                    for (int u = 0; u<uDivisions; u++)
                    {
                        // Vertices of the cuboid (relative to the cuboid's position in the grid):
                        //          p3 -------- p7
                        //         /|          /|
                        //       p1 -------- p5 |
                        //       |  |        |  |
                        //       |  |        |  |
                        //       |  p2 ------|- p6
                        //       | /         | /
                        //       p0 -------- p4

                        int p0 = w * (uDivisions + 1) * (vDivisions + 1) + v * (uDivisions + 1) + u; 
                        int p1 = p0 + 1;
                        int p2 = p0 + (uDivisions + 1);
                        int p3 = p2 + 1;
                        int p4 = p0 + (uDivisions + 1) * (vDivisions + 1);
                        int p5 = p4 + 1;
                        int p6 = p4 + (uDivisions + 1);
                        int p7 = p6 + 1;

                        // Define faces using the vertex indicessteam
                        var face1 = new List<int> { p0, p1, p3, p2 }; // Right
                        var face2 = new List<int> { p4, p5, p7, p6 }; // Left
                        var face3 = new List<int> { p0, p4, p6, p2 }; // Bottom
                        var face4 = new List<int> { p1, p5, p7, p3 }; // Top
                        var face5 = new List<int> { p0, p1, p5, p4 }; // Front
                        var face6 = new List<int> { p2, p3, p7, p6 }; // Back

                        // Add each face to the data tree under a branch for the current cuboid
                        var path = new GH_Path(w, v, u); // Branch path for this cuboid
                        cuboidFaces.AddRange(face1.ConvertAll(i => new GH_Integer(i)), path);
                        cuboidFaces.AddRange(face2.ConvertAll(i => new GH_Integer(i)), path);
                        cuboidFaces.AddRange(face3.ConvertAll(i => new GH_Integer(i)), path);
                        cuboidFaces.AddRange(face4.ConvertAll(i => new GH_Integer(i)), path);
                        cuboidFaces.AddRange(face5.ConvertAll(i => new GH_Integer(i)), path);
                        cuboidFaces.AddRange(face6.ConvertAll(i => new GH_Integer(i)), path);
                    }
                }
            }
            return cuboidFaces;
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep brep = null;
            int uDivisions = 10;
            int vDivisions = 10;
            int wDivisions = 10;

            int topSurfaceInt = 0;
            int bottomSurfaceInt = 1;

            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref uDivisions)) return;
            if (!DA.GetData(2, ref vDivisions)) return;
            if (!DA.GetData(3, ref wDivisions)) return;
            if (!DA.GetData(4, ref topSurfaceInt)) return;
            if (!DA.GetData(5, ref bottomSurfaceInt)) return;


            var pointGrid = new List<Point3d>();

            // Get the top and bottom surfaces
            var surfaces = brep.Surfaces;
            if (surfaces.Count < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The Brep must have at least two surfaces.");
                return;
            }

            var topSurface = surfaces[topSurfaceInt];
            var bottomSurface = surfaces[bottomSurfaceInt];



            // Align the starting points of the surfaces (optional flipping for alignment)
            var topStart = topSurface.PointAt(topSurface.Domain(0).Min, topSurface.Domain(1).Min);
            var bottomStart = bottomSurface.PointAt(bottomSurface.Domain(0).Min, bottomSurface.Domain(1).Min);

            if (!topStart.EpsilonEquals(bottomStart, Rhino.RhinoMath.SqrtEpsilon))
            {
                bottomSurface.Reverse(0); // Flip U
            }

            // Generate points on the top and bottom surfaces and interpolate all points
            for (int u = 0; u <= uDivisions; u++)
            {
                for (int v = 0; v <= vDivisions; v++)
                {
                    // Calculate normalized parameters
                    double uParam = u / (double)uDivisions;
                    double vParam = v / (double)vDivisions;

                    // Map UV parameters to the domain of each surface
                    var topU = topSurface.Domain(0).ParameterAt(uParam);
                    var topV = topSurface.Domain(1).ParameterAt(vParam);
                    var topPoint = topSurface.PointAt(topU, topV);

                    var bottomU = bottomSurface.Domain(0).ParameterAt(uParam);
                    var bottomV = bottomSurface.Domain(1).ParameterAt(vParam);
                    var bottomPoint = bottomSurface.PointAt(bottomU, bottomV);

                    // Add top and bottom points to the grid
                    pointGrid.Add(topPoint);
                   

                    // Interpolate points between top and bottom surfaces
                    int totalDivisions = wDivisions; // Match the number of divisions to the U/V grid
                    for (int i = 1; i < totalDivisions; i++)
                    {
                        double blendFactor = i / (double)totalDivisions;

                        // Interpolate between the top and bottom points
                        var blendedPoint = new Point3d(
                            topPoint.X + blendFactor * (bottomPoint.X - topPoint.X),
                            topPoint.Y + blendFactor * (bottomPoint.Y - topPoint.Y),
                            topPoint.Z + blendFactor * (bottomPoint.Z - topPoint.Z)
                        );

                        // Add blended point to the grid
                        pointGrid.Add(blendedPoint);

                        
                    }
                    pointGrid.Add(bottomPoint);
                }
            }

            // Generate the conformal voxels
            var cuboidFaces = GenerateConformalVoxels(uDivisions, vDivisions, wDivisions);

            // Output the final grid of points
            DA.SetDataList(0, pointGrid);
            DA.SetDataTree(1, cuboidFaces);

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
            get { return new Guid("f189692f-54a3-41fa-a737-3da93517e66f"); }
        }
    }
}