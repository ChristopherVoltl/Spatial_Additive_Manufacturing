using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace Spatial_Additive_Manufacturing
{
    public class Spatial_Additive_ManufacturingInfo : GH_AssemblyInfo
    {
        public override string Name => "Spatial_Additive_Manufacturing";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("af06e098-a3bc-4282-af02-553b656710a5");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}