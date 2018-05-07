using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Truss3D
{
    public class Truss3DInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Truss3D";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("2170d22e-4c93-49f7-b83b-9cedbbc614e2");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
