using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace Truss3D
{
    public class BDCComponents : GH_Component
    {
        public BDCComponents()
          : base("BDCComponents", "Nickname",
              "Description",
              "Koala", "Truss3D")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points to apply Boundary Conditions", GH_ParamAccess.list);
            pManager.AddLineParameter("Geometry", "G", "Geometry", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Boundary Conditions", "BDC", "Boundary Conditions x,y,z where 0=clamped and 1=free", GH_ParamAccess.list, new List<int>(new int[] { 0, 0, 0 }));
            pManager.AddTextParameter("Locked direction", "Ldir", "Lock x, y or z direction for deformation", GH_ParamAccess.item, "");
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("B.Cond.", "BDC", "Boundary Conditions for 2D Truss Calculation", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Expected inputs
            List<Point3d> pointList = new List<Point3d>();          //List of points where BDC is to be applied
            List<Line> geometry = new List<Line>();
            List<int> BDC = new List<int>();                        //is BDC free? (=clamped) (1 == true, 0 == false)
            List<string> pointInStringFormat = new List<string>();  //output in form of list of strings
            string lock_dir = "";


            //Set expected inputs from Indata and aborts with error message if input is incorrect
            if (!DA.GetDataList(0, pointList)) return;
            if (!DA.GetDataList(1, geometry)) return;
            if (!DA.GetDataList(2, BDC)) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "testing"); return; }
            if (!DA.GetData(3, ref lock_dir)) return;

            //Preallocate temporary variables
            string BDCString;
            int bdcx = 0;
            int bdcy = 0;
            int bdcz = 0;

            if (lock_dir == "")
            {
                if (BDC.Count == 1) //Boundary condition input for identical conditions in all points. Split into if/else for optimization
                {
                    bdcx = BDC[0];
                    bdcy = BDC[0];
                    bdcz = BDC[0];

                    BDCString = bdcx + "," + bdcy + "," + bdcz;

                    for (int i = 0; i < pointList.Count; i++)   //Format stringline for all points (identical boundary conditions for all points)
                    {
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);
                    }
                }
                else if (BDC.Count == 3) //Boundary condition input for identical conditions in all points. Split into if/else for optimization
                {
                    bdcx = BDC[0];
                    bdcy = BDC[1];
                    bdcz = BDC[2];

                    BDCString = bdcx + "," + bdcy + "," + bdcz;

                    for (int i = 0; i < pointList.Count; i++)   //Format stringline for all points (identical boundary conditions for all points)
                    {
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);
                    }
                }
                else    //BDCs are not identical for all points
                {
                    for (int i = 0; i < pointList.Count; i++)
                    {
                        if (i > (BDC.Count / 3) - 1)  //Are there more points than BDCs given? (BDC always lists x,y,z per point)
                        {
                            BDCString = bdcx + "," + bdcy + "," + bdcz; //use values from last BDC in list of BDCs
                        }
                        else
                        {
                            //retrieve BDC for x,y,z-dir
                            bdcx = BDC[i * 3];
                            bdcy = BDC[i * 3 + 1];
                            bdcz = BDC[i * 3 + 2];
                            BDCString = bdcx + "," + bdcy + "," + bdcz;
                        }
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);    //Add stringline to list of strings
                    }
                }
            }
            else
            {
                bool lx = false;
                bool ly = false;
                bool lz = false;

                if (lock_dir == "X" || lock_dir == "x")
                {
                    lx = true;
                    bdcx = 0;
                }
                else if (lock_dir == "Y" || lock_dir == "y")
                {
                    ly = true;
                    bdcy = 0;
                }
                else if (lock_dir == "Z" || lock_dir == "z")
                {
                    ly = true;
                    bdcz = 0;
                }

                List<Point3d> points = CreatePointList(geometry);
                for (int i = 0; i < pointList.Count; i++)
                {
                    points.Remove(pointList[i]);
                }
                for (int i = 0; i < points.Count; i++)
                {
                    if (!lx) bdcx = 1;
                    if (!ly) bdcy = 1;
                    if (!lz) bdcz = 1;

                    BDCString = bdcx + "," + bdcy + "," + bdcz;
                    pointInStringFormat.Add(points[i].X + "," + points[i].Y + "," + points[i].Z + ":" + BDCString);
                }

                if (BDC.Count == 1) //Boundary condition input for identical conditions in all points. Split into if/else for optimization
                {
                    if (!lx) bdcx = BDC[0];
                    if (!ly) bdcy = BDC[0];
                    if (!lz) bdcz = BDC[0];

                    BDCString = bdcx + "," + bdcy + "," + bdcz;

                    for (int i = 0; i < pointList.Count; i++)   //Format stringline for all points (identical boundary conditions for all points)
                    {
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);
                    }
                }
                else if (BDC.Count == 3) //Boundary condition input for identical conditions in all points. Split into if/else for optimization
                {
                    if (!lx) bdcx = BDC[0];
                    if (!ly) bdcy = BDC[1];
                    if (!lz) bdcz = BDC[2];

                    BDCString = bdcx + "," + bdcy + "," + bdcz;

                    for (int i = 0; i < pointList.Count; i++)   //Format stringline for all points (identical boundary conditions for all points)
                    {
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);
                    }
                }
                else    //BDCs are not identical for all points
                {
                    for (int i = 0; i < pointList.Count; i++)
                    {
                        if (i > (BDC.Count / 3) - 1)  //Are there more points than BDCs given? (BDC always lists x,y,z per point)
                        {
                            BDCString = bdcx + "," + bdcy + "," + bdcz; //use values from last BDC in list of BDCs
                        }
                        else
                        {
                            //retrieve BDC for x,y,z-dir
                            if (!lx) bdcx = BDC[i * 3];
                            if (!ly) bdcy = BDC[i * 3 + 1];
                            if (!lz) bdcz = BDC[i * 3 + 2];
                            BDCString = bdcx + "," + bdcy + "," + bdcz;
                        }
                        pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);    //Add stringline to list of strings
                    }
                }
            }

            DA.SetDataList(0, pointInStringFormat);
        } //End of main program

        private List<Point3d> CreatePointList(List<Line> geometry)
        {
            List<Point3d> points = new List<Point3d>();

            for (int i = 0; i < geometry.Count; i++) //adds every point unless it already exists in list
            {
                Line l1 = geometry[i];
                if (!points.Contains(l1.From))
                {
                    points.Add(l1.From);
                }
                if (!points.Contains(l1.To))
                {
                    points.Add(l1.To);
                }
            }

            return points;
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.BDC; //Setting component icon
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("1376de2c-8393-45c9-81c8-512c87f6061f"); }
        }
    }
}