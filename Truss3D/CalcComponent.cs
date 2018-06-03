using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Truss3D
{
    public class CalcComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public CalcComponent()
          : base("Truss3DCalc", "TCalc",
              "Description",
              "Koala", "Truss3D")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Lines", "LNS", "Geometry, in form of Lines)", GH_ParamAccess.list);
            pManager.AddTextParameter("Boundary Conditions", "BDC", "Boundary Conditions in form (x,z):1,1 where 1=free and 0=restrained", GH_ParamAccess.list);
            pManager.AddNumberParameter("Crossection area", "A", "Crossectional area, initial value 10e3 [mm*mm]", GH_ParamAccess.item, 3600);
            pManager.AddNumberParameter("Material E modulus", "E", "Material Property, initial value 210e3 [MPa]", GH_ParamAccess.item, 210000);
            pManager.AddTextParameter("Loads", "L", "Load given as Vector [N]", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Deformations", "Def", "Deformations", GH_ParamAccess.list);
            pManager.AddNumberParameter("Reactions", "R", "Reaction Forces", GH_ParamAccess.list);
            pManager.AddNumberParameter("Element stresses", "Strs", "The Stress in each element", GH_ParamAccess.list);
            pManager.AddNumberParameter("Element strains", "Strn", "The Strain in each element", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Expected inputs
            List<Line> geometry = new List<Line>();         //initial Geometry of lines
            double E = 0;                                   //Material property, initial value 210000 [MPa]
            double A = 0;                                   //Area for each element in same order as geometry, initial value 10000 [mm^2]
            List<string> bdctxt = new List<string>();       //Boundary conditions in string format
            List<string> loadtxt = new List<string>();      //loads in string format


            //Set expected inputs from Indata
            if (!DA.GetDataList(0, geometry)) return;       //sets geometry
            if (!DA.GetDataList(1, bdctxt)) return;         //sets boundary conditions
            if (!DA.GetData(2, ref A)) return;              //sets Area
            if (!DA.GetData(3, ref E)) return;              //sets material
            if (!DA.GetDataList(4, loadtxt)) return;        //sets load


            //List all nodes (every node only once), numbering them according to list index
            List<Point3d> points = CreatePointList(geometry);


            //Interpret the BDC inputs (text) and create list of boundary condition (1/0 = free/clamped) for each dof.
            Vector<double> bdc_value = CreateBDCList(bdctxt, points);


            //Interpreting input load (text) and creating load list (double)
            List<double> load = CreateLoadList(loadtxt, points);


            //Create global stiffness matrix
            Matrix<double> K_tot = CreateGlobalStiffnessMatrix(geometry, points, E, A);


            Matrix<double> K_red;
            Vector<double> load_red;
            //Create reduced K-matrix and reduced load list (removed free dofs)
            CreateReducedGlobalStiffnessMatrix(bdc_value, K_tot, load, out K_red, out load_red);


            //Calculate deformations
            Vector<double> def_reduced = K_red.Cholesky().Solve(load_red);


            //Add the clamped dofs (= 0) to the deformations list
            Vector<double> def_tot = RestoreTotalDeformationVector(def_reduced, bdc_value);


            //Calculate the reaction forces from the deformations
            Vector<double> reactions = K_tot.Multiply(def_tot);


            List<double> internalStresses;
            List<double> internalStrains;
            //Calculate the internal strains and stresses in each member
            CalculateInternalStrainsAndStresses(def_tot, points, E, geometry, out internalStresses, out internalStrains);


            DA.SetDataList(0, def_tot);
            DA.SetDataList(1, reactions);
            DA.SetDataList(2, internalStresses);
            DA.SetDataList(3, internalStrains);
        } //End of main program

        private void CalculateInternalStrainsAndStresses(Vector<double> def, List<Point3d> points, double E, List<Line> geometry, out List<double> internalStresses, out List<double> internalStrains)
        {
            //preallocating lists
            internalStresses = new List<double>(geometry.Count);
            internalStrains = new List<double>(geometry.Count);

            foreach (Line line in geometry)
            {
                int index1 = points.IndexOf(new Point3d(Math.Round(line.From.X, 5), Math.Round(line.From.Y, 5), Math.Round(line.From.Z, 5)));
                int index2 = points.IndexOf(new Point3d(Math.Round(line.To.X, 5), Math.Round(line.To.Y, 5), Math.Round(line.To.Z, 5)));

                //fetching deformation of point
                double x1 = def[index1 * 3 + 0];
                double y1 = def[index1 * 3 + 1];
                double z1 = def[index1 * 3 + 2];
                double x2 = def[index2 * 3 + 0];
                double y2 = def[index2 * 3 + 1];
                double z2 = def[index2 * 3 + 2];

                //new node coordinates for deformed nodes
                double nx1 = points[index1].X + x1;
                double ny1 = points[index1].Y + y1;
                double nz1 = points[index1].Z + z1;
                double nx2 = points[index2].X + x2;
                double ny2 = points[index2].Y + y2;
                double nz2 = points[index2].Z + z2;

                //calculating dL = length of deformed line - original length of line
                double dL = Math.Sqrt(Math.Pow((nx2 - nx1), 2) + Math.Pow((ny2 - ny1), 2) + Math.Pow((nz2 - nz1), 2)) - line.Length;

                //calculating strain and stress
                internalStrains.Add(dL / line.Length);
                internalStresses.Add(internalStrains[internalStrains.Count - 1] * E);
            }
        }

        private Vector<double> RestoreTotalDeformationVector(Vector<double> deformations_red, Vector<double> bdc_value)
        {
            Vector<double> def = Vector<double>.Build.Dense(bdc_value.Count);
            for (int i = 0, j = 0; i < bdc_value.Count; i++)
            {
                if (bdc_value[i] == 1)
                {
                    def[i] = deformations_red[j];
                    j++;
                }
            }
            return def;
        }

        private static void CreateReducedGlobalStiffnessMatrix(Vector<double> bdc_value, Matrix<double> K, List<double> load, out Matrix<double> K_red, out Vector<double> load_red)
        {
            K_red = Matrix<double>.Build.SparseOfMatrix(K);
            List<double> load_redu = new List<double>(load);
            for (int i = 0, j = 0; i < load.Count; i++)
            {
                if (bdc_value[i] == 0)
                {
                    K_red = K_red.RemoveRow(i - j);
                    K_red = K_red.RemoveColumn(i - j);
                    load_redu.RemoveAt(i - j);
                    j++;
                }
            }
            load_red = Vector<double>.Build.DenseOfEnumerable(load_redu);
        }

        private Matrix<double> CreateGlobalStiffnessMatrix(List<Line> geometry, List<Point3d> points, double E, double A)
        {
            int gdofs = points.Count * 3;
            Matrix<double> KG = SparseMatrix.OfArray(new double[gdofs, gdofs]);

            foreach (Line currentLine in geometry)
            {
                double lineLength = Math.Round(currentLine.Length, 6);
                double mat = (E * A) / (lineLength);    //material properties
                Point3d p1 = new Point3d(Math.Round(currentLine.From.X, 5), Math.Round(currentLine.From.Y, 5), Math.Round(currentLine.From.Z, 5));
                Point3d p2 = new Point3d(Math.Round(currentLine.To.X, 5), Math.Round(currentLine.To.Y, 5), Math.Round(currentLine.To.Z, 5));

                double cx = (p2.X - p1.X) / lineLength;
                double cy = (p2.Y - p1.Y) / lineLength;
                double cz = (p2.Z - p1.Z) / lineLength;
                
                Matrix<double> T = SparseMatrix.OfArray(new double[,]
                {
                    { (cx),  (cy),  (cz), 0,0,0},
                    { (cx),  (cy),  (cz), 0,0,0},
                    { (cx),  (cy),  (cz), 0,0,0},
                    {0,0,0, (cx),  (cy),  (cz)},
                    {0,0,0, (cx),  (cy),  (cz)},
                    {0,0,0, (cx),  (cy),  (cz)},
                });

                Matrix<double> ke = DenseMatrix.OfArray(new double[,]
                                {
                    {   1,  0,  0,  -1,   0,   0},
                    {   0,  0,  0,  0,   0,   0},
                    {   0,  0,  0,  0,   0,   0},
                    {   -1,  0,  0,  1,   0,   0},
                    {   0,  0,  0,  0,   0,   0},
                    {   0,  0,  0,  0,   0,   0,}
                });

                Matrix<double> T_T = T.Transpose();
                Matrix<double> Ke = ke.Multiply(T);
                Ke = T_T.Multiply(Ke);
                Ke = mat * Ke;
                
                int node1 = points.IndexOf(p1);
                int node2 = points.IndexOf(p2);
                
                //Inputting values to correct entries in Global Stiffness Matrix
                for (int i = 0; i < Ke.RowCount / 2; i++)
                {

                    for (int j = 0; j < Ke.ColumnCount / 2; j++)
                    {
                        //top left 3x3 of k-element matrix
                        KG[node1 * 3 + i, node1 * 3 + j] += Ke[i, j];
                        //top right 3x3 of k-element matrix  
                        KG[node1 * 3 + i, node2 * 3 + j] += Ke[i, j + 3];
                        //bottom left 3x3 of k-element matrix
                        KG[node2 * 3 + i, node1 * 3 + j] += Ke[i + 3, j];
                        //bottom right 3x3 of k-element matrix
                        KG[node2 * 3 + i, node2 * 3 + j] += Ke[i + 3, j + 3];
                    }
                }
            }
            return KG;
        }

        private List<double> CreateLoadList(List<string> loadtxt, List<Point3d> points)
        {
            List<double> loads = new List<double>(new double[points.Count * 3]);
            List<double> inputLoads = new List<double>();
            List<Point3d> coordlist = new List<Point3d>();

            for (int i = 0; i < loadtxt.Count; i++)
            {
                string coordstr = (loadtxt[i].Split(':')[0]);
                string loadstr = (loadtxt[i].Split(':')[1]);

                string[] coordstr1 = (coordstr.Split(','));
                string[] loadstr1 = (loadstr.Split(','));

                inputLoads.Add(Math.Round(double.Parse(loadstr1[0]), 5));
                inputLoads.Add(Math.Round(double.Parse(loadstr1[1]), 5));
                inputLoads.Add(Math.Round(double.Parse(loadstr1[2]), 5));

                coordlist.Add(new Point3d(Math.Round(double.Parse(coordstr1[0]), 5), Math.Round(double.Parse(coordstr1[1]), 5), Math.Round(double.Parse(coordstr1[2]), 5)));
            }

            foreach (Point3d point in coordlist)
            {
                int i = points.IndexOf(point);
                int j = coordlist.IndexOf(point);
                loads[i * 3 + 0] = inputLoads[j * 3 + 0];
                loads[i * 3 + 1] = inputLoads[j * 3 + 1];
                loads[i * 3 + 2] = inputLoads[j * 3 + 2];
            }
            return loads;
        }

        private Vector<double> CreateBDCList(List<string> bdctxt, List<Point3d> points)
        {
            Vector<double> bdc_value = Vector<double>.Build.Dense(points.Count * 3, 1);
            List<int> bdcs = new List<int>();
            List<Point3d> bdc_points = new List<Point3d>(); //Coordinates relating til bdc_value in for (eg. x y z)

            for (int i = 0; i < bdctxt.Count; i++)
            {
                string coordstr = (bdctxt[i].Split(':')[0]);
                string bdcstr = (bdctxt[i].Split(':')[1]);

                string[] coordstr1 = (coordstr.Split(','));
                string[] bdcstr1 = (bdcstr.Split(','));

                bdc_points.Add(new Point3d(Math.Round(double.Parse(coordstr1[0]), 5), Math.Round(double.Parse(coordstr1[1]), 5), Math.Round(double.Parse(coordstr1[2]), 5)));

                bdcs.Add(int.Parse(bdcstr1[0]));
                bdcs.Add(int.Parse(bdcstr1[1]));
                bdcs.Add(int.Parse(bdcstr1[2]));
            }

            foreach (var point in bdc_points)
            {
                int i = points.IndexOf(point);
                bdc_value[i * 3 + 0] = bdcs[bdc_points.IndexOf(point) * 3 + 0];
                bdc_value[i * 3 + 1] = bdcs[bdc_points.IndexOf(point) * 3 + 1];
                bdc_value[i * 3 + 2] = bdcs[bdc_points.IndexOf(point) * 3 + 2];
            }
            return bdc_value;
        }

        private List<Point3d> CreatePointList(List<Line> geometry)
        {
            List<Point3d> points = new List<Point3d>();
            foreach (Line line in geometry) //adds point unless it already exists in pointlist
            {
                Point3d tempFrom = new Point3d(Math.Round(line.From.X, 5), Math.Round(line.From.Y, 5), Math.Round(line.From.Z, 5));
                Point3d tempTo = new Point3d(Math.Round(line.To.X, 5), Math.Round(line.To.Y, 5), Math.Round(line.To.Z, 5));

                if (!points.Contains(tempFrom))
                {
                    points.Add(tempFrom);
                }
                if (!points.Contains(tempTo))
                {
                    points.Add(tempTo);
                }
            }
            return points;
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.Calc;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("b4e6e6ea-86b2-46dd-8475-dfa04892a212"); }
        }
    }
}