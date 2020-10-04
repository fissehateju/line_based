using LBDD.Dataplane;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace LBDD.Intilization
{
    public class Operations
    {
        /// <summary>
        /// A function to check whether point P(x, y) lies inside the rectangle formed by p1(x1, y1), p2(x2, y2), p3(x3, y3) and p4(x4, y4)   
        /// </summary>
        /// <param name="p"></param>
        /// <param name="rec1"></param>
        /// <param name="rec2"></param>
        /// <param name="rec3"></param>
        /// <param name="rec4"></param>
        /// <returns></returns>
        public static bool IsPointWithRectangle(Point p, Point p1,Point p2,Point p3,Point p4) 
        {
            // Calculate area of rectangle ABCD  
            double A = GetTriangleArea(p1, p2, p3) + GetTriangleArea(p1, p4, p3);
            // Calculate area of triangle PAB  
            double A1 = GetTriangleArea(p, p1, p2);
            // Calculate area of triangle PBC  
            double A2 = GetTriangleArea(p, p2, p3);
            // Calculate area of triangle PCD  
            double A3 = GetTriangleArea(p, p3, p4);
            // Calculate area of triangle PAD 
            double A4 = GetTriangleArea(p, p1, p4);

            double sum = Math.Round( A1 + A2 + A3 + A4);
           
            return (A == sum);
        }

        /// <summary>
        /// A utility function to calculate area of triangle formed by (x1, y1), (x2, y2) and (x3, y3) 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public static double GetTriangleArea(Point p1, Point p2, Point p3) 
        {
            return Math.Abs((p1.X * (p2.Y - p3.Y) +
                                p2.X * (p3.Y - p1.Y) +
                                p3.X * (p1.Y - p2.Y)) / 2.0);
        }


        public static void DrawPoint(Point i, Brush color)
        {

            Ellipse myEllipse = new Ellipse();
            myEllipse.StrokeThickness = 1;
            myEllipse.Stroke = color;
            myEllipse.Fill = color;
            myEllipse.Width = 1;
            myEllipse.Height = 1;
            myEllipse.Margin = new Thickness(i.X, i.Y, 0, 0);
            PublicParamerters.MainWindow.Canvas_SensingFeild.Children.Add(myEllipse);
        }


        public static void DrawPoint(Point i, Brush color, double raduis)
        {

            Ellipse myEllipse = new Ellipse();
            myEllipse.StrokeThickness = raduis;
            myEllipse.Stroke = color;
            myEllipse.Fill = color;
            myEllipse.Width = raduis;
            myEllipse.Height = raduis;
            myEllipse.Margin = new Thickness(i.X, i.Y, 0, 0);
            PublicParamerters.MainWindow.Canvas_SensingFeild.Children.Add(myEllipse);
        }

        public static double GetAngle(Point i, Point j, Point d)
        {
            System.Windows.Vector u = new System.Windows.Vector(i.X - d.X, i.Y - d.Y);
            System.Windows.Vector v = new System.Windows.Vector(j.X - d.X, j.Y - d.Y);
            double angle = System.Windows.Vector.AngleBetween(u, v);

            return Math.Abs(angle);

        }

        public static Sensor FindInAlist(Sensor sen, List<Sensor> List)
        {
            foreach (Sensor s in List)
            {
                if (s.ID == sen.ID)
                {
                    return sen;
                }
            }
            return null;
        }


        public static bool FindInAlistbool(Sensor sen, List<Sensor> List)
        {
            foreach (Sensor s in List)
            {
                if (s.ID == sen.ID)
                {
                    return true;
                }
            }
            return false;
        }



        /// <summary>
        /// https://www.geeksforgeeks.org/section-formula-point-divides-line-given-ratio/
        /// m close to the start point. 
        /// n close to the end point.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="d1"></param>
        /// <param name="d2"></param>
        /// <returns></returns>
        public static Point GetPointMtoNratio(Point p1, Point p2, double d1, double d2)
        {
            double x = ((d2 * p1.X) + (d1 * p2.X)) / (d1 + d2);
            double y = ((d2 * p1.Y) + (d1 * p2.Y)) / (d1 + d2);
            return new Point(x, y);
        }

        /// <summary>
        /// Given a line starts at ns and ends at dest. 
        /// return the the distance from the point nj to the line.
        /// </summary>
        /// <param name="nj">candidiate to be selected along the line </param>
        /// <param name="ns">line stard point</param>
        /// <param name="dest"></param>
        /// <returns>per distance from the point nj to the line </returns>
        public static double Perpendiculardistance(Point nj, Point ns, Point dest)
        {
            double past = Math.Abs(((dest.Y - ns.Y) * nj.X) - ((dest.X - ns.X) * nj.Y) + (dest.X * ns.Y) - (dest.Y * ns.X));
            double sbDis = DistanceBetweenTwoPoints(ns, dest);
            double perDis = past / sbDis;
            return perDis;


        }
        /// <summary>
        /// the angle bettween the two vectors i->j and  i->d
        /// </summary>
        /// <param name="i">currrent node</param>
        /// <param name="j">candidate</param>
        /// <param name="d"> the distination point</param>
        /// <returns></returns>
        public static double AngleDotProdection(Point i, Point j, Point d)
        {
            double axb = (j.X - i.X) * (d.X - i.X) + (j.Y - i.Y) * (d.Y - i.Y);
            double disMul = DistanceBetweenTwoPoints(i, d) * DistanceBetweenTwoPoints(i, j);
            double angale = Math.Acos(axb / disMul);
            double norAngle = angale / Math.PI;
            return norAngle;
        }


        public static void DrawLine(Point from, Point to)
        {
            Line lin = new Line();
            lin.Stroke = Brushes.Black;
            lin.StrokeThickness = 2;
            lin.X1 = from.X;
            lin.Y1 = from.Y;
            lin.X2 = to.X;
            lin.Y2 = to.Y;
            PublicParamerters.MainWindow.Canvas_SensingFeild.Children.Add(lin);
        }

        public static void DrawLine(Point from, Point to,double StrokeThickness)
        {
            Line lin = new Line();
            lin.Stroke = Brushes.Black;
            lin.StrokeThickness = StrokeThickness;
            lin.X1 = from.X;
            lin.Y1 = from.Y;
            lin.X2 = to.X;
            lin.Y2 = to.Y;
            PublicParamerters.MainWindow.Canvas_SensingFeild.Children.Add(lin);
        }


        public static double DistanceBetweenTwoSensors(Sensor sensor1, Sensor sensor2)
        {
            if (sensor1 != null && sensor2 != null)
            {
                double dx = (sensor1.CenterLocation.X - sensor2.CenterLocation.X);
                dx *= dx;
                double dy = (sensor1.CenterLocation.Y - sensor2.CenterLocation.Y);
                dy *= dy;
                return Math.Sqrt(dx + dy);
            }
            else
            {
                return double.NegativeInfinity;
            }
        }

        public static double DistanceBetweenTwoPoints(Point p1, Point p2)
        {
            double dx = (p1.X - p2.X);
            dx *= dx;
            double dy = (p1.Y - p2.Y);
            dy *= dy;
            return Math.Sqrt(dx + dy);
        }

        /// <summary>
        /// the communication range is overlapped.
        /// 
        /// </summary>
        /// <param name="sensor1"></param>
        /// <param name="sensor2"></param>
        /// <returns></returns>
        public static bool isOverlapped(Sensor sensor1, Sensor sensor2)
        {
            bool re = false;
            double disttance = DistanceBetweenTwoSensors(sensor1, sensor2);
            if (disttance < (sensor1.ComunicationRangeRadius + sensor2.ComunicationRangeRadius))
            {
                re = true;
            }
            return re;
        }

        internal static double DistanceBetweenTwoPoints(Point centerLocation1, object centerLocation2)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// check if j is within the range of i.
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        public static bool isInMySensingRange(Sensor i, Sensor j)
        {
            bool re = false;
            double disttance = DistanceBetweenTwoSensors(i, j);
            if (disttance <= (i.VisualizedRadius))
            {
                re = true;
            }
            return re;
        }

        /// <summary>
        /// commnication=sensing rang*2
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        public static bool isInMyComunicationRange(Sensor i, Sensor j)
        {
            bool re = false;
            double disttance = DistanceBetweenTwoSensors(i, j);
            if (disttance <= (i.ComunicationRangeRadius))
            {
                re = true;
            }
            return re;
        }

        public static double FindNodeArea(double com_raduos)
        {
            return Math.PI * Math.Pow(com_raduos, 2);
        }

        /// <summary>
        /// n!
        /// </summary>
        /// <param name="n"></param>
        /// <returns></returns>
        public static double Factorial(int n)
        {
            long i, fact;
            fact = n;
            for (i = n - 1; i >= 1; i--)
            {
                fact = fact * i;
            }
            return fact;
        }

        /// <summary>
        /// combination 
        /// </summary>
        /// <param name="n"></param>
        /// <param name="k"></param>
        /// <returns></returns>
        public static double Combination(int n, int k)
        {
            if (k == 0 || n == k) return 1;
            if (k == 1) return n;
            int dif = n - k;
            int max = Max(dif, k);
            int min = Min(dif, k);

            long i, bast;
            bast = n;
            for (i = n - 1; i > max; i--)
            {
                bast = bast * i;
            }
            double mack = Factorial(min);
            double x = bast / mack;
            return x;
        }


        private static int Max(int n1,int n2) { if (n1 > n2) return n1; else return n2; }
        private static int Min(int n1, int n2) { if (n1 < n2) return n1; else return n2; } 
    }
}
