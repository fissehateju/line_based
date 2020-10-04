using LBDD.Dataplane;
using LBDD.Intilization;
using LBDD.MergedPath.Routing;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace LBDD.MergedPath.DiagonalVirtualLine
{

    public class ClosePointToMe
    {
        /// <summary>
        /// SendreY is the node which want to send the paxck
        /// </summary>
        /// <param name="SendreY"></param>
        /// <returns></returns>
        public static Point PerPendicaularPoint(Sensor sender)
        {
            double hs = PublicParamerters.NetworkSquareSideLength / 2;
            double hl = Settings.Default.LineWidth / 2;
            double x = hs + hl;

            return new Point(x, sender.CenterLocation.Y);
        }


        public static Point TopPoint
        {
            get
            {
                double s = PublicParamerters.NetworkSquareSideLength;
                double hl = Settings.Default.LineWidth;
                return new Point((s / 2) + (hl / 2), -100);

            }
        }

        public static Point DownPoint
        {
            get
            {
                double s = PublicParamerters.NetworkSquareSideLength;
                double hs = s / 2;
                return new Point(hs, s+100);
            }
        }
    }

    public class LBDDLINEPOINTS
    {
        List<Point> rec = new List<Point>();
        double hs = PublicParamerters.NetworkSquareSideLength / 2;
        double hl = Settings.Default.LineWidth / 2;

        Point point1; // left- top
        Point point2; // right top
        Point point3; // right bottom
        Point point4; // left bottom.
       

        public LBDDLINEPOINTS()
        {
            point1 = new Point(hs - hl, 0);
            point2 = new Point(hs + hl, 0);
            point3 = new Point(hs + hl, PublicParamerters.NetworkSquareSideLength);
            point4 = new Point(hs - hl, PublicParamerters.NetworkSquareSideLength);
           
            rec.Add(point1);
            rec.Add(point2);
            rec.Add(point3);
            rec.Add(point4);
        }


        public List<Point> GetPoints
        {
            get
            {
                return rec;
            }
        }


        public Point GetPoint1
        {
            get
            {
                return point1;
            }
        }

        public Point GetPoint2
        {
            get
            {
                return point2;
            }
        }


        public Point GetPoint3
        {
            get
            {
                return point3;
            }
        }
        public Point GetPoint4
        {
            get
            {
                return point4;
            }
        }
    }


    /// <summary>
    /// DVL consists of a one-node-width strip of nodes along the diagonal of square or a rectangular as shown in Figure 6(a). Beside the benefits of one-node-width structure as discussed in Section 2, we used the diagonal line for two more reasons. First, the construction of DVL is easier, and  it costs a linear overhead O(n), lower than the cost required to construct a ring which costs O(n log n) if Graham scan is used, or in the worst case requires O(n^2) if Gift wrapping algorithm is used. Second, the average distance to the diagonal is smaller than that to the circumference of the ring, mathematically proven in Section 5. This means the request and response paths are shorter to the diagonal line than that to the circumference of the ring, which in turn reduces the delivery delay and minimizes the energy consumption.
    /// </summary>
    public class LineConstruction
    {

       

        List<Sensor> network;
        public LineConstruction(List<Sensor> sensors, Canvas mycanvas)
        {
            network = sensors;

            LBDDLINEPOINTS bDDLINEPOINTS = new LBDDLINEPOINTS();

            Operations.DrawPoint(bDDLINEPOINTS.GetPoint1, Brushes.Red, 2);
            Operations.DrawPoint(bDDLINEPOINTS.GetPoint2, Brushes.Green, 2);
            Operations.DrawPoint(bDDLINEPOINTS.GetPoint3, Brushes.Brown, 2);
            Operations.DrawPoint(bDDLINEPOINTS.GetPoint4, Brushes.Blue, 2);




            Operations.DrawLine(bDDLINEPOINTS.GetPoint1, bDDLINEPOINTS.GetPoint4,0.3); //
            Operations.DrawLine(bDDLINEPOINTS.GetPoint2, bDDLINEPOINTS.GetPoint3,0.3); // 




            Sensor starter = GetStarterNode(sensors, bDDLINEPOINTS.GetPoints);


           LineConstructionMessage ShareLocations = new LineConstructionMessage(starter, bDDLINEPOINTS.GetPoints, mycanvas);



        }

        public Sensor GetStarterNode(List<Sensor> network, List<Point> Rec)
        {
            // take the points which has the loweest y value and within the the zone.
            double minY = double.MaxValue;
            Sensor start = null;
            foreach (Sensor sen in network)
            {
                if (Operations.IsPointWithRectangle(sen.CenterLocation, Rec[0], Rec[1], Rec[2], Rec[3]))
                {
                    // inside: 
                    if (sen.CenterLocation.Y < minY)
                    {
                        minY = sen.CenterLocation.Y;
                        start = sen;
                    }
                }
            }

            return start;
        }



        

       








    }
}
