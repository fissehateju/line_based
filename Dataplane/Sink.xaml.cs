using LBDD.Computations;
using LBDD.Dataplane.PacketRouter;
using LBDD.Intilization;
using LBDD.MergedPath.Routing;
using LBDD.Models.Mobility;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace LBDD.Dataplane
{
    /// <summary>
    /// Interaction logic for Sink.xaml
    /// </summary>
    public partial class Sink : UserControl
    {

        RandomWaypointMobilityModel randomWaypoint;
        public int ID
        {
            get;set;
        }
        public Sink()
        {
            InitializeComponent();

            int id = PublicParamerters.SinkCount + 1;
            lbl_sink_id.Text = id.ToString();
            ID = id;
            PublicParamerters.MainWindow.mySinks.Add(this);

            if (Settings.Default.IsMobileSink)
            {
                SetMobility();
            }

            Width = 10;
            Height = 10;
            RandomAgentAtInitialization();

        }


        /// <summary>
        /// set the mobility model
        /// </summary>
        public void SetMobility()
        {
            randomWaypoint = new RandomWaypointMobilityModel(this, PublicParamerters.NetworkSquareSideLength, PublicParamerters.NetworkSquareSideLength);
            randomWaypoint.StartMove();
        }

        public void StopMoving()
        {
            if (randomWaypoint != null) randomWaypoint.StopMoving();
        }
        /// <summary>
        /// Real postion of object.
        /// </summary>
        public Point Position
        {
            get
            {
                double x = Margin.Left;
                double y = Margin.Top;
                Point p = new Point(x, y);
                return p;
            }
            set
            {
                Point p = value;
                Margin = new Thickness(p.X, p.Y, 0, 0);
            }
        }

        /// <summary>
        /// center location of node.
        /// </summary>
        public Point CenterLocation
        {
            get
            {
                double x = Margin.Left;
                double y = Margin.Top;
                Point p = new Point(x, y);
                return p;
            }
        }

        /// <summary>
        /// report the new position of the sink.
        /// </summary>
        public void ReportMyPosition()
        {
            if (MySinksAgentsRow != null)
            {
                if (MySinksAgentsRow.AgentNode != null)
                {
                    ReportSinkPositionMessage rep = new ReportSinkPositionMessage(MySinksAgentsRow);
                }
            }
        }

        public SinksAgentsRow MySinksAgentsRow { get; set; }

        /// <summary>
        /// intailization:
        /// </summary>
        public void RandomAgentAtInitialization()
        {
            int count = PublicParamerters.MainWindow.myNetWork.Count;
            if (count > 0)
            {
                int index;
                if (Settings.Default.SinksStartAtNetworkCenter)
                    index = 0; // all sinks started node with ID=0;
                else
                    index = RandomvariableStream.UniformRandomVariable.GetIntValue(0, count - 1); // random sink location

                Sensor agent = PublicParamerters.MainWindow.myNetWork[index];
                SinksAgentsRow sinksAgentsRow = new SinksAgentsRow();
                sinksAgentsRow.AgentNode = agent;
                sinksAgentsRow.Sink = this;
                agent.AddNewAGent(sinksAgentsRow);
                MySinksAgentsRow = sinksAgentsRow;
                Position = agent.CenterLocation;
                ReportMyPosition();
            }
        }

        /// <summary>
        /// check if the distance almost out.
        /// 
        /// </summary>
        /// <returns></returns>
        public bool AlmostOutOfMyAgent()
        {
            if (MySinksAgentsRow != null)
            {
                if (MySinksAgentsRow.AgentNode != null)
                {
                    double dis = Operations.DistanceBetweenTwoPoints(CenterLocation, MySinksAgentsRow.AgentNode.CenterLocation);
                    if (dis >= (PublicParamerters.CommunicationRangeRadius * 0.7))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        public void ReselectAgentNode()
        {
            double mindis = double.MaxValue;
            Sensor newAgent = null;
            foreach(Sensor sj in MySinksAgentsRow.AgentNode.NeighborsTable)
            {
                double curDis = Operations.DistanceBetweenTwoPoints(CenterLocation, sj.CenterLocation);
                if (curDis < PublicParamerters.CommunicationRangeRadius)
                {
                    if (curDis < mindis)
                    {
                        mindis = curDis;
                        newAgent = sj;
                    }
                }
            }

            // found:
            if (newAgent != null)
            {
                // Prev one:
                Sensor prevAgent = MySinksAgentsRow.AgentNode;
                bool preRemoved = prevAgent.RemoveFromAgent(MySinksAgentsRow);

                if (preRemoved)
                {
                    // set the new one:
                    SinksAgentsRow newsinksAgentsRow = new SinksAgentsRow();
                    newsinksAgentsRow.AgentNode = newAgent;
                    newsinksAgentsRow.Sink = this;
                    newAgent.AddNewAGent(newsinksAgentsRow);
                    MySinksAgentsRow = newsinksAgentsRow;
                    Console.WriteLine("Sink: " + ID + " re-selected " + newAgent.ID + " as new agent. Prev. Agent: " + prevAgent.ID);
                    ReportMyPosition();
                }
                else
                {
                    MessageBox.Show("sink->ReselectAgentNode()-> preRemoved=false.");
                }
            }
            else
            {
                Console.WriteLine("Sink: " + ID + "Out of network and has no agent.");
                // use the same prev agent:
                Position = MySinksAgentsRow.AgentNode.CenterLocation;
                
            }
        }



        private void UserControl_SizeChanged(object sender, SizeChangedEventArgs e)
        {

           bool imOut= AlmostOutOfMyAgent();
            if (imOut)
            {
                // reselect:
                ReselectAgentNode();
            }
            else
            {
                // do no thing.
            }
        }
    }
}
