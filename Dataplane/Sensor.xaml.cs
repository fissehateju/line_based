using LBDD.Intilization;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using LBDD.ui;
using LBDD.Properties;
using System.Windows.Threading;
using LBDD.ui.conts;
using LBDD.Dataplane.NOS;
using LBDD.MergedPath.computing;
using LBDD.NetAnimator;
using LBDD.MergedPath.Routing;

namespace LBDD.Dataplane
{
    public enum SensorState { intalized, Active, Sleep } // defualt is not used. i 
   


    /// <summary>
    /// Interaction logic for Node.xaml
    /// </summary>
    public partial class Sensor : UserControl
    {

        #region MP
        public bool IsHightierNode { get; set; } // true means this node is high tier node.
        public bool IsAnAgent { get; set; } // is the node an agent.


        public NetworkOverheadCounter counter = new NetworkOverheadCounter();
        public MyNetAnimator Animator;

        /// <summary>
        /// this is to be used as an agent record only.
        /// the agent save this one.
        /// </summary>
        private List<SinksAgentsRow> SinksAgentsList = new List<SinksAgentsRow>(); // in case the node is an agent.

        public RecoveryRow RecoveryRow;// for recovery.

        /// <summary>
        /// This is to be used When high tier node response to the source. The high tier node save the records of sinks in this list.
        /// </summary>
        private  List<SinksAgentsRow> HighTierKeepRecordsOFSinksLocations = new List<SinksAgentsRow>(); // in case the node is high tier node. the node save the records of the sinks.

        //-- clusring items:
        public bool IsClustered = false; // this is used when clustering the sinks. The agent of the sink is representing the sink.
         
       

        #endregion
        #region Commone parameters.

        public Radar Myradar; 
        public List<Arrow> MyArrows = new List<Arrow>();
        public MainWindow MainWindow { get; set; } // the mian window where the sensor deployed.
        public static double SR { get; set; } // the radios of SENSING range.
        public double SensingRangeRadius { get { return SR; } }
        public static double CR { get; set; }  // the radios of COMUNICATION range. double OF SENSING RANGE
        public double ComunicationRangeRadius { get { return CR; } }
        public double BatteryIntialEnergy; // jouls // value will not be changed
        private double _ResidualEnergy; //// jouls this value will be changed according to useage of battery
        public BoXMAC Mac { get; set; } // the mac protocol for the node.
        public SensorState CurrentSensorState { get; set; } // state of node.
        public List<RoutingLog> Logs = new List<RoutingLog>();
        public List<Sensor> NeighborsTable = null; // neighboring table.
        public int ID { get; set; } // the ID of sensor.
        private DispatcherTimer SendPacketTimer = new DispatcherTimer();// 
        public DispatcherTimer QueuTimer = new DispatcherTimer();// to check the packets in the queue right now.
        public Queue<Packet> WaitingPacketsQueue = new Queue<Packet>(); // packets queue.


        /// <summary>
        /// CONFROM FROM NANO NO JOUL
        /// </summary>
        /// <param name="UsedEnergy_Nanojoule"></param>
        /// <returns></returns>
        


       
        public Sensor(int nodeID)
        {
            InitializeComponent();
            //: sink is diffrent:
            if (IsAnAgent==true)
                BatteryIntialEnergy = PublicParamerters.BatteryIntialEnergyForSink; // the value will not be change
            else
                BatteryIntialEnergy = PublicParamerters.BatteryIntialEnergy;
           
            
            ResidualEnergy = BatteryIntialEnergy;// joules. intializing.
            Prog_batteryCapacityNotation.Value = BatteryIntialEnergy;
            Prog_batteryCapacityNotation.Maximum = BatteryIntialEnergy;
            lbl_Sensing_ID.Content = nodeID;
            ID = nodeID;
            QueuTimer.Interval = PublicParamerters.QueueTime;
            QueuTimer.Tick += DeliveerPacketsInQueuTimer_Tick;
            //:

            SendPacketTimer.Interval = TimeSpan.FromSeconds(1);
            Animator = new MyNetAnimator(this);

           
        }

       

        private void UserControl_MouseLeave(object sender, MouseEventArgs e)
        {
            

        }

        /// <summary>
        /// hide all arrows.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void UserControl_MouseEnter(object sender, MouseEventArgs e)
        {
            
            if(HighTierKeepRecordsOFSinksLocations.Count>0)
            {
                string str = "";
                foreach(SinksAgentsRow row in HighTierKeepRecordsOFSinksLocations)
                {
                    str += "Agent ID=" + row.AgentNode.ID + "  Sink ID=" + row.Sink.ID + "\r\n";
                }

                ToolTip = new Label() { Content = str };
            }
            else
            {
                ToolTip = new Label() { Content = "No AGENT" };
            }
            

        }

        

        private void UserControl_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
           
        }

       

        public int ComputeMaxHopsUplink
        {
            get
            {
               
                return 10;
            }
        }

        public int ComputeMaxHopsDownlink(Sensor endNode)
        {
            return 10;
        }




        #region send data: /////////////////////////////////////////////////////////////////////////////


        public void IdentifySourceNode(Sensor source)
        {
            if (Settings.Default.ShowAnimation)
            {
                Action actionx = () => source.Ellipse_indicator.Visibility = Visibility.Visible;
                Dispatcher.Invoke(actionx);

                Action actionxx = () => source.Ellipse_indicator.Fill = Brushes.Yellow;
                Dispatcher.Invoke(actionxx);
            }
        }

        public void UnIdentifySourceNode(Sensor source)
        {
            if (Settings.Default.ShowAnimation)
            {
                Action actionx = () => source.Ellipse_indicator.Visibility = Visibility.Hidden;
                Dispatcher.Invoke(actionx);

                Action actionxx = () => source.Ellipse_indicator.Fill = Brushes.Transparent;
                Dispatcher.Invoke(actionxx);
            }
        }


        /// <summary>
        /// in JOULE
        /// </summary>
        public double ResidualEnergy // jouls this value will be changed according to useage of battery
        {
            get { return _ResidualEnergy; }
            set
            {
                _ResidualEnergy = value;
                Prog_batteryCapacityNotation.Value = _ResidualEnergy;
            }
        } //@unit(JOULS);


        /// <summary>
        /// 0%-100%
        /// </summary>
        public double ResidualEnergyPercentage
        {
            get { return (ResidualEnergy / BatteryIntialEnergy) * 100; }
        }
        /// <summary>
        /// visualized sensing range and comuinication range
        /// </summary>
        public double VisualizedRadius
        {
            get { return Ellipse_Sensing_range.Width / 2; }
            set
            {
                // sensing range:
                Ellipse_Sensing_range.Height = value * 2; // heigh= sen rad*2;
                Ellipse_Sensing_range.Width = value * 2; // Width= sen rad*2;
                SR = VisualizedRadius;
                CR = SR * 2; // comunication rad= sensing rad *2;

                // device:
                Device_Sensor.Width = value * 4; // device = sen rad*4;
                Device_Sensor.Height = value * 4;
                // communication range
                Ellipse_Communication_range.Height = value * 4; // com rang= sen rad *4;
                Ellipse_Communication_range.Width = value * 4;

                // battery:
                Prog_batteryCapacityNotation.Width = 8;
                Prog_batteryCapacityNotation.Height = 2;
            }
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
                Point p = new Point(x + CR, y + CR);
                return p;
            }
        }

        bool StartMove = false; // mouse start move.
        private void Device_Sensor_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (Settings.Default.IsIntialized == false)
            {
                if (e.LeftButton == MouseButtonState.Pressed)
                {
                    System.Windows.Point P = e.GetPosition(MainWindow.Canvas_SensingFeild);
                    P.X = P.X - CR;
                    P.Y = P.Y - CR;
                    Position = P;
                    StartMove = true;
                }
            }
        }

        private void Device_Sensor_MouseMove(object sender, MouseEventArgs e)
        {
            if (Settings.Default.IsIntialized == false)
            {
                if (StartMove)
                {
                    System.Windows.Point P = e.GetPosition(MainWindow.Canvas_SensingFeild);
                    P.X = P.X - CR;
                    P.Y = P.Y - CR;
                    this.Position = P;


                    
                }
            }
        }

       

        private void Device_Sensor_MouseUp(object sender, MouseButtonEventArgs e)
        {
            StartMove = false;
        }




        private void Prog_batteryCapacityNotation_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {

            double val = ResidualEnergyPercentage;
            if (val <= 0)
            {

                PublicParamerters.IsNetworkDied = true;
                // dead certificate:
                ExpermentsResults.Lifetime.DeadNodesRecord recod = new ExpermentsResults.Lifetime.DeadNodesRecord();
                recod.DeadAfterPackets = PublicParamerters.NumberofGeneratedPackets;
                recod.DeadOrder = PublicParamerters.DeadNodeList.Count + 1;
                double deadAt = PublicParamerters.SimulationTime;
                recod.DeadAtSecond = deadAt;
                recod.DeadNodeID = ID;

                PublicParamerters.DeadNodeList.Add(recod);

                Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col0));
                Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col0));


                if (Settings.Default.StopeWhenFirstNodeDeid)
                {
                    MainWindow.TimerCounter.Stop();
                    MainWindow.RandomSelectSourceNodesTimer.Stop();
                    Settings.Default.StopSimlationWhen = PublicParamerters.SimulationTime;
                    MainWindow.top_menu.IsEnabled = true;
                    Settings.Default.Save();


                    // stop the sinks.
                    MainWindow.StopSinks();


                }


            }
            if (val >= 1 && val <= 9)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col1_9)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col1_9)));
            }

            if (val >= 10 && val <= 19)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col10_19)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col10_19)));
            }

            if (val >= 20 && val <= 29)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col20_29)));
                Dispatcher.Invoke(() => Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col20_29))));
            }

            // full:
            if (val >= 30 && val <= 39)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col30_39)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col30_39)));
            }
            // full:
            if (val >= 40 && val <= 49)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col40_49)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col40_49)));
            }
            // full:
            if (val >= 50 && val <= 59)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col50_59)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col50_59)));
            }
            // full:
            if (val >= 60 && val <= 69)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col60_69)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col60_69)));
            }
            // full:
            if (val >= 70 && val <= 79)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col70_79)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col70_79)));
            }
            // full:
            if (val >= 80 && val <= 89)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col80_89)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col80_89)));
            }
            // full:
            if (val >= 90 && val <= 100)
            {
                Dispatcher.Invoke(() => Prog_batteryCapacityNotation.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col90_100)));
                Dispatcher.Invoke(() => Ellipse_battryIndicator.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(BatteryLevelColoring.col90_100)));
            }
        }

        #endregion






        /// <summary>
        /// 
        /// </summary>
        public void SwichToActive()
        {
            Mac.SwichToActive();

        }

        /// <summary>
        /// 
        /// </summary>
        private void SwichToSleep()
        {
            Mac.SwichToSleep();
        }












        /// <summary>
        ///  select this node as a source and let it 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btn_send_packet_MouseDown(object sender, MouseButtonEventArgs e)
        {

            ObtainSinkFreshPositionMessage ob = new ObtainSinkFreshPositionMessage(this);


            /*
            double hs = PublicParamerters.NetworkSquareSideLength / 2;
            double hl = Settings.Default.LineWidth / 2;

            List<Point> rec = new List<Point>();

            Point point1 = new Point(hs - hl, 0);
            Point point2 = new Point(hs + hl, 0);
            Point point4 = new Point(hs - hl, PublicParamerters.NetworkSquareSideLength);
            Point  point3 = new Point(hs + hl, PublicParamerters.NetworkSquareSideLength);

            Operations.DrawPoint(point1, Brushes.Red,20);
            Operations.DrawPoint(point2, Brushes.Green,20);
            Operations.DrawPoint(point3, Brushes.Brown, 20);
            Operations.DrawPoint(point4, Brushes.Blue,20);
            


            Operations.DrawLine(MainWindow.Canvas_SensingFeild, point1, point4); //

            Operations.DrawLine(MainWindow.Canvas_SensingFeild, point2, point3); // 

            rec.Add(point1);
            rec.Add(point2);
            rec.Add(point3);
            rec.Add(point4);
           

         bool x=   Operations.IsPointWithRectangle(CenterLocation, rec[0], rec[1], rec[2], rec[3]);

            if (x) Console.WriteLine(">>Sensor:" + ID + " Is inside");
            else Console.WriteLine("<<Sensor:" + ID + " Is outside");
            */
        }


        // try.
        private void DeliveerPacketsInQueuTimer_Tick(object sender, EventArgs e)
        {
            Packet waitingpacket = WaitingPacketsQueue.Dequeue();
            waitingpacket.ReTransmissionTry++;
            if (waitingpacket.ReTransmissionTry <= Settings.Default.ReTransmissionAttemps)
            {
                // Console.WriteLine(waitingpacket.PID + ">>>" + waitingpacket.ReTransmissionTry);
                switch (waitingpacket.PacketType)
                {
                    case PacketType.ReportSinkPosition:
                        new ReportSinkPositionMessage().HandelInQueuPacket(this, waitingpacket);
                        break;
                    case PacketType.ShareSinkPosition:
                        new ShareSinkPositionIntheHighTier().HandelInQueuPacket(this, waitingpacket);
                        break;
                    case PacketType.ObtainSinkPosition:
                        new ObtainSinkFreshPositionMessage().HandelInQueuPacket(this, waitingpacket);
                        break;
                    case PacketType.ResponseSinkPosition:
                        new ResonseSinkPositionMessage().HandelInQueuPacket(this, waitingpacket);
                        break;
                    case PacketType.Data:
                        new DataPacketMessages().HandelInQueuPacket(this, waitingpacket);
                        break;
                }
            }
            else
            {
                // the packet should be droped here:
                counter.DropPacket(waitingpacket, this, PacketDropedReasons.WaitingTime);
            }


            if (WaitingPacketsQueue.Count == 0)
            {
                if (Settings.Default.ShowRadar) Myradar.StopRadio();
                QueuTimer.Stop();
                MainWindow.Dispatcher.Invoke(() => Ellipse_indicator.Fill = Brushes.Transparent);
                MainWindow.Dispatcher.Invoke(() => Ellipse_indicator.Visibility = Visibility.Hidden);

                // dont have data any mode than sleep:
                SwichToSleep();
            }
        
        }



        #endregion


        private void lbl_MouseEnter(object sender, MouseEventArgs e)
        {
            ToolTip = new Label() { Content = "("+ID + ") [ " + ResidualEnergyPercentage + "% ] [ " + ResidualEnergy + " J ]" };
        }

        private void btn_show_routing_log_MouseDown(object sender, MouseButtonEventArgs e)
        {
            
        }

        private void btn_draw_random_numbers_MouseDown(object sender, MouseButtonEventArgs e)
        {
           
        }

        private void Ellipse_center_MouseEnter(object sender, MouseEventArgs e)
        {
            
        }

        private void btn_show_my_duytcycling_MouseDown(object sender, MouseButtonEventArgs e)
        {
           
        }

        private void btn_draw_paths_MouseDown(object sender, MouseButtonEventArgs e)
        {
           
        }

       
         
        private void btn_show_my_flows_MouseDown(object sender, MouseButtonEventArgs e)
        {
           
            ListControl ConMini = new ui.conts.ListControl();
            ConMini.lbl_title.Content = "Mini-Flow-Table";
            //ConMini.dg_date.ItemsSource = MiniFlowTable;


            ListControl ConNei = new ui.conts.ListControl();
            ConNei.lbl_title.Content = "Neighbors-Table";
            ConNei.dg_date.ItemsSource = NeighborsTable;

            UiShowLists win = new UiShowLists();
            win.stack_items.Children.Add(ConMini);
            win.stack_items.Children.Add(ConNei);
            win.Title = "Tables of Node " + ID;
            win.Show();
            win.WindowState = WindowState.Maximized;
        }

        private void btn_send_1_p_each1sec_MouseDown(object sender, MouseButtonEventArgs e)
        {
            SendPacketTimer.Start();
            SendPacketTimer.Tick += SendPacketTimer_Random; // redfine th trigger.
        }



        public void RandomSelectEndNodes(int numOFpACKETS)
        {
            
        }

        private void SendPacketTimer_Random(object sender, EventArgs e)
        {
           
        }

        /// <summary>
        /// i am slected as end node.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Btn_select_me_as_end_node_MouseDown(object sender, MouseButtonEventArgs e)
        {
            
            
        }

        public void SelectMeAsEndNodeAndSendonepacketPer5s_Tick(object sender, EventArgs e)
        {
         //   PublicParamerters.SinkNode.GenerateMultipleControlPackets(1, this);
        }

        #region MergedPaths

        /// <summary>
        /// add or update sink agent record.
        /// agent only. 
        /// </summary>
        /// <param name="newsink"></param>
        public void AddNewAGent(SinksAgentsRow newsink)  
        {
            if (newsink != null)
            {
                // add new record
                SinksAgentsList.Add(newsink);
                //: add notation for being selected as agent:
                Ellipse_nodeTypeIndicator.Fill = Brushes.BurlyWood;
                IsAnAgent = true;
                Ellipse_Communication_range.Visibility = Visibility.Visible;
            }
        }

        /// <summary>
        /// remove sink record from my list
        /// agent only.
        /// </summary>
        /// <param name="newsink"></param>
        /// <returns></returns>
        public bool RemoveFromAgent(SinksAgentsRow newsink)
        {
           bool isRemoved= SinksAgentsList.Remove(newsink);
            if (isRemoved)
            {
                if (SinksAgentsList.Count == 0)
                {
                    //: notation for a 
                    Ellipse_nodeTypeIndicator.Fill = Brushes.Transparent;
                    Ellipse_Communication_range.Visibility = Visibility.Hidden;
                    IsAnAgent = false;
                }
            }

            return isRemoved;
        }



        /// <summary>
        /// this is for the nodes in the high tier node. they save the locations of the sink forever.
        /// </summary>
        /// <param name="newsink"></param>
        public void AddSinkRecordInHighTierNode(SinksAgentsRow newsink) 
        {
            SinksAgentsRow oldone = null;
            foreach (SinksAgentsRow agentsRow in HighTierKeepRecordsOFSinksLocations)
            {
                if (agentsRow.Sink.ID == newsink.Sink.ID)
                {
                    oldone = agentsRow;
                    break;
                }
            }

            //

            if (oldone != null)
            {
                // found: then we update the agent only of the sink only.
                // the sink is not changed.
                oldone.AgentNode = newsink.AgentNode;
            }
            else
            {
                //
                SinksAgentsRow copy = new SinksAgentsRow();
                copy.AgentNode = newsink.AgentNode;
                copy.Sink = newsink.Sink;
                copy.ClosestPointOnTheDiagonal = newsink.ClosestPointOnTheDiagonal;
                HighTierKeepRecordsOFSinksLocations.Add(copy);
            }


        }

        /// <summary>
        /// retrun the list of sinks
        /// the node is high tier node. 
        /// this will be uese to response to the requester source node.
        /// </summary>
        public List<SinksAgentsRow> GetSinksAgentsFromHighTierNode
        {
            get
            {
                return HighTierKeepRecordsOFSinksLocations;
            }
        }


       /// <summary>
       /// Get the sinks which are mine.
       /// the node is low tier node
       /// </summary>
        public List<SinksAgentsRow> GetSinksAgentsList
        {
            get
            {
                return SinksAgentsList;
            }
        }

        

        #endregion





        /*** Vistualize****/

        public void ShowID(bool isVis )
        {
           // if (isVis) { lbl_Sensing_ID.Visibility = Visibility.Visible; lbl_hops_to_sink.Visibility = Visibility.Visible; }
           // else { lbl_Sensing_ID.Visibility = Visibility.Hidden; lbl_hops_to_sink.Visibility = Visibility.Hidden; }
        }

        public void ShowSensingRange(bool isVis)
        {
            if (isVis) Ellipse_Sensing_range.Visibility = Visibility.Visible;
            else Ellipse_Sensing_range.Visibility = Visibility.Hidden;
        }

        public void ShowComunicationRange(bool isVis)
        {
            if (isVis) Ellipse_Communication_range.Visibility = Visibility.Visible;
            else Ellipse_Communication_range.Visibility = Visibility.Hidden;
        }

        public void ShowBattery(bool isVis) 
        {
            if (isVis) Prog_batteryCapacityNotation.Visibility = Visibility.Visible;
            else Prog_batteryCapacityNotation.Visibility = Visibility.Hidden;
        }

        private void btn_update_mini_flow_MouseDown(object sender, MouseButtonEventArgs e)
        {
           // UplinkRouting.UpdateUplinkFlowEnery(this);
        }

        private void Btn_see_distrubutions_MouseDown(object sender, MouseButtonEventArgs e)
        {
           

        }
    }
}
