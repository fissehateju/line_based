using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Dataplane.PacketRouter;
using LBDD.Energy;
using LBDD.Intilization;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;
using static LBDD.Computations.RandomvariableStream;
using static LBDD.MergedPath.computing.Sorters;

namespace LBDD.MergedPath.computing
{

    public enum CoordinationType { Random, Maximal, Mixed }
    // MP.MergedPath.computing
    public enum EnergyConsumption { Transmit, Recive } // defualt is not used. i 
    public class NetworkOverheadCounter
    {
        FirstOrderRadioModel EnergyModel;
        public NetworkOverheadCounter ()
        {
            EnergyModel = new FirstOrderRadioModel(); // energy model.
        }

        private double ConvertToJoule(double UsedEnergy_Nanojoule) //the energy used for current operation
        {
            double _e9 = 1000000000; // 1*e^-9
            double _ONE = 1;
            double oNE_DIVIDE_e9 = _ONE / _e9;
            double re = UsedEnergy_Nanojoule * oNE_DIVIDE_e9;
            return re;
        }


        /// <summary>
        /// redunduant transmisin
        /// </summary>
        /// <param name="pacekt"></param>
        /// <param name="reciverNode"></param>
        public void RedundantTransmisionCost(Packet pacekt, Sensor reciverNode)
        {
            // logs.
            PublicParamerters.TotalReduntantTransmission += 1;
            double UsedEnergy_Nanojoule = EnergyModel.Receive(128); // preamble packet length.
            double UsedEnergy_joule = ConvertToJoule(UsedEnergy_Nanojoule);
            reciverNode.ResidualEnergy = reciverNode.ResidualEnergy - UsedEnergy_joule;
            pacekt.UsedEnergy_Joule += UsedEnergy_joule;
            PublicParamerters.TotalEnergyConsumptionJoule += UsedEnergy_joule;
            PublicParamerters.TotalWastedEnergyJoule += UsedEnergy_joule;
            reciverNode.MainWindow.Dispatcher.Invoke(() => reciverNode.MainWindow.lbl_Redundant_packets.Content = PublicParamerters.TotalReduntantTransmission);
            reciverNode.MainWindow.Dispatcher.Invoke(() => reciverNode.MainWindow.lbl_Wasted_Energy_percentage.Content = PublicParamerters.WastedEnergyPercentage);
        }


        /// <summary>
        /// this counts the energy consumption, the delay and the hops.
        /// </summary>
        /// <param name="packt"></param>
        /// <param name="enCon"></param>
        /// <param name="sender"></param>
        /// <param name="Reciver"></param>
        public void ComputeOverhead(Packet packt, EnergyConsumption enCon, Sensor sender, Sensor Reciver)
        {
            if (enCon == EnergyConsumption.Transmit)
            {
                // calculate the energy 
                double Distance_M = Operations.DistanceBetweenTwoSensors(sender, Reciver);
                double UsedEnergy_Nanojoule = EnergyModel.Transmit(packt.PacketLength, Distance_M);
                double UsedEnergy_joule = ConvertToJoule(UsedEnergy_Nanojoule);
                sender.ResidualEnergy = sender.ResidualEnergy - UsedEnergy_joule;
                PublicParamerters.TotalEnergyConsumptionJoule += UsedEnergy_joule;
                packt.UsedEnergy_Joule += UsedEnergy_joule;
              //  packt.RoutingDistance += Distance_M;
                packt.Hops += 1;
                double delay = DelayModel.DelayModel.Delay(sender, Reciver, packt);
                packt.Delay += delay;
               
                PublicParamerters.TotalDelayMs += delay;
                PublicParamerters.TotalNumberOfHops += 1;
                PublicParamerters.TotalRoutingDistance += Distance_M;

                if (Settings.Default.SaveRoutingLog)
                {
                    RoutingLog log = new RoutingLog();
                    log.PacketType = packt.PacketType;
                    log.IsSend = true;
                    log.NodeID = sender.ID;
                    log.Operation = "To:" + Reciver.ID;
                    log.Time = DateTime.Now;
                    log.Distance_M = Distance_M;
                    log.UsedEnergy_Nanojoule = UsedEnergy_Nanojoule;
                    log.RemaimBatteryEnergy_Joule = sender.ResidualEnergy;
                    log.PID = packt.PID;
                    sender.Logs.Add(log);
                }

                switch (packt.PacketType)
                {
                    case PacketType.Data:
                        PublicParamerters.EnergyConsumedForDataPackets += UsedEnergy_joule; // data packets.
                        break;
                    default:
                        PublicParamerters.EnergyComsumedForControlPackets += UsedEnergy_joule; // other packets.
                        break;
                }
            }
            else if (enCon == EnergyConsumption.Recive)
            {
                if (Reciver != null)
                {
                    double UsedEnergy_Nanojoule = EnergyModel.Receive(packt.PacketLength);
                    double UsedEnergy_joule = ConvertToJoule(UsedEnergy_Nanojoule);
                    Reciver.ResidualEnergy = Reciver.ResidualEnergy - UsedEnergy_joule;
                    packt.UsedEnergy_Joule += UsedEnergy_joule;
                    PublicParamerters.TotalEnergyConsumptionJoule += UsedEnergy_joule;

                    switch (packt.PacketType)
                    {
                        case PacketType.Data:
                            PublicParamerters.EnergyConsumedForDataPackets += UsedEnergy_joule; // data packets.
                            break;
                        default:
                            PublicParamerters.EnergyComsumedForControlPackets += UsedEnergy_joule; // other packets.
                            break;
                    }
                }
            }

        }

        public void DropPacket(Packet packt, Sensor Reciver, PacketDropedReasons packetDropedReasons)
        {

            PublicParamerters.NumberofDropedPacket += 1;
            packt.PacketDropedReasons = packetDropedReasons;
            packt.isDelivered = false;
            Reciver.MainWindow.lbl_Number_of_Droped_Packet.Content = PublicParamerters.NumberofDropedPacket.ToString();


            PublicParamerters.DropedPacketsList.Add(packt);

            /*
            if (Settings.Default.SavePackets)
                PublicParamerters.FinishedRoutedPackets.Add(packt);
            else
                packt.Dispose();*/
        }


        public void SuccessedDeliverdPacket(Packet packt)
        {
            if (!packt.isDelivered)
            {
                packt.isDelivered = true;
                PublicParamerters.NumberofDeliveredPacket += 1;
                PublicParamerters.MainWindow.lbl_average_delay.Content = PublicParamerters.AverageDelay.ToString();

                if (Settings.Default.SavePackets)
                    PublicParamerters.FinishedRoutedPackets.Add(packt);
                else
                    packt.Dispose();
            }
            else
            {
                MessageBox.Show("Problem....");
            }
        }



        public void Animate(Sensor sender, Sensor Reciver, Packet pck)
        {
            if (Settings.Default.ShowRoutingPaths)
            {
                sender.Animator.StartAnimate(Reciver.ID, pck.PacketType);
            }
        }
       

        public void IncreasePacketsCounter(Sensor packetSource, PacketType type)
        {

           
            packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_num_of_gen_packets.Content = PublicParamerters.NumberofGeneratedPackets, DispatcherPriority.Normal);

            switch (type)
            {
                case PacketType.Data:
                    PublicParamerters.NumberOfDataPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_data_packets.Content = PublicParamerters.NumberOfDataPacket, DispatcherPriority.Normal);
                    break;
                case PacketType.ObtainSinkPosition:
                    PublicParamerters.NumberOfObtainSinkPositionPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_obtian_packets.Content = PublicParamerters.NumberOfObtainSinkPositionPacket, DispatcherPriority.Normal);
                    break;
                case PacketType.ReportSinkPosition:
                    PublicParamerters.NumberOfReportSinkPositionPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_report_packets.Content = PublicParamerters.NumberOfReportSinkPositionPacket, DispatcherPriority.Normal);
                    break;
                case PacketType.ResponseSinkPosition:
                    PublicParamerters.NumberOfResponseSinkPositionPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_response_packets.Content = PublicParamerters.NumberOfResponseSinkPositionPacket, DispatcherPriority.Normal);
                    break;
                case PacketType.ShareSinkPosition:
                    PublicParamerters.NumberOfShareSinkPositionPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_share_packets.Content = PublicParamerters.NumberOfShareSinkPositionPacket, DispatcherPriority.Normal);
                    break;
                case PacketType.VirtualLineConstruction:
                    PublicParamerters.NumberOfDiagonalVirtualLineConstructionPacket += 1;
                    packetSource.MainWindow.Dispatcher.Invoke(() => packetSource.MainWindow.lbl_construction_packets.Content = PublicParamerters.NumberOfDiagonalVirtualLineConstructionPacket, DispatcherPriority.Normal);
                    break;
            }
        }

        public void DisplayRefreshAtReceivingPacket(Sensor Reciver)
        {
            Reciver.MainWindow.Dispatcher.Invoke(() => Reciver.MainWindow.lbl_total_consumed_energy.Content = PublicParamerters.TotalEnergyConsumptionJoule + " (JOULS)", DispatcherPriority.Send);
            Reciver.MainWindow.Dispatcher.Invoke(() => Reciver.MainWindow.lbl_Number_of_Delivered_Packet.Content = PublicParamerters.NumberofDeliveredPacket, DispatcherPriority.Send);
            Reciver.MainWindow.Dispatcher.Invoke(() => Reciver.MainWindow.lbl_sucess_ratio.Content = PublicParamerters.DeliveredRatio, DispatcherPriority.Send);
            Reciver.MainWindow.Dispatcher.Invoke(() => Reciver.MainWindow.lbl_nymber_inQueu.Content = PublicParamerters.InQueuePackets.ToString());
        }


        /// <summary>
        /// the packet should wait in the queue
        /// </summary>
        public void SaveToQueue(Sensor sender, Packet packet)
        {
            sender.WaitingPacketsQueue.Enqueue(packet);
            PublicParamerters.TotalWaitingTime += 1; // total;
            packet.WaitingTimes += 1;
            if (!sender.QueuTimer.IsEnabled)
            {
                sender.QueuTimer.Start();
            }
            if (Settings.Default.ShowRadar) sender.Myradar.StartRadio();
            PublicParamerters.MainWindow.Dispatcher.Invoke(() => sender.Ellipse_indicator.Fill = Brushes.DeepSkyBlue);
            PublicParamerters.MainWindow.Dispatcher.Invoke(() => sender.Ellipse_indicator.Visibility = Visibility.Visible);
        }


        /// <summary>
        /// match one reciver:
        /// </summary>
        /// <param name="coordinationEntries"></param>
        /// <param name="packet"></param>
        /// <returns></returns>
        public Sensor MaximalCoordinate(List<CoordinationEntry> coordinationEntries, Packet packet,double sum)
        {

            // normalized:
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                if (neiEntry.Sensor.ResidualEnergy > 0)
                {
                    double val = neiEntry.Priority;
                    double normalized = val / sum;
                    neiEntry.Priority = normalized;
                }
                else
                {
                    neiEntry.Priority = 0;
                }
            }
            // sort:

            coordinationEntries.Sort(new CoordinationEntrySorter());

            // select forwarders:

            List<CoordinationEntry> Forwarders = new List<CoordinationEntry>();
            int n = coordinationEntries.Count;
            double average = 1 / Convert.ToDouble(n);

            int maxForwarders = Convert.ToInt16(Math.Floor(Math.Sqrt(Math.Sqrt(n)))) - 1;
            int MaxforwardersCount = 0;
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                if (neiEntry.Priority >= average && MaxforwardersCount <= maxForwarders)
                {
                    if (neiEntry.Sensor.ResidualEnergy > 0)
                    {
                        Forwarders.Add(neiEntry);
                        MaxforwardersCount++;
                    }
                }
            }

            // one forwarder:
            // forward:
            Sensor forwarder = null;

            foreach (CoordinationEntry neiEntry in Forwarders)
            {
                if (neiEntry.Sensor.CurrentSensorState == SensorState.Active)
                {
                    if (forwarder == null)
                    {
                        forwarder = neiEntry.Sensor;
                    }
                    else
                    {
                        RedundantTransmisionCost(packet, neiEntry.Sensor);
                    }
                }
            }
            return forwarder;
        }



        public Sensor RandomCoordinate(List<CoordinationEntry> coordinationEntries, Packet packet, double sum)
        {
            // normalized:
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                if (neiEntry.Sensor.ResidualEnergy > 0)
                {
                    double val = neiEntry.Priority;
                    double normalized = val / sum;
                    neiEntry.Priority = normalized;
                }
                else
                {
                    neiEntry.Priority = 0;
                }
            }

            List<CoordinationEntry> Forwarders = new List<CoordinationEntry>();
            int n = coordinationEntries.Count;
            double average = 1 / Convert.ToDouble(n); // this needs to be considered
            int maxForwarders = Convert.ToInt16(Math.Floor(Math.Sqrt(Math.Sqrt(n)))) - 1; // theshold.
            int MaxforwardersCount = 0;
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                if (neiEntry.Priority >= average && MaxforwardersCount <= maxForwarders && neiEntry.Sensor.CurrentSensorState == SensorState.Active)
                {
                    if (neiEntry.Sensor.ResidualEnergy > 0)
                    {
                        Forwarders.Add(neiEntry);
                        MaxforwardersCount++;
                    }
                }
            }

            double dsum = 0;
            for (int i = 0; i < Forwarders.Count; i++)
            {
                dsum += Forwarders[i].Priority;
            }

            foreach (CoordinationEntry neiEntry in Forwarders)
            {
                double val = neiEntry.Priority;
                double normalized = val / dsum;
                neiEntry.Priority = normalized;
            }



            // range:
            for (int i = 0; i < Forwarders.Count; i++)
            {
                if (i == 0)
                {
                    Forwarders[0].MinRange = 0;
                    Forwarders[0].MaxRange = Forwarders[i].Priority;
                }
                else
                {
                    Forwarders[i].MinRange = Forwarders[i - 1].MaxRange;// min
                    Forwarders[i].MaxRange = Forwarders[i - 1].MaxRange + Forwarders[i].Priority; // max
                }
            }

            // one forwarder:
            // forward:
            Sensor forwarder = null;

            double random = UniformRandomVariable.GetDoubleValue(0, 1);

            CoordinationEntry neiEntry1 = null;
            foreach (CoordinationEntry neiEntry in Forwarders)
            {
                if (random >= neiEntry.MinRange && random <= neiEntry.MaxRange)
                {
                    forwarder = neiEntry.Sensor;
                    neiEntry1 = neiEntry;

                    break;
                }
            }


            bool isRemoved = Forwarders.Remove(neiEntry1);
            if (isRemoved)
            {
                foreach (CoordinationEntry neiEntry in Forwarders)
                {
                    RedundantTransmisionCost(packet, neiEntry.Sensor);
                }
            }


            return forwarder;
        }


        public Sensor CoordinateGetMin(List<CoordinationEntry> coordinationEntries, Packet packet, double sum)
        {


            // normalized to 1:
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                double newPr = neiEntry.Priority / sum;
                neiEntry.Priority = newPr;
            }

            // coordinationEntries.Sort(new CoordinationEntrySorter());


            List<CoordinationEntry> Forwarders = new List<CoordinationEntry>();
            int n = coordinationEntries.Count;
            double average = 1 / Convert.ToDouble(n); // this needs to be considered
            int maxForwarders = 1 + Convert.ToInt16(Math.Ceiling(Math.Sqrt(Math.Sqrt(n)))); // theshold.
            int MaxforwardersCount = 0;
            foreach (CoordinationEntry neiEntry in coordinationEntries)
            {
                // take the first (maxForwarders)
                // the smaller priority is better. select the nodes with smaller priority
                if (MaxforwardersCount <= maxForwarders && neiEntry.Priority <= average)
                {
                    Forwarders.Add(neiEntry);
                    MaxforwardersCount++;
                }
            }


            Forwarders.Sort(new CoordinationEntrySorter());

            // one forwarder:
            // forward:
            Sensor forwarder = null;
            foreach (CoordinationEntry neiEntry in Forwarders)
            {
                if (neiEntry.Sensor.CurrentSensorState == SensorState.Active)
                {
                    if (forwarder == null)
                    {
                        forwarder = neiEntry.Sensor;
                    }
                    else
                    {
                        RedundantTransmisionCost(packet, neiEntry.Sensor);
                    }
                }
            }

            return forwarder;
        }



    }
}
