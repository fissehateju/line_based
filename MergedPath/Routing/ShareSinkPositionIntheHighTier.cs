using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using LBDD.MergedPath.DiagonalVirtualLine;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;

namespace LBDD.MergedPath.Routing
{
    public enum PacketDirection { OmiDirection, Left, Right, Up, Down }

    /// <summary>
    /// This selection mechanism is repeated until the packet that holds the new position of the mobile sink is received by an DVL node, say n_v which is the closest to the point  V on the diagonal. Then, n_v shares the new position with all DVL nodes via one-hop or multiple hops.
    /// </summary>
    class ShareSinkPositionIntheHighTier
    {
        private LoopMechanizimAvoidance LoopMechan = new LoopMechanizimAvoidance();
        private NetworkOverheadCounter counter;
        public ShareSinkPositionIntheHighTier(Sensor highTierGateWay, SinksAgentsRow reportSinkPositionRow)
        {
            if (highTierGateWay.IsHightierNode)
            {
                counter = new NetworkOverheadCounter();


                Packet upPlacket = GeneragtePacket(highTierGateWay, reportSinkPositionRow, PacketDirection.Up);
                Packet downpacket = GeneragtePacket(highTierGateWay, reportSinkPositionRow, PacketDirection.Down);
               

                SendPacket(highTierGateWay, upPlacket);
                SendPacket(highTierGateWay, downpacket);

                // floood:


                //: SAVE Sink positions.// this is not agent record. becarful here
                highTierGateWay.AddSinkRecordInHighTierNode(reportSinkPositionRow);

            }
        }

        public ShareSinkPositionIntheHighTier()
        {
            counter = new NetworkOverheadCounter();
        }

        public void HandelInQueuPacket(Sensor currentNode, Packet InQuepacket)
        {
            SendPacket(currentNode, InQuepacket);
        }

        private Packet GeneragtePacket(Sensor highTierGateWay, SinksAgentsRow reportSinkPositionRow, PacketDirection direction)
        {
          //  PublicParamerters.NumberofGeneratedPackets += 1;
            Packet pck = new Packet();
            pck.PacketDirection = direction;
            pck.Source = highTierGateWay;
            pck.ReportSinkPosition = reportSinkPositionRow;
            pck.Path = "" + highTierGateWay.ID;
            if (direction == PacketDirection.Down)
            {
                pck.Destination = ClosePointToMe.DownPoint; // point 3 or 4 or medi
            }
            else
            {
                pck.Destination = ClosePointToMe.TopPoint; // 1 or 2

                Operations.DrawPoint(pck.Destination, Brushes.Black, 10);
            }

            pck.TimeToLive = 1; // Convert.ToInt16((Operations.DistanceBetweenTwoPoints(highTierGateWay.CenterLocation, pck.Destination) / (PublicParamerters.CommunicationRangeRadius / 3)));
            pck.PacketType = PacketType.ShareSinkPosition;
            pck.PID = PublicParamerters.NumberofGeneratedPackets;
            counter.IncreasePacketsCounter(highTierGateWay, PacketType.ShareSinkPosition);
            return pck;
        }

        private void SendPacket(Sensor sender, Packet pck)
        {
            if (pck.PacketType == PacketType.ShareSinkPosition)
            {
                sender.SwichToActive();
                List<Sensor> Recivers = Flood(sender, pck);
                if (Recivers != null)
                {
                    foreach (Sensor Reciver in Recivers)
                    {
                        // overhead:
                        counter.ComputeOverhead(pck, EnergyConsumption.Transmit, sender, Reciver);
                        counter.Animate(sender, Reciver, pck);
                        RecivePacket(Reciver, pck);
                    }
                }
            }
        }


        private void RecivePacket(Sensor Reciver, Packet packt)
        {

            if (Reciver == null) // packet is recived.
            {
                counter.SuccessedDeliverdPacket(packt);
                counter.DisplayRefreshAtReceivingPacket(packt.Source);

            }
            else
            {
                packt.Path += ">" + Reciver.ID;
                packt.ReTransmissionTry = 0;
                counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                Reciver.AddSinkRecordInHighTierNode(packt.ReportSinkPosition); // keep track of the sink position


                if (packt.Hops <= packt.TimeToLive)
                {
                    if (packt.PacketDirection == PacketDirection.Up)
                    {
                        Packet upPlacket = GeneragtePacket(Reciver, packt.ReportSinkPosition, PacketDirection.Up);
                        SendPacket(Reciver, upPlacket);
                    }
                    else
                    {
                        // down:
                        Packet downPlacket = GeneragtePacket(Reciver, packt.ReportSinkPosition, PacketDirection.Down);
                        SendPacket(Reciver, downPlacket);
                    }
                }
            }
        }


        public List<Sensor> Flood(Sensor ni, Packet packt)
        {

            // sensor ni whill send the packet to its all neighbors in the end dairection in one direction
            List<Sensor> re = new List<Sensor>();
            if (ni.ResidualEnergyPercentage > 0)
            {
                foreach (Sensor nj in ni.NeighborsTable)
                {
                    if (nj.ResidualEnergyPercentage > 0)
                    {

                        if (nj.IsHightierNode)
                        {
                            double Norangle = Operations.AngleDotProdection(ni.CenterLocation, nj.CenterLocation, packt.Destination);
                            Console.WriteLine("ni= " + ni.ID + " nj= " + nj.ID + " Norangle= " + Norangle  +" Dir= "+ packt.PacketDirection);
                            if (Double.IsNaN(Norangle) || Norangle <= 0.461)
                            {
                                re.Add(nj);
                            }
                        }
                    }
                }
            }

            if (re.Count == 0) return null;
            else return re;


        }


    }
}
