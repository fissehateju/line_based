using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Dataplane.PacketRouter;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;

namespace LBDD.MergedPath.Routing
{
    /// <summary>
    /// Taking the diagonal line y-x=0 with two known points W(x_1,y_1 ) and Q(x_2,y_2), the DVL nodes are decentrally found by the following mechanism. Given the closest node, denoted by n_i, to the diagonal point (x_1,y_1 ). The node n_i nominates one of its neighbors, say n_j, if n_j satisfies two conditions, expressed by Eq.(12). First, the node n_j has the smallest point-to-line distance, denoted by ψ_(i,j), to the diagonal line. Second, The normalized angle from n_i to n_j with respect to point W, denoted by χ_(i,j), is smaller or equal to 〖0.5〗^°. The node n_j with χ_(i,j)>〖0.5〗^°are ignored since it is located behind the node n_i. We used L to denote the set of DVL nodes. The lower bound of |L| is computed through dividing the diagonal’s length by the half of communication range, |L|=(2√2 s)/δ.
    /// </summary>
    public class LineConstructionMessage
    {
        private LoopMechanizimAvoidance LoopMechan = new LoopMechanizimAvoidance();
        private Canvas can;
        List<Point> _rec;
        private NetworkOverheadCounter counter;
        /// <summary>
        /// Sensor anchor1, Sensor anchor2 are the two points of the diagonal line.
        /// </summary>
        /// <param name="anchor1"></param>
        /// <param name="anchor2"></param>
        /// <param name="canvas"></param>
        public LineConstructionMessage(Sensor starterNode, List<Point> rec, Canvas canvas)
        {
            if (starterNode != null)
            {
                _rec = rec;
                can = canvas;
                counter = new NetworkOverheadCounter();
                starterNode.IsHightierNode = true; // this is the first node
                starterNode.Ellipse_nodeTypeIndicator.Fill = Brushes.LightSlateGray; // 
                Packet pck = GeneratePacket(starterNode, rec); // 
                SendPacket(starterNode, pck); // start from the node 1
            }
        }

        /// <summary>
        /// generate a DiagonalVirtualLineConstruction packet.
        /// </summary>
        /// <param name="scr">anchor1</param>
        /// <param name="des">anchor2</param>
        /// <returns></returns>
        private Packet GeneratePacket(Sensor scr, List<Point> rec)
        {
            //PublicParamerters.NumberofGeneratedPackets += 1; we will not count this packet for construction.
            Packet pck = new Packet();
            pck.Source = scr;
            pck.Path = "" + scr.ID;
            pck.Destination = rec[3]; // we take the point at the bottom as ref.
            pck.PacketType = PacketType.VirtualLineConstruction;
            // pck.PID = PublicParamerters.NumberofGeneratedPackets;
            pck.TimeToLive = System.Convert.ToInt16((Operations.DistanceBetweenTwoPoints(scr.CenterLocation, rec[3]) / (PublicParamerters.CommunicationRangeRadius / 3)));
            counter.IncreasePacketsCounter(scr, PacketType.VirtualLineConstruction);

            return pck;
        }


        private void SendPacket(Sensor sender, Packet pck)
        {
            if (pck.PacketType == PacketType.VirtualLineConstruction)
            {
                // neext hope:
                List<Sensor> Recivers = Flood(sender, pck);
                if (Recivers != null)
                {
                    foreach (Sensor Reciver in Recivers)
                    {
                        // overhead:
                        Reciver.Ellipse_nodeTypeIndicator.Fill = Brushes.LightSlateGray; // 
                        counter.ComputeOverhead(pck, EnergyConsumption.Transmit, sender, Reciver);
                        counter.Animate(sender, Reciver, pck);
                        RecivePacket(Reciver, pck);
                    }
                }
            }
        }

        private void RecivePacket(Sensor Reciver, Packet packt)
        {
            if (LoopMechan.isLoop(packt))
            {
                counter.DropPacket(packt, Reciver, PacketDropedReasons.Loop);
            }
            else
            {
                // flooooding.
                if (Reciver.IsHightierNode == false)
                {
                    // Recivece the packe and send it.
                    counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                    Reciver.IsHightierNode = true; // let each node send once.
                    counter.DisplayRefreshAtReceivingPacket(Reciver);
                    SendPacket(Reciver, packt);
                }
            }
        }

        /// <summary>
        /// Eq.(12)
        /// </summary>
        /// <param name="ni"></param>
        /// <returns></returns>
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
                        // in the the right direction
                        bool withinTheLine = Operations.IsPointWithRectangle(nj.CenterLocation, _rec[0], _rec[1], _rec[2], _rec[3]);
                        if (withinTheLine)
                        {
                            if (!nj.IsHightierNode)
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
