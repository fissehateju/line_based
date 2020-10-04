using LBDD.Comuting.Routing;
using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Dataplane.PacketRouter;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using LBDD.Properties;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;

namespace LBDD.MergedPath.Routing
{
    /// <summary>
    /// from a hightier node to a given source
    /// </summary>
    class ResonseSinkPositionMessage
    {
        private LoopMechanizimAvoidance LoopMechan = new LoopMechanizimAvoidance();
        private NetworkOverheadCounter counter;

        /// <summary>
        ///  the hightierNode should response to the lowtiernode. 
        ///  here the respnse should contain all sinks.
        /// </summary>
        /// <param name="hightierNode"></param>
        /// <param name="sourceNode"></param>
        public ResonseSinkPositionMessage(Sensor hightierNode, Sensor sourceNode)
        {
            counter = new NetworkOverheadCounter();
            // the hightierNode=lowtiernode --> means that the high tier node itself has data to send.
            if (hightierNode.ID == sourceNode.ID)
            {
                // here high tier has data to send.
                Packet responspacket = GeneragtePacket(hightierNode, hightierNode, null,true); // fack.
                counter.SuccessedDeliverdPacket(responspacket); // count it as delivered
                PreparDataTransmission(hightierNode, responspacket);
            }
            else
            {
                Packet responspacket = GeneragtePacket(hightierNode, sourceNode, null, true);
                SendPacket(hightierNode, responspacket);
            }
        }

        /// <summary>
        /// the recovery process. 
        /// the response from the hightierNode to the lowtiernode should include the SinkIDs only but not all sinks.
        /// </summary>
        /// <param name="hightierNode"></param>
        /// <param name="lowtiernode"></param>
        /// <param name="SinkIDs"></param>
        public ResonseSinkPositionMessage(Sensor hightierNode, Sensor lowtiernode, List<int> SinkIDs)
        {
            counter = new NetworkOverheadCounter();
            // the hightierNode=lowtiernode --> means that the high tier node itself has data to send.
            if (hightierNode.ID == lowtiernode.ID)
            {
                // here high tier has data to send.
                Packet responspacket = GeneragtePacket(hightierNode, hightierNode, SinkIDs,false);
                PreparDataTransmission(hightierNode, responspacket); 
            }
            else
            {
                Packet responspacket = GeneragtePacket(hightierNode, lowtiernode, SinkIDs, true);
                SendPacket(hightierNode, responspacket);
            }
        }

        public Packet GeneragtePacket(Sensor hightierNode, Sensor lowtiernode, List<int> SinkIDs, bool IncreasePid)
        {
            if (IncreasePid)
            {
                if (SinkIDs == null)
                {
                    // normal ResponseSinkPosition:
                    PublicParamerters.NumberofGeneratedPackets += 1;
                    Packet pck = new Packet();
                    pck.SinkIDsNeedsRecovery = null;
                    pck.Source = hightierNode;
                    pck.Path = "" + hightierNode.ID;
                    pck.Destination = lowtiernode.CenterLocation; // has no destination.
                    pck.PacketType = PacketType.ResponseSinkPosition;
                    pck.PID = PublicParamerters.NumberofGeneratedPackets;
                    pck.SinksAgentsList = CopyAllSinks(hightierNode.GetSinksAgentsFromHighTierNode); // normal packet. not recovery packet
                    double dis = Operations.DistanceBetweenTwoPoints(hightierNode.CenterLocation, pck.Destination);
                    pck.TimeToLive = 5 + Convert.ToInt16(dis / (PublicParamerters.CommunicationRangeRadius / 2));
                    counter.IncreasePacketsCounter(hightierNode, PacketType.ResponseSinkPosition);
                    return pck;
                }
                else
                {
                    // recovery packet: that is to say , only few sinks are reqiured. not need to respnse by all sinks.
                    PublicParamerters.NumberofGeneratedPackets += 1;
                    Packet pck = new Packet();
                    pck.SinkIDsNeedsRecovery = SinkIDs;
                    pck.Source = hightierNode;
                    pck.Path = "" + hightierNode.ID;
                    pck.Destination = lowtiernode.CenterLocation; // has no destination.
                    pck.PacketType = PacketType.ResponseSinkPosition;
                    pck.PID = PublicParamerters.NumberofGeneratedPackets;
                    pck.SinksAgentsList = CopyFewSinks(hightierNode.GetSinksAgentsFromHighTierNode, SinkIDs); // needs recovery
                    double dis = Operations.DistanceBetweenTwoPoints(hightierNode.CenterLocation, pck.Destination);
                    pck.TimeToLive = 5 + Convert.ToInt16(dis / (PublicParamerters.CommunicationRangeRadius / 2));
                    counter.IncreasePacketsCounter(hightierNode, PacketType.ResponseSinkPosition);
                    return pck;
                }
            }
            else
            {
                // fack:
                if (SinkIDs == null)
                {
                    // normal ResponseSinkPosition:
                    // PublicParamerters.NumberofGeneratedPackets += 1;
                    Packet pck = new Packet();
                    pck.SinkIDsNeedsRecovery = null;
                    pck.Source = hightierNode;
                    pck.Path = "" + hightierNode.ID;
                    pck.Destination = lowtiernode.CenterLocation; // has no destination.
                    pck.PacketType = PacketType.ResponseSinkPosition;
                    pck.PID = PublicParamerters.NumberofGeneratedPackets;
                    pck.SinksAgentsList = CopyAllSinks(hightierNode.GetSinksAgentsFromHighTierNode); // normal packet. not recovery packet
                    double dis = Operations.DistanceBetweenTwoPoints(hightierNode.CenterLocation, pck.Destination);
                    pck.TimeToLive = 5 + Convert.ToInt16(dis / (PublicParamerters.CommunicationRangeRadius / 2));
                    counter.IncreasePacketsCounter(hightierNode, PacketType.ResponseSinkPosition);
                    return pck;
                }
                else
                {
                    // recovery packet: that is to say , only few sinks are reqiured. not need to respnse by all sinks.
                    // PublicParamerters.NumberofGeneratedPackets += 1;
                    Packet pck = new Packet();
                    pck.SinkIDsNeedsRecovery = SinkIDs;
                    pck.Source = hightierNode;
                    pck.Path = "" + hightierNode.ID;
                    pck.Destination = lowtiernode.CenterLocation; // has no destination.
                    pck.PacketType = PacketType.ResponseSinkPosition;
                    pck.PID = PublicParamerters.NumberofGeneratedPackets;
                    pck.SinksAgentsList = CopyFewSinks(hightierNode.GetSinksAgentsFromHighTierNode, SinkIDs); // needs recovery
                    double dis = Operations.DistanceBetweenTwoPoints(hightierNode.CenterLocation, pck.Destination);
                    pck.TimeToLive = 5 + Convert.ToInt16(dis / (PublicParamerters.CommunicationRangeRadius / 2));
                    counter.IncreasePacketsCounter(hightierNode, PacketType.ResponseSinkPosition);
                    return pck;
                }
            }
        }

         
        public List<SinksAgentsRow> CopyFewSinks(List<SinksAgentsRow> list, List<int> SinkIDs)
        {
            List<SinksAgentsRow> re = new List<SinksAgentsRow>(); 
            foreach( int id in SinkIDs)
            {
                foreach (SinksAgentsRow row in list)
                {
                    if (id == row.Sink.ID)
                    {
                        re.Add(new SinksAgentsRow() { AgentNode = row.AgentNode, ClosestPointOnTheDiagonal = row.ClosestPointOnTheDiagonal, Sink = row.Sink });
                    }
                }
            }
            return re;
        }

        public List<SinksAgentsRow> CopyAllSinks(List<SinksAgentsRow> list)
        {
            List<SinksAgentsRow> re = new List<SinksAgentsRow>();
            foreach (SinksAgentsRow row in list)
            {
                re.Add(new SinksAgentsRow() { AgentNode = row.AgentNode, ClosestPointOnTheDiagonal = row.ClosestPointOnTheDiagonal, Sink = row.Sink });
            }

            return re;
        }

        public ResonseSinkPositionMessage()
        {
            counter = new NetworkOverheadCounter();
        }

        public void HandelInQueuPacket(Sensor currentNode, Packet InQuepacket)
        {
            SendPacket(currentNode, InQuepacket);
        }

        public void SendPacket(Sensor sender, Packet pck)
        {
            if (pck.PacketType == PacketType.ResponseSinkPosition)
            {
                sender.SwichToActive();
                // neext hope:
                Sensor Reciver = SelectNextHop(sender, pck);
                if (Reciver != null)
                {
                    // overhead:
                    counter.ComputeOverhead(pck, EnergyConsumption.Transmit, sender, Reciver);
                    counter.Animate(sender, Reciver, pck);
                    //:
                    RecivePacket(Reciver, pck);
                }
                else
                {
                    counter.SaveToQueue(sender, pck);
                }
            }
        }

        /// <summary>
        /// if source is an agent, we just send the packet directly to the sink and no need for clustring.
        /// </summary>
        /// <param name="source"></param>
        /// <param name="packt"></param>
        private void PreparDataTransmission(Sensor source, Packet packt)
        {
            if (packt.isRecovery)
            {
                // durig recovery, the agent should not be a source.
                // source should add a record.
                List<Sensor> NewAents = new List<Sensor>(); // new agent for the recovery.
                foreach (SinksAgentsRow row in packt.SinksAgentsList)
                {
                    if (row.AgentNode.ID != source.ID)
                    {
                        bool isFound = Operations.FindInAlistbool(row.AgentNode, NewAents);
                        if (!isFound)
                        {
                            // agent row.AgentNode is for the sink row.sink.
                            NewAents.Add(row.AgentNode);
                        }
                    }
                }

                if (source.RecoveryRow == null)
                {
                    Console.WriteLine("ResonseSinkPositionMessage. New Recovery Record is created at prevAgent ID" + source.ID + " Packet PID " + packt.PID + " Path: " + packt.Path);
                    // keep recored for the recovery of upcoming packets. no need for request and respanse a gain. 
                    source.RecoveryRow = new RecoveryRow()
                    {
                        ObtiantedTime = PublicParamerters.SimulationTime,
                        PrevAgent = source,
                        RecoveryAgentList = packt.SinksAgentsList,
                        ObtainPacketsRetry = 0
                    };
                }
                else
                {
                    Console.WriteLine("ResonseSinkPositionMessage: PrevAgent ID " + source.ID + " has recovery record Packet PID " + packt.PID + " Path:" + packt.Path);
                    source.RecoveryRow.ObtiantedTime = PublicParamerters.SimulationTime;
                    source.RecoveryRow.PrevAgent = source;
                    source.RecoveryRow.RecoveryAgentList = packt.SinksAgentsList;
                    source.RecoveryRow.ObtainPacketsRetry += 1;
                }



                if (NewAents.Count > 0)
                {
                    new DataPacketMessages(source, packt);
                }
                else
                {
                    if (packt.SinkIDsNeedsRecovery != null)
                    {
                        new RecoveryMessage(source, packt); // obtain
                    }
                    else
                    {
                        Console.WriteLine("ResonseSinkPositionMessage: ******************************* PrevAgent ID " + source.ID + " has recovery record Packet PID " + packt.PID + " Path:" + packt.Path);
                        MessageBox.Show("xxx&&&&&&&&&&&ResonseSinkPositionMessage");
                    }
                }
            }
            else
            {
                // normal delveiry
                new DataPacketMessages(source, packt);
            }


        }

        private void RecivePacket(Sensor Reciver, Packet packt)
        {
            
            packt.Path += ">" + Reciver.ID;
            if (LoopMechan.isLoop(packt))
            {
                // drop the packet:
                counter.DropPacket(packt, Reciver, PacketDropedReasons.Loop);
            }
            else
            {
                packt.ReTransmissionTry = 0;

                if (Reciver.CenterLocation == packt.Destination) // packet is recived.
                {

                    if (packt.SinksAgentsList.Count == 0)
                    {
                        Console.WriteLine("SinksAgentsList.Count == 0 " + "Source " + packt.Source.ID);
                    }

                    counter.SuccessedDeliverdPacket(packt);
                    counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                    counter.DisplayRefreshAtReceivingPacket(Reciver);
                    Reciver.Ellipse_nodeTypeIndicator.Fill = Brushes.Transparent;
                    PreparDataTransmission(Reciver, packt); // the merge path


                }
                else
                {
                    // compute the overhead:
                    counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                    if (packt.Hops <= packt.TimeToLive)
                    {
                        SendPacket(Reciver, packt);
                    }
                    else
                    {
                        counter.DropPacket(packt, Reciver, PacketDropedReasons.TimeToLive);
                    }
                }
            }
        }


        public Sensor SelectNextHop(Sensor ni, Packet packet)
        {
            return new GreedyRoutingMechansims().LBDDGreedy(ni, packet);
        }



    }
}
