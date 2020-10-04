using LBDD.Comuting.Routing;
using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Dataplane.PacketRouter;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using LBDD.Properties;
using System;
using System.Collections.Generic;

namespace LBDD.MergedPath.Routing
{

    //HandOffToTheSinkOrRecovry
   // number of data packet is not right... check it.
   // the problem is when delivering to the sink.
    class DataPacketMessages
    {
        private LoopMechanizimAvoidance LoopMechan = new LoopMechanizimAvoidance();
        private NetworkOverheadCounter counter;
        /// <summary>
        /// currentBifSensor: current node that has the packet.
        /// Branches: the branches 
        /// isSourceAnAgent: the source is an agent for a sink. That is to say no need for clustering the source itslef this time.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="Branches"></param>
        /// <param name="packet"></param>
        /// <param name="isSourceAnAgent"></param>
        public DataPacketMessages(Sensor sender, Packet packet)
        {
            counter = new NetworkOverheadCounter();

            // the source node creates new
            if (packet.PacketType == PacketType.ResponseSinkPosition) // i d
            {
                // create new: packet for each sink.
                foreach (SinksAgentsRow sinksAgents in packet.SinksAgentsList)
                {
                    if (sender.ID == sinksAgents.AgentNode.ID)
                    {
                        //skip the test here and send to the known sink by urself
                        //Hand of to the sink by urself this is me,
                        Packet pkt = GeneragtePacket(sender, sinksAgents); //                                                         
                        pkt.SinksAgentsList = packet.SinksAgentsList;
                        HandOffToTheSinkOrRecovry(sender, pkt);

                    }
                    else
                    {
                        Packet pck = GeneragtePacket(sender, sinksAgents); // duplicate.                                                            
                        pck.SinksAgentsList = packet.SinksAgentsList;
                        SendPacket(sender, pck);
                    }
                }
            }
            else if (packet.PacketType == PacketType.Data) // duplicate:
            {
                // recovery packets:
                Packet dupPck = Duplicate(packet, sender, false);
                SendPacket(sender, dupPck);
            }
        }

        /// <summary>
        /// we call this for HandelInQueuPacket
        /// </summary>
        public DataPacketMessages()
        {
            counter = new NetworkOverheadCounter();
        }

     
        /// <summary>
        ///  handel the packets in the queue of the node. 
        /// </summary>
        /// <param name="currentNode"></param>
        /// <param name="InQuepacket"></param>
        public void HandelInQueuPacket(Sensor currentNode, Packet InQuepacket)
        {
            SendPacket(currentNode, InQuepacket);
        }



        /// <summary>
        /// increase the number of packets by one.
        /// </summary>
        /// <param name="packet"></param>
        /// <param name="currentBifSensor"></param>
        /// <returns></returns>
        private Packet Duplicate(Packet packet, Sensor currentBifSensor, bool IncreasePid)
        {
            Packet pck = packet.Clone() as Packet;
            if (IncreasePid)
            {
                PublicParamerters.NumberofGeneratedPackets += 1;
                pck.PID = PublicParamerters.NumberofGeneratedPackets;
                counter.IncreasePacketsCounter(currentBifSensor, PacketType.Data); // 
            }
            return pck;
        }

        

        /// <summary>
        /// due to duplication, we dont count the generated packets here. we count it when it recived.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="branch"></param>
        /// <returns></returns>
        private Packet GeneragtePacket(Sensor sender, SinksAgentsRow row)
        {

            PublicParamerters.NumberofGeneratedPackets += 1;
            Packet pck = new Packet();
            pck.Source = sender;
            pck.Path = "" + sender.ID;
            pck.Destination = row.AgentNode.CenterLocation;
            pck.PacketType = PacketType.Data;
            pck.PID = PublicParamerters.NumberofGeneratedPackets;
            pck.TimeToLive = Convert.ToInt16((Operations.DistanceBetweenTwoPoints(sender.CenterLocation, pck.Destination) / (PublicParamerters.CommunicationRangeRadius / 3)));
            counter.IncreasePacketsCounter(sender, PacketType.Data); // 
            return pck;
        }

        public void SendPacket(Sensor sender, Packet pck)
        {
            if (pck.PacketType == PacketType.Data)
            {
                sender.SwichToActive();
                // neext hope:
                Sensor Reciver = SelectNextHop(sender, pck);
                if (Reciver != null)
                {
                    // overhead:
                    counter.ComputeOverhead(pck, EnergyConsumption.Transmit, sender, Reciver);
                    counter.Animate(sender, Reciver, pck);
                    RecivePacket(Reciver, pck);
                }
                else
                {
                    counter.SaveToQueue(sender, pck); // save in the queue.
                }
            }
        }

        private void RecivePacket(Sensor Reciver, Packet packt)
        {
            packt.Path += ">" + Reciver.ID;
            if (LoopMechan.isLoop(packt))
            {
                counter.DropPacket(packt, Reciver, PacketDropedReasons.Loop);
            }
            else
            {
               
                packt.ReTransmissionTry = 0;
                if (packt.Destination == Reciver.CenterLocation)
                {
                    
                    counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                    counter.DisplayRefreshAtReceivingPacket(Reciver);
                    HandOffToTheSinkOrRecovry(Reciver, packt);
                }
                else
                {
                    counter.ComputeOverhead(packt, EnergyConsumption.Recive, null, Reciver);
                    counter.DisplayRefreshAtReceivingPacket(Reciver);
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


        /// <summary>
        /// find x in inlist
        /// 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="inlist"></param>
        /// <returns></returns>
        private bool StillWithinMyRange(SinksAgentsRow x, List<SinksAgentsRow> inlist)
        {
            foreach (SinksAgentsRow rec in inlist)
            {
                if (rec.Sink.ID == x.Sink.ID)
                {
                    return true;
                }
            }

            return false;
        }

        private List<SinksAgentsRow> GetMySinksFromPacket(int AgentID, List<SinksAgentsRow> inpacketSinks)
        {
            List<SinksAgentsRow> re = new List<SinksAgentsRow>();
            foreach(SinksAgentsRow x in inpacketSinks)
            {
                if(x.AgentNode.ID==AgentID)
                {
                    re.Add(x);
                }
            }

            return re;
        }


        /// <summary>
        /// hand the packet to my sink.
        /// </summary>
        /// <param name="agent"></param>
        /// <param name="packt"></param>
        public void HandOffToTheSinkOrRecovry(Sensor agent, Packet packt)
        {
            // check how many sinks are there in my record
            if (agent != null)
            {
                if (packt.SinksAgentsList != null)
                {
                    // my sinks recored in the packet:
                    List<SinksAgentsRow> MysinksInPpaket = GetMySinksFromPacket(agent.ID, packt.SinksAgentsList); // my sinks in the packet.
                    List<SinksAgentsRow> MyCurrentSinks = agent.GetSinksAgentsList; //my sinks that currently within my range.
                    List<int> SinksIDsRequiredRecovery = new List<int>(); //  sinks that required recovery. those sinks which are in the packet but not within my range anymore.

                    for (int i = 0; i < MysinksInPpaket.Count; i++)
                    {
                        SinksAgentsRow sinkInPacket = MysinksInPpaket[i];
                        // check if sink still within the range of the agent
                        bool stillWithinMyRange = StillWithinMyRange(sinkInPacket, MyCurrentSinks); // check if sink x  still within my range
                        if (stillWithinMyRange)
                        {

                            // I am an agent for more than one sink
                            // here we should increase the PID, otherwise the number of delivered packets will be more than the generated packets.
                            Packet pck = Duplicate(packt, agent, false); // duplicate and increase the PID
                            Sink sink = sinkInPacket.Sink;
                            pck.Path += "> Sink: " + sink.ID;
                            counter.SuccessedDeliverdPacket(pck);
                            counter.DisplayRefreshAtReceivingPacket(agent);
                        }
                        else
                        {
                            // sinkInPacket.Sink is out of agent range.
                            SinksIDsRequiredRecovery.Add(sinkInPacket.Sink.ID);
                        }
                    }

                    // recovery: SinksIDsRequiredRecovery
                    if (SinksIDsRequiredRecovery.Count > 0)
                    {
                        packt.SinkIDsNeedsRecovery = SinksIDsRequiredRecovery;
                        new RecoveryMessage(agent, packt);
                    }
                }
                else
                {
                    // drop the packet.
                    // i dont know when it should be null.
                    Console.Write(">>>>No agents. MergedPathsMessages->HandOffToTheSinkOrRecovry->packt.SinksAgentsList==null");
                    counter.DropPacket(packt, agent, PacketDropedReasons.Unknow);
                }
            }
            else
            {
                // drop the packet
                Console.Write(">>>>HandOffToTheSinkOrRecovry->agent = null");
                counter.DropPacket(packt, agent, PacketDropedReasons.Unknow);
            }
        }

        /// <summary>
        /// get the max value
        /// </summary>
        /// <param name="ni"></param>
        /// <param name="packet"></param>
        /// <returns></returns>
        public Sensor SelectNextHop(Sensor ni, Packet packet)
        {
            return new GreedyRoutingMechansims().LBDDGreedy(ni, packet);

        }


    }
}
