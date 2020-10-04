using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using LBDD.Properties;
using System;
using System.Collections.Generic;

namespace LBDD.MergedPath.Routing
{
    public class RecoveryRow
    {
        /// <summary>
        /// old agent
        /// </summary>
        public Sensor PrevAgent { get; set; }

        /// <summary>
        ///  the sinks that needs recovery.
        /// </summary>
        public List<SinksAgentsRow> RecoveryAgentList = null;
        /// <summary>
        /// when this record is obtian. it has lemited time.
        /// </summary>
        public double ObtiantedTime { get; set; } // the time that the recovery row is obtianted. 

        /// <summary>
        /// recovery record. old gent send oabtian packet. this record should be
        /// </summary>
        public bool IsExpired
        {
            get
            {
                if (PublicParamerters.SimulationTime - ObtiantedTime >= 10)
                    return true;
                else return false;
            }
        }

        /// <summary>
        /// in case the old gent send obtian and get no response from the high teir node. 
        /// we give two chance for the old agent.
        /// </summary>
        public int ObtainPacketsRetry { get; set; }
    }

    /// <summary>
    /// recovery algorithm:
    /// the previose agent sends obtian new sink position and wait for 10 sinks. 
    /// if the new position is arrived to old gent then, the old agent will send the packet to new agent.
    /// if 10s are past, the packet should be droped.
    /// </summary>
    class RecoveryMessage
    {
        /// <summary>
        /// preAgent send an obtian packet to ask for the new agent of the sink.
        /// </summary>
        /// <param name="preAgent"></param>
        /// <param name="SinksIDsRequiredRecovery"></param>
        public RecoveryMessage(Sensor preAgent, Packet packet)
        {
            // preAgent: if it has no record, then we obtian the sink position first.
            // if the preAgent did not sent an obtian message to the hightier node
            if (preAgent.RecoveryRow == null)
            {
                // obtian the recovery. obtian the sink location.
                Console.WriteLine("RecoveryMessage. >>>>Trying to recover the packet. Source ID=" + packet.Source.ID + " PID=" + packet.PID + " Path " + packet.Path);
                new ObtainSinkFreshPositionMessage(preAgent, packet.SinkIDsNeedsRecovery);
            }
            else
            {
                // it has record. no need to resend an obtian: but you should wait.
                if (!preAgent.RecoveryRow.IsExpired)
                {
                    // not expired
                    List<Sensor> NewAents = new List<Sensor>(); // new agent for the recovery.
                    foreach (SinksAgentsRow newAgentRow in preAgent.RecoveryRow.RecoveryAgentList)
                    {
                        if (newAgentRow.AgentNode.ID != preAgent.ID)
                        {
                            bool isFound = Operations.FindInAlistbool(newAgentRow.AgentNode, NewAents);
                            if (!isFound)
                            {
                                packet.Destination = newAgentRow.AgentNode.CenterLocation;
                                NewAents.Add(newAgentRow.AgentNode);
                            }
                        }
                    }

                    // if we found the new agent.
                    if (NewAents.Count > 0)
                    {
                        Console.WriteLine("RecoveryMessage. Source ID=" + packet.Source.ID + " PID=" + packet.PID + " Path " + packet.Path);
                        packet.SinksAgentsList = preAgent.RecoveryRow.RecoveryAgentList;
                        new DataPacketMessages(preAgent, packet);

                    }
                    else
                    {
                        if (preAgent.RecoveryRow.ObtainPacketsRetry >= 3)
                        {
                            // in case no new agent is found.
                            Console.WriteLine("RecoveryMessage. No agent is found during the recovery.Source ID = " + packet.Source.ID + " PID = " + packet.PID + " Path " + packet.Path);
                            NetworkOverheadCounter counter = new NetworkOverheadCounter();
                            counter.DropPacket(packet, preAgent, PacketDropedReasons.RecoveryNoNewAgentFound);
                        }
                        else
                        {
                            Console.WriteLine("RecoveryMessage. Recovery period is expired. Old Agent is sending new obtain packet!" + " Path " + packet.Path);
                            new ObtainSinkFreshPositionMessage(preAgent, packet.SinkIDsNeedsRecovery); // obtain
                        }
                    }
                }
                else
                {
                    // resent the obtian packet. 2 times.
                    if (preAgent.RecoveryRow.ObtainPacketsRetry <= 3)
                    {
                        Console.WriteLine("RecoveryMessage. Recovery period is expired. Old Agent is sending new obtain packet!" + " Path " + packet.Path);
                        new ObtainSinkFreshPositionMessage(preAgent, packet.SinkIDsNeedsRecovery); // obtain
                    }
                    else
                    {
                        Console.WriteLine("RecoveryMessage. Recovery period is expired. we tryied to re-sent the obtian packet for three times and faild. The packet will be droped." + " Path " + packet.Path);
                        // drop the packet:
                        NetworkOverheadCounter counter = new NetworkOverheadCounter();
                        counter.DropPacket(packet, preAgent, PacketDropedReasons.RecoveryPeriodExpired);
                        preAgent.RecoveryRow = null;
                    }

                }
            }
        }
    }
}
