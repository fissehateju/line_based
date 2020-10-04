using LBDD.Dataplane;
using LBDD.Dataplane.NOS;
using LBDD.Dataplane.PacketRouter;
using LBDD.Intilization;
using LBDD.MergedPath.computing;
using System;
using System.Collections.Generic;
using System.Windows;

namespace LBDD.Comuting.Routing 
{
    class GreedyRoutingMechansims
    {
        private NetworkOverheadCounter counter;

        public GreedyRoutingMechansims()
        {
            counter = new NetworkOverheadCounter();
        }

        /// <summary>
        /// good perforamce
        /// </summary>
        /// <param name="ni"></param>
        /// <param name="packet"></param>
        /// <returns></returns>
        public Sensor Greedy1(Sensor ni, Packet packet)
        {
            List<CoordinationEntry> coordinationEntries = new List<CoordinationEntry>();
            Point endPoint = packet.Destination;

            Sensor sj = null;
            double sum = 0;
            foreach (Sensor nj in ni.NeighborsTable)
            {
                if (nj.ResidualEnergyPercentage > 0)
                {
                    double Norangle = Operations.AngleDotProdection(ni.CenterLocation, nj.CenterLocation, endPoint);
                    double dj = Operations.DistanceBetweenTwoPoints(nj.CenterLocation, endPoint);
                    if (Norangle < 0.5)
                    {
                        double aggregatedValue = dj * Norangle;
                        sum += aggregatedValue;
                        coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                    }
                }
            }
            // coordination"..... here
            sj = counter.CoordinateGetMin(coordinationEntries, packet, sum);
            return sj;
        }


        /// <summary>
        /// worsre performance
        /// </summary>
        /// <param name="ni"></param>
        /// <param name="packet"></param>
        /// <returns></returns>
        public Sensor Greedy2(Sensor ni, Packet packet)
        {
            List<CoordinationEntry> coordinationEntries = new List<CoordinationEntry>();
            Point endPoint = packet.Destination;
            Sensor sj = null;
            double sum = 0;
            foreach (Sensor nj in ni.NeighborsTable)
            {
                if (nj.ResidualEnergyPercentage > 0)
                {
                    double dj = Operations.DistanceBetweenTwoPoints(nj.CenterLocation, endPoint);
                    double aggregatedValue = dj;
                    sum += aggregatedValue;
                    coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                }
            }
            // coordination"..... here
            sj = counter.CoordinateGetMin(coordinationEntries, packet, sum);
            return sj;
        }

        /// <summary>
        /// showed the best performance.
        /// </summary>
        /// <param name="ni"></param>
        /// <param name="packet"></param>
        /// <returns></returns>
        public Sensor Greedy3(Sensor ni, Packet packet)
        {
            List<CoordinationEntry> coordinationEntries = new List<CoordinationEntry>();
            Point endPoint = packet.Destination;

            Sensor sj = null;
            double sum = 0;

            switch( packet.PacketType)
            {
                case PacketType.ObtainSinkPosition:
                    {
                        foreach (Sensor nj in ni.NeighborsTable)
                        {
                            if (nj.ResidualEnergyPercentage > 0)
                            {
                                if (nj.IsHightierNode)
                                {

                                    double aggregatedValue = 1;
                                    sum += aggregatedValue;
                                    coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                    //return nj;
                                }
                                else
                                {
                                    double Norangle = Operations.AngleDotProdection(ni.CenterLocation, nj.CenterLocation, endPoint);
                                    double dj = Operations.DistanceBetweenTwoPoints(nj.CenterLocation, endPoint);
                                    if (Norangle < 0.5)
                                    {
                                        double aggregatedValue = dj;
                                        sum += aggregatedValue;
                                        coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                    }
                                    else if(Norangle==0 || Double.IsNaN(Norangle))
                                    {
                                        double aggregatedValue = 1;
                                        sum += aggregatedValue;
                                        coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                    }
                                }
                            }
                        }
                    }
                    break;
                case PacketType.ReportSinkPosition:
                    {
                        foreach (Sensor nj in ni.NeighborsTable)
                        {
                            if (nj.ResidualEnergyPercentage > 0)
                            {
                                if (nj.IsHightierNode) return nj;
                                else
                                {
                                    double Norangle = Operations.AngleDotProdection(ni.CenterLocation, nj.CenterLocation, endPoint);
                                    double dj = Operations.DistanceBetweenTwoPoints(nj.CenterLocation, endPoint);
                                    if (Norangle == 0 || Double.IsNaN(Norangle))
                                    {
                                        double aggregatedValue = 1;
                                        sum += aggregatedValue;
                                        coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                    }
                                    else if (Norangle < 0.5)
                                    {

                                        double aggregatedValue = dj;
                                        sum += aggregatedValue;
                                        coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                    }
                                }
                            }
                        }
                    }
                    break;
                default:
                    {
                        // defaul greedy:
                        foreach (Sensor nj in ni.NeighborsTable)
                        {
                            if (nj.ResidualEnergyPercentage > 0)
                            {
                                double Norangle = Operations.AngleDotProdection(ni.CenterLocation, nj.CenterLocation, endPoint);
                                double dj = Operations.DistanceBetweenTwoPoints(nj.CenterLocation, endPoint);

                                if (Norangle == 0 || Double.IsNaN(Norangle))
                                {
                                    double aggregatedValue = 1;
                                    sum += aggregatedValue;
                                    coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                }
                                else if (Norangle < 0.5 )
                                {
                                    double aggregatedValue = dj;
                                    sum += aggregatedValue;
                                    coordinationEntries.Add(new CoordinationEntry() { Priority = aggregatedValue, Sensor = nj }); // candidaite
                                }
                            }
                        }
                    }
                    break;
            }
            
            // coordination"..... here
            sj = counter.CoordinateGetMin(coordinationEntries, packet, sum);
            return sj;
        }

        public Sensor LBDDGreedy(Sensor ni, Packet packet)
        {
            return Greedy3(ni, packet);
        }

      

    }
}
