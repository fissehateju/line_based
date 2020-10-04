using LBDD.Dataplane;
using System;
using System.Collections.Generic;
using LBDD.Energy;
using LBDD.ExpermentsResults.Lifetime;
using LBDD.ui;
using System.Windows.Media;
using LBDD.ControlPlane.NOS;
using LBDD.Dataplane.NOS;
using LBDD.Properties;
using System.Windows.Controls.DataVisualization;

namespace LBDD.Dataplane
{
    /// <summary>
    /// 
    /// </summary>
    public class PublicParamerters
    {



        #region PacketsCounter
        public static long NumberOfDataPacket { get; set; } //Generated
        public static long NumberOfObtainSinkPositionPacket { get; set; } //Generated
        public static long NumberOfReportSinkPositionPacket { get; set; } //Generated
        public static long NumberOfResponseSinkPositionPacket { get; set; } //Generated
        public static long NumberOfShareSinkPositionPacket { get; set; } //Generated

        public static long NumberOfDiagonalVirtualLineConstructionPacket { get; set; } //Generated

        public static long NumberofGeneratedPackets { get; set; }
        public static long NumberofControlPackets { get; set; }
        public static long NumberofDropedPacket { get; set; } // no matter what is the type of the packet.
        public static long NumberofDeliveredPacket { get; set; } // the number of the pakctes recived in the sink node.


        public static long InQueuePackets => NumberofGeneratedPackets - NumberofDeliveredPacket - NumberofDropedPacket;
        public static double DeliveredRatio => 100 - DropedRatio;
        public static double DropedRatio => 100 * (Convert.ToDouble(NumberofDropedPacket) / Convert.ToDouble(NumberofGeneratedPackets));
        public static List<Packet> FinishedRoutedPackets = new List<Packet>(); // all the packets whatever dliverd or not.


        public static List<Packet> DropedPacketsList = new List<Packet>();
        #endregion



        #region Energy

        public static double EnergyComsumedForControlPackets { get; set; }
        public static double EnergyConsumedForDataPackets { get; set; }
        public static double BatteryIntialEnergy { get { return Settings.Default.BatteryIntialEnergy; } } //J 0.5 /////////////*******////////////////////////////////////
        public static double BatteryIntialEnergyForSink = 500; //500J.
        public static double E_elec = 50; // unit: (nJ/bit) //Energy dissipation to run the radio
        public static double Efs = 0.01;// unit( nJ/bit/m^2 ) //Free space model of transmitter amplifier
        public static double Emp = 0.0000013; // unit( nJ/bit/m^4) //Multi-path model of transmitter amplifier
        public static double TotalEnergyConsumptionJoule { get; set; } // keep all energy consumption. 
        public static double TotalWastedEnergyJoule { get; set; } // idel listening energy
        public static double WastedEnergyPercentage { get { return 100 * (TotalWastedEnergyJoule / TotalEnergyConsumptionJoule); } } // idel listening energy percentage  

        #endregion


        #region Time
        public static long TotalWaitingTime { get; set; } // how many times the node waitted for its coordinate to wake up.
        public static double TotalDelayMs { get; set; } // in ms 

        public static TimeSpan QueueTime => TimeSpan.FromSeconds(Settings.Default.QueueTime);

        /// <summary>
        /// including QueueDelay + transmission delay 
        /// </summary>
        public static double AverageDelay
        {
            get
            {

                double QueueDelay = (TotalWaitingTime * Settings.Default.QueueTime);
                double delay = TotalDelayMs;

                double sumDelay = QueueDelay + delay;

                double average = sumDelay / Convert.ToDouble(NumberofDeliveredPacket);
                return average;
            }
        }

        public static double AverageQueueDelay
        {
            get
            {
                double QueueDelay = (TotalWaitingTime * Settings.Default.QueueTime);
                double average = QueueDelay / Convert.ToDouble(NumberofDeliveredPacket);
                return average;
            }
        }

        public static double AverageTransmissionDelay
        {
            get
            {
                double delay = TotalDelayMs;
                double average = delay / Convert.ToDouble(NumberofDeliveredPacket);
                return average;
            }
        }


        public static double AverageWaitingTimes
        {
            get
            {
                double average = TotalWaitingTime / Convert.ToDouble(NumberofDeliveredPacket);
                return average;
            }
        }
        public static class Periods
        {
            public static double ActivePeriod { get { return Settings.Default.ActivePeriod; } } //  the node trun on and check for CheckPeriod seconds.// +1
            public static double SleepPeriod { get { return Settings.Default.SleepPeriod; } }  // the node trun off and sleep for SleepPeriod seconds.
        }

        public static int MacStartUp => Settings.Default.MacStartUp;

        /// <summary>
        /// the runnunin time of simulator. in SEC
        /// </summary>
        public static UInt32 SimulationTime
        {
            get; set;
        }



        #endregion



        #region One_Hope

        public static double TotalRoutingDistance { get; set; }
        /// <summary>
        ///  average routing distance foreach path.
        /// </summary>
        public static double AverageRoutingDistance
        {
            get
            {
                return TotalRoutingDistance / NumberofDeliveredPacket;
            }
        }
        /// <summary>
        /// average trnsmission distance.
        /// </summary>
        public static double AverageTransmissionDistance
        {
            get
            {
                return TotalRoutingDistance / TotalNumberOfHops;
            }
        }

        public static double TotalNumberOfHops { get; set; }
        public static double AverageHops
        {
            get
            {
                return TotalNumberOfHops / NumberofDeliveredPacket;
            }
        }
        public static long TotalReduntantTransmission { get; set; } // how many transmission are redundant, that is to say, recived and canceled.
        public static double AverageRedundantTransmissions
        {
            get
            {
                return Convert.ToDouble(TotalReduntantTransmission) / NumberofDeliveredPacket;
            }
        }

        #endregion



        #region General 
        public static long Rounds { get; set; } // how many rounds.
        public static List<DeadNodesRecord> DeadNodeList = new List<DeadNodesRecord>();
        public static bool IsNetworkDied { get; set; } // yes if the first node deide.
        public static double SensingRangeRadius { get; set; }
        



        public static double NetworkSquareSideLength { get; set; } // the size here is width x length// we assume the network is square.
        public static double CommunicationRangeRadius { get { return SensingRangeRadius * 2; } } // sensing range is R in the DB.
        public static double TransmissionRate = 2 * 1000000;////2Mbps 100 × 10^6 bit/s , //https://en.wikipedia.org/wiki/Transmission_time
        public static double SpeedOfLight = 299792458;//https://en.wikipedia.org/wiki/Speed_of_light // s
        public static string PowersString { get; set; }
        public static double ThresholdDistance  //Distance threshold ( unit m) 
        {
            get { return Math.Sqrt(Efs / Emp); }
        }


        
        #endregion



        public static List<Color> RandomColors { get; set; }

        public static double SensingFeildArea
        {
            get; set;
        }

        public static int NumberofNodes
        {
            get; set;
        }

        
        public static int SinkCount
        {
            get
            {
                return MainWindow.mySinks.Count;
            }
        }

        public static MainWindow MainWindow { get; set; } 

        /// <summary>
        /// Each time when the node loses 5% of its energy, it shares new energy percentage with its neighbors. The neighbor nodes update their energy distributions according to the new percentage immediately as explained by Algorithm 2. 
        /// </summary>
        

        // lifetime paramerts:
        public static int NOS { get; set; } // NUMBER OF RANDOM SELECTED SOURCES
        public static int NOP { get; set; } // NUMBER OF PACKETS TO BE SEND.



        /// <summary>
        /// reset all counters.
        /// </summary>
        public static void ResetAndStopSimulation()
        {
            NumberOfDataPacket = 0;
            NumberOfObtainSinkPositionPacket = 0;
            NumberOfReportSinkPositionPacket = 0;
            NumberOfResponseSinkPositionPacket = 0;
            NumberOfShareSinkPositionPacket = 0;
            NumberOfDiagonalVirtualLineConstructionPacket = 0;
            NumberofGeneratedPackets = 0;
            NumberofControlPackets = 0;
            NumberofDropedPacket = 0;
            NumberofDeliveredPacket = 0;
            FinishedRoutedPackets.Clear();

            // energy
            EnergyComsumedForControlPackets = 0;
            EnergyConsumedForDataPackets = 0;
            TotalEnergyConsumptionJoule = 0;
            TotalWastedEnergyJoule = 0;

            // time:
            SimulationTime = 0;
            TotalWaitingTime = 0;
            TotalDelayMs = 0;

            // hops-distance:
            TotalRoutingDistance = 0;
            TotalNumberOfHops = 0;
            TotalReduntantTransmission = 0;

            // generral:
            IsNetworkDied = false;
            NumberofNodes = 0;


            // clear window:
            MainWindow.myNetWork.Clear();
            MainWindow.mySinks.Clear();
            MainWindow.Canvas_SensingFeild.Children.Clear();


            // stop: sumulation 
            MainWindow.StopSimulation();
        }


    }
}
