using LBDD.MergedPath.Routing;
using System;
using System.Collections.Generic;
using System.Windows;

namespace LBDD.Dataplane.NOS
{
    public enum PacketType
    {
        Data,
        VirtualLineConstruction,
        ShareSinkPosition,
        ReportSinkPosition,
        ObtainSinkPosition,
        ResponseSinkPosition
    }

    public enum PacketDropedReasons
    {
        NULL,
        TimeToLive,
        WaitingTime,
        Loop,
        RecoveryNoNewAgentFound,
        RecoveryPeriodExpired,
        Unknow
    } 

    public class Packet: ICloneable, IDisposable
    {
        //: Packet section:
        
        public long PID { get; set; } // SEQ ID OF PACKET.
       // public long MID { get; set; } // merged path ID
        public PacketType PacketType { get; set; }
        public bool isDelivered { get; set; }
        public bool IsLooped { get; set; }
        public bool isRecovery
        {
            get
            {
                if(SinkIDsNeedsRecovery==null)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }
        public double PacketLength
        {
            get
            {
                switch (PacketType)
                {
                    case PacketType.Data: return 1024;
                    case PacketType.VirtualLineConstruction: return 256;
                    case PacketType.ShareSinkPosition: return 128;
                    case PacketType.ObtainSinkPosition: return 128;
                    case PacketType.ReportSinkPosition: return 128;
                    case PacketType.ResponseSinkPosition: return 128;
                    default: return 512;
                }
            }
        }

       
        //public int H2S { get { if (PacketType == PacketType.Data) return Source.HopsToSink; else return Destination.HopsToSink; } }
        public int TimeToLive { get; set; }
        public int Hops { get; set; }
        public string Path { get; set; }
        //public double RoutingDistance { get; set; }
        public double Delay { get; set; }
        public double UsedEnergy_Joule { get; set; }
        public int WaitingTimes { get; set; }

        public List<int> SinkIDsNeedsRecovery = null;// if the agent node did not find its sink. then recovery is rquired for these sinks. this should be set during obtiansinkposition.
        public Sensor Source { get; set; }
        public Point Destination { get; set; }
        public SinksAgentsRow ReportSinkPosition { get; set; } // ReportSinkPosition
        public PacketDropedReasons PacketDropedReasons { get; set; } 
        public List<SinksAgentsRow> SinksAgentsList { get; set; } // the sinks that should be resonsed to the source that requested
        public PacketDirection PacketDirection { get; set; }
       

        /// <summary>
        /// copy the object
        /// </summary>
        /// <returns></returns>
        public object Clone()
        {
            return MemberwiseClone();
        }

        // remove the object
        public void Dispose()
        {
            GC.SuppressFinalize(this);
        }

        public int ReTransmissionTry { get; set; } // in one hope. this should be =0 when the packet is recived.
    }
}
