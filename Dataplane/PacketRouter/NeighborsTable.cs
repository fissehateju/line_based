using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LBDD.Dataplane.PacketRouter
{
    

    public class CoordinationEntry
    {
        public double Priority { get; set; }
        public int SensorID { get { return Sensor.ID; } }
        public Sensor Sensor { get; set; }

        public double MinRange { get; set; }
        public double MaxRange { get; set; } 

    }


}
