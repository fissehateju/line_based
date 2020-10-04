using LBDD.Dataplane.PacketRouter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LBDD.MergedPath.computing
{
    class Sorters
    {
        public class CoordinationEntrySorter : IComparer<CoordinationEntry>
        {
            public int Compare(CoordinationEntry y, CoordinationEntry x)
            {
                return x.Priority.CompareTo(y.Priority);
            }
        }
    }
}
