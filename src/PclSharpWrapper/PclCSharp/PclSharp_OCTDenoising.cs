using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace PclCSharp
{
    public class PclSharp_OCTDenoising
    {
        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "greedyProjection", CharSet = CharSet.Auto)]
        public static extern IntPtr greedyProjection(IntPtr in_pc);

        [DllImport("PclDll.dll", CallingConvention = CallingConvention.StdCall, EntryPoint = "saveMesh", CharSet = CharSet.Auto)]
        public static extern void saveMesh(IntPtr meshPtr, [MarshalAs(UnmanagedType.LPStr)] string file_name, [MarshalAs(UnmanagedType.LPStr)] string file_extension);
    }
}
