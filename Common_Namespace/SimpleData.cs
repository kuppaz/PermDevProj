using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace Common_Namespace
{
    public class SimpleData
    {
        public static string PathInputString = Regex.Replace(Application.StartupPath.ToString(), "(\\\\bin|\\\\Debug|\\\\Motion processing)", String.Empty) + "\\_Data In",
                             PathInputConfigurations = Regex.Replace(Application.StartupPath.ToString(), "(\\\\bin|\\\\Debug|\\\\Motion processing)", String.Empty) + "\\_Configurations",
                             PathOutputString = Regex.Replace(Application.StartupPath.ToString(), "(\\\\bin|\\\\Debug|\\\\Motion processing)", String.Empty) + "\\_Output"
                             ;

        public static string ConfigurationFileIn = "";

        public static int iMx;
        public static int iMxSmthd;
        public static int iMz;
        public static int iMq;

        public static int iMx_Vertical, iMq_Vertical;

        public static int iMz_Align = 7, iMx_Align = 9, iMq_Align = 3;

        public static double A = 6378245.0; // a - большая полуось
        public static double Ex_Squared = 0.00669342749;
        public static double U = 0.000072921151467;

        public static double ToRadian = Math.PI / 180.0;
        public static double ToDegree = 180.0 / Math.PI;
        public static double ToRadian_min = ToRadian / 60.0;
        public static double ToRadian_sec = ToRadian_min / 60.0;
        public static double ToDegree_min = ToDegree * 60.0;
        public static double ToDegree_sec = ToDegree_min * 60.0;

        public static double Gravity_Normal = 9.78049;

        public static Matrix Identity = new Matrix(3, 3);

        public static int StartPosNum;

        //  Parameters of the Earth
        public static double A_84 = 6378137.0;          /* WGS84 Earth ellipsoide big semiaxe in meter */
        public static double Alpha_84 = (1.0 / 298.257223563);
        public static double E2_84 = (2.0 * Alpha_84 - Alpha_84 * Alpha_84);

        public static double A_90 = 6378136.0;
        public static double Alpha_90 = (1.0 / 298.25784);
        public static double E2_90 = (2.0 * Alpha_90 - Alpha_90 * Alpha_90);

        public static double A_42 = 6378245.0;
        public static double Alpha_42 = (1.0 / 298.3);
        public static double E2_42 = (2.0 * Alpha_42 - Alpha_42 * Alpha_42);
    }
}
