using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Alignment
{
    public class Alignment_Rought
    {
        public static int SINS_Alignment_Rought(Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate_OdoMod, StreamReader myFile, Kalman_Vars KalmanVars, StreamWriter GRTV_output)
        {
            int i = 0;

            //---Этап грубой выставки---
            i = RougthAlignment(ProcHelp, SINSstate, myFile, KalmanVars, SINSstate_OdoMod, GRTV_output);

            ProcHelp.initCount = false;
            return i;
        }




        public static int RougthAlignment(Proc_Help ProcHelp, SINS_State SINSstate, StreamReader myFile, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod, StreamWriter GRTV_output)
        {
            int k = 0, i = 0;
            double[] f_avg = new double[3]; double[] w_avg = new double[3]; double[] w_avg_x = new double[3]; double[] U_s = new double[3];
            Matrix A_xs = new Matrix(3, 3);

            StreamWriter Alignment_avg_rougth = new StreamWriter(SimpleData.PathOutputString + "\\Alignment\\Alignment_avg_rougth.txt");
            StreamWriter Alignment_avg_rougthMovingAVG = new StreamWriter(SimpleData.PathOutputString + "\\Alignment\\Alignment_avg_rougth_MovingAVG.txt");

            // --- вспомогательные массивы для определения сигмы шумов
            // --- array_&_i - полный массив показаний датчиков;
            // --- array_sigma_&_i - частичный массив показаний датчиков, не включающий в себя интервалы, где вместо показаний датчиков константа;
            double[] array_f_1 = new double[100000], array_sigma_f_1 = new double[100000];
            double[] array_f_2 = new double[100000], array_sigma_f_2 = new double[100000];
            double[] array_f_3 = new double[100000], array_sigma_f_3 = new double[100000];
            double[] array_w_1 = new double[100000], array_sigma_w_1 = new double[100000];
            double[] array_w_2 = new double[100000], array_sigma_w_2 = new double[100000];
            double[] array_w_3 = new double[100000], array_sigma_w_3 = new double[100000];

            // --- вектора СКО
            double[] sigma_f = new double[3];
            double[] sigma_w = new double[3];

            Alignment_avg_rougth.WriteLine("count f_1 f_2 f_3 w_1 w_2 w_3 heading roll pitch Latitude");
            Alignment_avg_rougthMovingAVG.WriteLine("count MA_f_1 MA_f_2 MA_f_3 MA_w_1 MA_w_2 MA_w_3");

            while (true)
            {
                i++;
                if (i < ProcHelp.AlignmentStartTime)
                    myFile.ReadLine();
                else
                    break;
            }
            i = 0;


            while (true)
            {
                i++;
                if (i < 1) { myFile.ReadLine(); continue; }
                if (SINSstate.FLG_Stop == 0 && false)
                {
                    // --- Чтение строки их входного файла с данными и разкладывание по структурам
                    ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);
                }
                else
                {
                    i--;
                    break;
                }
            }

            int t = i;

            double Latitude = 0.0, Pitch = 0.0, Roll = 0.0, Heading = 0.0;

            // --- длинна окна для скользящего среднего
            int MovingWindow = 500;
            // --- Массив скользящих средних для датчиков
            double[] MovingAverageAccGyro = new double[6];

            // --- k_f, k_nu - отдельные счетчики сколько обновлений соответствующих датчиков были использованы для осреднения (в некоторых заездах
            // --- почему-то ньютонометры на начальной выставке поставляют константное значение)
            int k_f = 0, k_nu = 0;

            for (i = t; ; i++)
            {
                // --- Чтение строки их входного файла с данными и разкладывание по структурам
                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);

                if ((ProcHelp.AlignmentCounts != 0 && i == ProcHelp.AlignmentCounts))
                    break;

                array_f_1[k] = SINSstate.F_z[0];
                array_f_2[k] = SINSstate.F_z[1];
                array_f_3[k] = SINSstate.F_z[2];
                array_w_1[k] = SINSstate.W_z[0];
                array_w_2[k] = SINSstate.W_z[1];
                array_w_3[k] = SINSstate.W_z[2];

                // --- Вычисляем среднее значение показаний акселерометров. Цель - детектирование константы в показаниях ньютонометров
                double array_sigma_f_1_tmp_sum = 0.0, array_sigma_w_1_tmp_sum = 0.0;
                int u = 0;
                for (u = 1; u <= Math.Min(i, 50); u++)
                    array_sigma_f_1_tmp_sum += array_f_1[i - u];
                array_sigma_f_1_tmp_sum /= (u - 1);

                // --- Если показания датчиков меняются, то заполняем соответствующие массивы
                if (Math.Abs(array_sigma_f_1_tmp_sum - array_f_1[i]) > 1E-9)
                {
                    array_sigma_f_1[k_f] = SINSstate.F_z[0];
                    array_sigma_f_2[k_f] = SINSstate.F_z[1];
                    array_sigma_f_3[k_f] = SINSstate.F_z[2];
                    f_avg[0] += SINSstate.F_z[0];
                    f_avg[1] += SINSstate.F_z[1];
                    f_avg[2] += SINSstate.F_z[2];
                    k_f++;
                }

                // --- Вычисляем среднее значение показаний ДУСов
                u = 0;
                for (u = 1; u <= Math.Min(i, 50); u++)
                    array_sigma_w_1_tmp_sum += array_w_1[i - u];
                array_sigma_w_1_tmp_sum /= (u - 1);

                // --- Если показания датчиков меняются, то заполняем соответствующие массивы
                if (Math.Abs(array_sigma_w_1_tmp_sum - array_w_1[i]) > 1E-9)
                {
                    array_sigma_w_1[k_nu] = SINSstate.W_z[0];
                    array_sigma_w_2[k_nu] = SINSstate.W_z[1];
                    array_sigma_w_3[k_nu] = SINSstate.W_z[2];
                    w_avg[0] += SINSstate.W_z[0];
                    w_avg[1] += SINSstate.W_z[1];
                    w_avg[2] += SINSstate.W_z[2];
                    k_nu++;
                }

                k++;

                // --- Вычисление скользящего среднего для его вывода в файл и только
                for (int u1 = 1; u1 <= Math.Min(k, MovingWindow); u1++)
                {
                    MovingAverageAccGyro[0] += array_f_1[k - u1];
                    MovingAverageAccGyro[1] += array_f_2[k - u1];
                    MovingAverageAccGyro[2] += array_f_3[k - u1];
                    MovingAverageAccGyro[3] += array_w_1[k - u1];
                    MovingAverageAccGyro[4] += array_w_2[k - u1];
                    MovingAverageAccGyro[5] += array_w_3[k - u1];
                }
                for (int u1 = 0; u1 < 6; u1++)
                    MovingAverageAccGyro[u1] = MovingAverageAccGyro[u1] / MovingWindow;


                // --- Вычисляем текущее значение углов
                Pitch = Math.Atan2(f_avg[1], Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
                Roll = -Math.Atan2(f_avg[0], f_avg[2]);
                A_xs = SimpleOperations.A_xs(Heading, Roll, Pitch);
                w_avg_x = Matrix.Multiply(A_xs, w_avg);

                Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);
                Latitude = Math.Atan2(w_avg_x[2], Math.Sqrt(w_avg_x[1] * w_avg_x[1] + w_avg_x[0] * w_avg_x[0]));

                SINSstate.A_sx0 = SimpleOperations.A_sx0(Heading, Roll, Pitch);
                U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);

                if (Math.Abs(w_avg[0] / k_nu - U_s[0]) < 0.000005) { }
                else
                {
                    Heading = Heading - Math.PI;
                    SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
                    U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);
                }

                // --- Вывод текущих вычисленных параметров в файлы
                if (k > MovingWindow && k % 10 == 0)
                {
                    Alignment_avg_rougth.WriteLine(SINSstate.Count.ToString()
                        + " " + (f_avg[0] / k_f).ToString() + " " + (f_avg[1] / k_f).ToString() + " " + (f_avg[2] / k_f).ToString()
                        + " " + (w_avg[0] / k_nu).ToString() + " " + (w_avg[1] / k_nu).ToString() + " " + (w_avg[2] / k_nu).ToString()
                        + " " + (Heading * SimpleData.ToDegree).ToString() + " " + (Roll * SimpleData.ToDegree).ToString()
                        + " " + (Pitch * SimpleData.ToDegree).ToString() + " " + Latitude.ToString()
                        + " " + (w_avg_x[0] / k_nu).ToString() + " " + (w_avg_x[1] / k_nu).ToString() + " " + (w_avg_x[2] / k_nu).ToString()
                        );

                    Alignment_avg_rougthMovingAVG.WriteLine(SINSstate.Time.ToString() + " " + MovingAverageAccGyro[0] + " " + MovingAverageAccGyro[1] + " " + MovingAverageAccGyro[2] + " " + MovingAverageAccGyro[3] + " " + MovingAverageAccGyro[4]
                        + " " + MovingAverageAccGyro[5]);
                }




                // --- Вывод данных для формирования GRTV файла --- //
                if (SINSstate.flag_GRTV_output)
                {
                    GRTV_output.WriteLine(
                        SINSstate.Count
                        + " " + "4" + " "
                        + " " + SINSstate.F_z_orig[1] + " " + SINSstate.F_z_orig[2] + " " + SINSstate.F_z_orig[0]
                        + " " + SINSstate.W_z_orig[1] + " " + SINSstate.W_z_orig[2] + " " + SINSstate.W_z_orig[0]

                        + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Height
                        + " " + SINSstate.Vx_0[1] + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[2]

                        + " " + SINSstate.Heading + " " + SINSstate.Pitch + " " + SINSstate.Roll
                        + " " + SINSstate.Latitude + " 1 " + SINSstate.Longitude + " 1 " + SINSstate.Height + " 1"
                        + " " + SINSstate.Vx_0[1] + " 1 " + SINSstate.Vx_0[0] + " 1 " + SINSstate.Vx_0[2] + " 1"

                        + " " + SINSstate.OdometerData.odometer_left.Value_orig + " " + SINSstate.OdometerData.odometer_left.isReady_orig

                        //метка времени - отмечает момент времени формирования пакета СНС-данных
                        + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                        + " " + SINSstate.GPS_Data.gps_Latitude.Value_orig + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                        + " " + SINSstate.GPS_Data.gps_Longitude.Value_orig + " " + SINSstate.GPS_Data.gps_Longitude.isReady_orig
                        + " " + SINSstate.GPS_Data.gps_Altitude.Value_orig + " " + SINSstate.GPS_Data.gps_Altitude.isReady_orig
                        + " " + SINSstate.GPS_Data.gps_Vn.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                        + " " + SINSstate.GPS_Data.gps_Ve.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                        + " " + " 0 0" //Скорость GPS вертикальная
                        );
                }
            }

            // --- Вычисляем средние значения показаний каждого из датчиков
            f_avg[0] = f_avg[0] / k_f; w_avg[0] = w_avg[0] / k_nu;
            f_avg[1] = f_avg[1] / k_f; w_avg[1] = w_avg[1] / k_nu;
            f_avg[2] = f_avg[2] / k_f; w_avg[2] = w_avg[2] / k_nu;

            // --- вычисляем СКО датчиков
            for (int j = 1; j < k_f; j++)
            {
                sigma_f[0] += Math.Abs(array_sigma_f_1[j] - f_avg[0]);
                sigma_f[1] += Math.Abs(array_sigma_f_2[j] - f_avg[1]);
                sigma_f[2] += Math.Abs(array_sigma_f_3[j] - f_avg[2]);
            }
            sigma_f[0] = sigma_f[0] / k_f;
            sigma_f[1] = sigma_f[1] / k_f;
            sigma_f[2] = sigma_f[2] / k_f;


            for (int j = 1; j < k_nu; j++)
            {
                sigma_w[0] += Math.Abs(array_sigma_w_1[j] - w_avg[0]);
                sigma_w[1] += Math.Abs(array_sigma_w_2[j] - w_avg[1]);
                sigma_w[2] += Math.Abs(array_sigma_w_3[j] - w_avg[2]);
            }
            sigma_w[0] = sigma_w[0] / k_nu;
            sigma_w[1] = sigma_w[1] / k_nu;
            sigma_w[2] = sigma_w[2] / k_nu;


            // --- вычисляются шумы ньютонометров и дусов --- //
            for (int j = 0; j < 3; j++)
            {
                KalmanVars.Noise_Vel[j] = sigma_f[j];
                KalmanVars.Noise_Angl[j] = sigma_w[j];
            }

            // === По вертикальному шум обычно будет меньше на выставке, поэтому немного сглаживаем  === //
            //KalmanVars.Noise_Vel[2] = (KalmanVars.Noise_Vel[0] + KalmanVars.Noise_Vel[1]) / 2.0;
            //KalmanVars.Noise_Angl[2] = (KalmanVars.Noise_Angl[0] + KalmanVars.Noise_Angl[1]) / 2.0;


            SINSstate.Pitch = Math.Atan2(f_avg[1], Math.Sqrt(f_avg[0] * f_avg[0] + f_avg[2] * f_avg[2]));
            SINSstate.Roll = -Math.Atan2(f_avg[0], f_avg[2]);
            SINSstate.Heading = -Math.Atan2(w_avg_x[0], w_avg_x[1]);

            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            U_s = SINSstate.A_sx0 * SimpleOperations.U_x0(SINSstate.Latitude);

            double[] gilmertF = new double[3];
            gilmertF[2] = SimpleOperations.GilmertGravityForce(SINSstate.Latitude, SINSstate.Height);
            SimpleOperations.CopyArray(gilmertF, SINSstate.A_sx0 * gilmertF);

            // --- алгебраическая калибровка нулей ДУСов
            for (int j = 0; j < 3; j++)
                SINSstate.AlignAlgebraDrifts[j] = w_avg[j] - U_s[j];

            // --- алгебраическая калибровка нулей ньютонометров
            // --- Покскольку на начальной выставке значения ньютонометров могут быть константами, то по флагу
            if (SINSstate.AlgebraicCalibration_F_Zero == true)
            {
                for (int j = 0; j < 3; j++)
                    SINSstate.AlignAlgebraZeroF[j] = f_avg[j] - gilmertF[j];
            }

            SINSstate.Time_Alignment = SINSstate.Time;


            // --- Если заданы начальные углы в настройках, то берем их с поправками на углы докалибровки ---//
            if (SINSstate.Alignment_HeadingDetermined == true)
                SINSstate.Heading = SINSstate.Alignment_HeadingValue + SINSstate.alpha_kappa_3;
            if (SINSstate.Alignment_RollDetermined == true)
                SINSstate.Roll = SINSstate.Alignment_RollValue;
            if (SINSstate.Alignment_PitchDetermined == true)
                SINSstate.Pitch = SINSstate.Alignment_PitchValue - SINSstate.alpha_kappa_1;



            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time - SINSstate.Time_Alignment, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);


            Alignment_avg_rougth.Close();
            Alignment_avg_rougthMovingAVG.Close();
            return i;
        }

    }
}
