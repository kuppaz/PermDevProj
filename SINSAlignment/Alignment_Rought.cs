﻿using System;
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
            double[] f_avg = new double[3], f_sum_squared = new double[3];
            double[] w_avg = new double[3], w_sum_squared = new double[3];
            double[] w_avg_x = new double[3]; double[] U_s = new double[3];
            Matrix A_xs = new Matrix(3, 3);

            StreamWriter Alignment_avg_rougth = new StreamWriter(SimpleData.PathOutputString + "\\Alignment\\Alignment_avg_rougth.txt");
            StreamWriter Alignment_InputData = new StreamWriter(SimpleData.PathOutputString + "\\Alignment\\Alignment_InputData.txt");
            StreamWriter Alignment_avg_rougthMovingAVG = new StreamWriter(SimpleData.PathOutputString + "\\Alignment\\Alignment_avg_rougth_MovingAVG.txt");

            // --- вектора СКО
            double[] sigma_f = new double[3];
            double[] sigma_w = new double[3];

            Alignment_avg_rougth.WriteLine("time f_1 f_2 f_3 w_1 w_2 w_3 heading roll pitch Latitude");
            Alignment_avg_rougthMovingAVG.WriteLine("time MA_f_1 MA_f_2 MA_f_3 MA_w_1 MA_w_2 MA_w_3");
            Alignment_InputData.WriteLine("time f_1 f_1_avg f_1_sigma f_2 f_2_avg f_2_sigma f_3 f_3_avg f_3_sigma w_1 w_1_avg w_1_sigma w_2 w_2_avg w_2_sigma w_3 w_3_avg w_3_sigma");


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
            // --- вспомогательные массивы
            double[] array_f_1 = new double[MovingWindow], array_f_2 = new double[MovingWindow], array_f_3 = new double[MovingWindow];
            double[] array_w_1 = new double[MovingWindow], array_w_2 = new double[MovingWindow], array_w_3 = new double[MovingWindow];

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

                int k_mode = k % MovingWindow;
                array_f_1[k_mode] = SINSstate.F_z[0];
                array_f_2[k_mode] = SINSstate.F_z[1];
                array_f_3[k_mode] = SINSstate.F_z[2];
                array_w_1[k_mode] = SINSstate.W_z[0];
                array_w_2[k_mode] = SINSstate.W_z[1];
                array_w_3[k_mode] = SINSstate.W_z[2];

                // --- Вычисляем среднее значение показаний акселерометров. Цель - детектирование константы в показаниях ньютонометров
                double tmp_f1_avg = 0.0, tmp_w1_avg = 0.0;
                int u = 0;
                for (u = 1; u <= Math.Min(i, 50); u++)
                {
                    if (k_mode - u < 0)
                        tmp_f1_avg += array_f_1[MovingWindow + k_mode - u];
                    else
                        tmp_f1_avg += array_f_1[k_mode - u];
                }
                tmp_f1_avg /= (u - 1);

                // --- Вычисляем среднее значение показаний ДУСов
                u = 0;
                for (u = 1; u <= Math.Min(i, 50); u++)
                {
                    if (k_mode - u < 0)
                        tmp_w1_avg += array_w_1[MovingWindow + k_mode - u];
                    else
                        tmp_w1_avg += array_w_1[k_mode - u];
                }
                tmp_w1_avg /= (u - 1);


                // --- Если показания датчиков меняются, то заполняем соответствующие массивы
                if (SINSstate.NoiseParamDetermin_mode != 1 || SINSstate.NoiseParamDetermin_mode == 1 && SINSstate.i_global > SINSstate.NoiseParamDetermin_startTime && SINSstate.i_global < SINSstate.NoiseParamDetermin_endTime)
                {
                    if (Math.Abs(tmp_f1_avg - array_f_1[k_mode]) > 1E-9)
                    {
                        f_avg[0] += SINSstate.F_z[0];
                        f_avg[1] += SINSstate.F_z[1];
                        f_avg[2] += SINSstate.F_z[2];
                        f_sum_squared[0] += SINSstate.F_z[0] * SINSstate.F_z[0];
                        f_sum_squared[1] += SINSstate.F_z[1] * SINSstate.F_z[1];
                        f_sum_squared[2] += SINSstate.F_z[2] * SINSstate.F_z[2];
                        k_f++;
                    }

                    // --- Если показания датчиков меняются, то заполняем соответствующие массивы
                    if (Math.Abs(tmp_w1_avg - array_w_1[k_mode]) > 1E-9)
                    {
                        w_avg[0] += SINSstate.W_z[0];
                        w_avg[1] += SINSstate.W_z[1];
                        w_avg[2] += SINSstate.W_z[2];
                        w_sum_squared[0] += SINSstate.W_z[0] * SINSstate.W_z[0];
                        w_sum_squared[1] += SINSstate.W_z[1] * SINSstate.W_z[1];
                        w_sum_squared[2] += SINSstate.W_z[2] * SINSstate.W_z[2];
                        k_nu++;
                    }

                    if (SINSstate.i_global % 250 == 0 && k_f > 1 && k_nu > 1)
                    {
                        // --- вычисляем СКО датчиков в процессе
                        sigma_f[0] = Math.Sqrt((f_sum_squared[0] - k_f * Math.Pow(f_avg[0] / k_f, 2)) / (k_f - 1));
                        sigma_f[1] = Math.Sqrt((f_sum_squared[1] - k_f * Math.Pow(f_avg[1] / k_f, 2)) / (k_f - 1));
                        sigma_f[2] = Math.Sqrt((f_sum_squared[2] - k_f * Math.Pow(f_avg[1] / k_f, 2)) / (k_f - 1));

                        sigma_w[0] = Math.Sqrt((w_sum_squared[0] - k_nu * Math.Pow(w_avg[0] / k_nu, 2)) / (k_nu - 1));
                        sigma_w[1] = Math.Sqrt((w_sum_squared[1] - k_nu * Math.Pow(w_avg[1] / k_nu, 2)) / (k_nu - 1));
                        sigma_w[2] = Math.Sqrt((w_sum_squared[2] - k_nu * Math.Pow(w_avg[2] / k_nu, 2)) / (k_nu - 1));
                    }


                }



                k++;

                // --- Вычисление скользящего среднего для его вывода в файл и только
                SimpleOperations.NullingOfArray(MovingAverageAccGyro);
                for (int u1 = 1; u1 < Math.Min(k, MovingWindow); u1++)
                {
                    MovingAverageAccGyro[0] += array_f_1[u1];
                    MovingAverageAccGyro[1] += array_f_2[u1];
                    MovingAverageAccGyro[2] += array_f_3[u1];
                    MovingAverageAccGyro[3] += array_w_1[u1];
                    MovingAverageAccGyro[4] += array_w_2[u1];
                    MovingAverageAccGyro[5] += array_w_3[u1];
                }
                for (int u1 = 0; u1 < 6; u1++)
                    MovingAverageAccGyro[u1] = MovingAverageAccGyro[u1] / (Math.Min(k, MovingWindow) - 1);


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
                    Alignment_avg_rougth.WriteLine(SINSstate.Time.ToString()
                        + " " + (f_avg[0] / Math.Max(k_f, 1)).ToString() + " " + (f_avg[1] / Math.Max(k_f, 1)).ToString() + " " + (f_avg[2] / Math.Max(k_f, 1)).ToString()
                        + " " + (w_avg[0] / Math.Max(k_nu, 1)).ToString() + " " + (w_avg[1] / Math.Max(k_nu, 1)).ToString() + " " + (w_avg[2] / Math.Max(k_nu, 1)).ToString()
                        + " " + (Heading * SimpleData.ToDegree).ToString() + " " + (Roll * SimpleData.ToDegree).ToString()
                        + " " + (Pitch * SimpleData.ToDegree).ToString() + " " + Latitude.ToString()
                        + " " + (w_avg_x[0] / k_nu).ToString() + " " + (w_avg_x[1] / k_nu).ToString() + " " + (w_avg_x[2] / k_nu).ToString()
                        );

                    Alignment_avg_rougthMovingAVG.WriteLine(SINSstate.Time.ToString() + " " + MovingAverageAccGyro[0] + " " + MovingAverageAccGyro[1] + " " + MovingAverageAccGyro[2] + " " + MovingAverageAccGyro[3] + " " + MovingAverageAccGyro[4]
                        + " " + MovingAverageAccGyro[5]);
                }


                // --- Вывод в файл показаний датчиков, среднего и сигмы. Для аналитики
                Alignment_InputData.WriteLine(SINSstate.Time
                        + " " + SINSstate.F_z[0] + " " + f_avg[0] / Math.Max(k_f, 1) + " " + (f_avg[0] / Math.Max(k_f, 1) + sigma_f[0])
                        + " " + SINSstate.F_z[1] + " " + f_avg[1] / Math.Max(k_f, 1) + " " + (f_avg[1] / Math.Max(k_f, 1) + sigma_f[1])
                        + " " + SINSstate.F_z[2] + " " + f_avg[2] / Math.Max(k_f, 1) + " " + (f_avg[2] / Math.Max(k_f, 1) + sigma_f[2])
                        + " " + SINSstate.W_z[0] + " " + w_avg[0] / Math.Max(k_nu, 1) + " " + (w_avg[0] / Math.Max(k_nu, 1) + sigma_w[0])
                        + " " + SINSstate.W_z[1] + " " + w_avg[1] / Math.Max(k_nu, 1) + " " + (w_avg[1] / Math.Max(k_nu, 1) + sigma_w[1])
                        + " " + SINSstate.W_z[2] + " " + w_avg[2] / Math.Max(k_nu, 1) + " " + (w_avg[2] / Math.Max(k_nu, 1) + sigma_w[2])
                    );



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

            //sigma_mu = Math.Sqrt(sigma_mu);

            // --- Вычисляем средние значения показаний каждого из датчиков
            f_avg[0] = f_avg[0] / k_f; w_avg[0] = w_avg[0] / k_nu;
            f_avg[1] = f_avg[1] / k_f; w_avg[1] = w_avg[1] / k_nu;
            f_avg[2] = f_avg[2] / k_f; w_avg[2] = w_avg[2] / k_nu;

            // --- вычисляем СКО датчиков
            sigma_f[0] = Math.Sqrt((f_sum_squared[0] - k_f * f_avg[0] * f_avg[0]) / (k_f - 1));
            sigma_f[1] = Math.Sqrt((f_sum_squared[1] - k_f * f_avg[1] * f_avg[1]) / (k_f - 1));
            sigma_f[2] = Math.Sqrt((f_sum_squared[2] - k_f * f_avg[2] * f_avg[2]) / (k_f - 1));

            sigma_w[0] = Math.Sqrt((w_sum_squared[0] - k_nu * w_avg[0] * w_avg[0]) / (k_nu - 1));
            sigma_w[1] = Math.Sqrt((w_sum_squared[1] - k_nu * w_avg[1] * w_avg[1]) / (k_nu - 1));
            sigma_w[2] = Math.Sqrt((w_sum_squared[2] - k_nu * w_avg[2] * w_avg[2]) / (k_nu - 1));


            // --- вычисляются шумы ньютонометров и дусов --- //
            for (int j = 0; j < 3; j++)
            {
                // --- Если двигатель на стоянке включен, то уменьшаем шум
                double decrementNoiseF = 1, decrementNoiseNu = 1;
                if (SINSstate.AlignmentEngineIsOff == 0)
                {
                    decrementNoiseF = 4;
                    decrementNoiseNu = 7;
                }

                KalmanVars.Noise_Vel[j] = sigma_f[j] / decrementNoiseF;
                KalmanVars.Noise_Angl[j] = sigma_w[j] / decrementNoiseNu;
            }

            // --- Если выбран режим задание конкретных значений сигм шумов датчиков
            if (SINSstate.NoiseParamDetermin_mode == 2)
            {
                for (int j = 0; j < 3; j++)
                {
                    KalmanVars.Noise_Vel[j] = SINSstate.NoiseParamDetermin_SigmaValueF;
                    KalmanVars.Noise_Angl[j] = SINSstate.NoiseParamDetermin_SigmaValueNu;
                }
            }


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
            for (int j = 0; j < 3; j++)
                SINSstate.AlignAlgebraZeroF[j] = f_avg[j] - gilmertF[j];


            SINSstate.Time_Alignment = SINSstate.Time;


            // --- Если заданы начальные углы в настройках, то берем их с поправками на углы докалибровки ---//
            if (SINSstate.Alignment_HeadingDetermined == true)
                SINSstate.Heading = SINSstate.Alignment_HeadingValue + SINSstate.alpha_kappa_3 - SINSstate.initError_kappa_3;
            if (SINSstate.Alignment_RollDetermined == true)
                SINSstate.Roll = SINSstate.Alignment_RollValue;
            if (SINSstate.Alignment_PitchDetermined == true)
                SINSstate.Pitch = SINSstate.Alignment_PitchValue - SINSstate.alpha_kappa_1 + SINSstate.initError_kappa_1;



            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time - SINSstate.Time_Alignment, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);


            Alignment_avg_rougth.Close();
            Alignment_InputData.Close();
            Alignment_avg_rougthMovingAVG.Close();
            return i;
        }

    }
}
