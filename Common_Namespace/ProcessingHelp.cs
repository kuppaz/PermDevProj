using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using Common_Namespace;

namespace Common_Namespace
{
    public partial class ProcessingHelp
    {

        /*-------------------------------Вспомогательные функции---------------------------------------------------------*/

        public static void DefSNSData(Proc_Help ProcHelp, SINS_State SINSstate)
        {
            //if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
            if (SINSstate.GPS_Data.gps_Latitude.Value > 0.1)
            {
                ProcHelp.LatSNS = SINSstate.GPS_Data.gps_Latitude.Value * 180 / Math.PI;
                ProcHelp.LongSNS = SINSstate.GPS_Data.gps_Longitude.Value * 180 / Math.PI;
                ProcHelp.AltSNS = SINSstate.GPS_Data.gps_Altitude.Value;
                ProcHelp.SpeedSNS = Math.Sqrt(SINSstate.GPS_Data.gps_Ve.Value * SINSstate.GPS_Data.gps_Ve.Value + SINSstate.GPS_Data.gps_Vn.Value * SINSstate.GPS_Data.gps_Vn.Value);
                ProcHelp.Ve_SNS = SINSstate.GPS_Data.gps_Ve.Value;
                ProcHelp.Vn_SNS = SINSstate.GPS_Data.gps_Vn.Value;
            }
        }

        public static void ReadSINSStateFromString(Proc_Help ProcHelp, StreamReader myFile, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int t = 0;
            string[] dataArray;

            ProcHelp.datastring = myFile.ReadLine();

            if (ProcHelp.datastring.Contains("Count") || ProcHelp.datastring.Contains("Latitude"))
                ProcHelp.datastring = myFile.ReadLine();

            if (myFile.EndOfStream == false)
            {

                dataArray = ProcHelp.datastring.Split(' ');

                for (int y = 0; y < dataArray.Length; y++)
                    if (dataArray[y] != "")
                        t++;
                string[] dataArray2 = new string[t];
                t = 0;

                for (int y = 0; y < dataArray.Length; y++)
                    if (dataArray[y] != "")
                    {
                        dataArray2[t] = dataArray[y];
                        t++;
                    }

                SINSstate.Count = Convert.ToDouble(dataArray2[0]);

                if (ProcHelp.initCount == false) { ProcHelp.initCount = true; SINSstate.initCount = SINSstate.Count - 1; }

                SINSstate.Time = (SINSstate.Count - SINSstate.initCount) * Math.Abs(SINSstate.timeStep);

                SINSstate.F_z[1] = Convert.ToDouble(dataArray2[1]);
                SINSstate.F_z[2] = Convert.ToDouble(dataArray2[2]);
                SINSstate.F_z[0] = Convert.ToDouble(dataArray2[3]);

                SINSstate.W_z[1] = Convert.ToDouble(dataArray2[4]);
                SINSstate.W_z[2] = Convert.ToDouble(dataArray2[5]);
                SINSstate.W_z[0] = Convert.ToDouble(dataArray2[6]);

                // --- Поворот показаний датчиков на заданные углы несоосности БИНС и динамических осей объекта --- //
                {
                    double[] fz = new double[3], Wz = new double[3];

                    fz[0] = SINSstate.F_z[0] - SINSstate.alpha_kappa_3 * SINSstate.F_z[1];
                    fz[1] = SINSstate.F_z[1] + SINSstate.alpha_kappa_3 * SINSstate.F_z[0] - SINSstate.alpha_kappa_1 * SINSstate.F_z[2];
                    fz[2] = SINSstate.F_z[2] + SINSstate.alpha_kappa_1 * SINSstate.F_z[1];

                    Wz[0] = SINSstate.W_z[0] - SINSstate.alpha_kappa_3 * SINSstate.W_z[1];
                    Wz[1] = SINSstate.W_z[1] + SINSstate.alpha_kappa_3 * SINSstate.W_z[0] - SINSstate.alpha_kappa_1 * SINSstate.W_z[2];
                    Wz[2] = SINSstate.W_z[2] + SINSstate.alpha_kappa_1 * SINSstate.W_z[1];

                    SimpleOperations.CopyArray(SINSstate.F_z, fz);
                    SimpleOperations.CopyArray(SINSstate.W_z, Wz);
                }


                //---Запоминаем координаты предыдущей контрольной точки
                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Latitude_prev.Value = SINSstate.GPS_Data.gps_Latitude.Value;
                    SINSstate.GPS_Data.gps_Longitude_prev.Value = SINSstate.GPS_Data.gps_Longitude.Value;
                    SINSstate.GPS_Data.gps_Altitude_prev.Value = SINSstate.GPS_Data.gps_Altitude.Value;

                    SINSstate.OdometerData.odometer_left_prev.Value = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerData.odometer_right_prev.Value = SINSstate.OdometerData.odometer_right.Value;
                }

                // --- Заполняем структуру показний GPS, если они имеются
                // --- Предполагается, что GPS записываются в WGS84
                SINSstate.GPS_Data.gps_Latitude.Value = Convert.ToDouble(dataArray2[7]);
                SINSstate.GPS_Data.gps_Latitude.isReady = Convert.ToInt32(dataArray2[8]);
                SINSstate.GPS_Data.gps_Longitude.Value = Convert.ToDouble(dataArray2[9]);
                SINSstate.GPS_Data.gps_Longitude.isReady = Convert.ToInt32(dataArray2[10]);
                SINSstate.GPS_Data.gps_Altitude.Value = Convert.ToDouble(dataArray2[11]);
                SINSstate.GPS_Data.gps_Altitude.isReady = Convert.ToInt32(dataArray2[12]);

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                    SINSstate.GPS_CounterOfPoints++;

                SINSstate.GPS_Data.gps_Vn.Value = Convert.ToDouble(dataArray2[13]);
                SINSstate.GPS_Data.gps_Vn.isReady = Convert.ToInt32(dataArray2[14]);
                SINSstate.GPS_Data.gps_Ve.Value = Convert.ToDouble(dataArray2[15]);
                SINSstate.GPS_Data.gps_Ve.isReady = Convert.ToInt32(dataArray2[16]);

                if (SINSstate.GPS_Data.gps_Latitude.isReady == 1)
                {
                    SINSstate.GPS_Data.gps_Vn.Value_prev = SINSstate.GPS_Data.gps_Vn.Value;
                    SINSstate.GPS_Data.gps_Ve.Value_prev = SINSstate.GPS_Data.gps_Ve.Value;
                }

                SINSstate.FLG_Stop = Convert.ToInt32(dataArray2[17]);

                // --- Обрабатываем ситуацию, когда одометр установлен неправильно и не введен масштабный коэффициент
                double Odo_SINS_VS_distance_coefficient = SINSstate.OdometerData.odometer_left.Value / SINSstate.distance_by_SINS;
                if (SINSstate.distance_by_SINS < 110.0 && Math.Abs(Odo_SINS_VS_distance_coefficient) > 0.2)
                {
                    if (SINSstate.distance_by_SINS > 5.0 && Odo_SINS_VS_distance_coefficient < 0)
                    {
                        SINSstate.OdometerData_Sign = -1;
                        SINSstate.OdometerLeftPrev *= SINSstate.OdometerData_Sign;
                    }
                    if (SINSstate.distance_by_SINS > 100.0 && SINSstate.OdometerData_RoughtScale_flag == 0 && Math.Abs(1 - Odo_SINS_VS_distance_coefficient) > 0.3)
                    {
                        SINSstate.OdometerData_RoughtScale = Odo_SINS_VS_distance_coefficient;
                        SINSstate.OdometerLeftPrev /= SINSstate.OdometerData_RoughtScale;
                        SINSstate.OdometerData_RoughtScale_flag = 1;
                    }
                }

                // --- Считываем показания одометров
                SINSstate.OdometerData.odometer_left.Value = (Convert.ToDouble(dataArray2[18]) - SINSstate.OdometerData.odometer_left.Value_Correction) * SINSstate.OdometerData_Sign / SINSstate.OdometerData_RoughtScale;
                SINSstate.OdometerData.odometer_left.isReady = Convert.ToInt32(dataArray2[19]);
                SINSstate.OdometerData.odometer_right.Value = (Convert.ToDouble(dataArray2[20]) - SINSstate.OdometerData.odometer_left.Value_Correction) * SINSstate.OdometerData_Sign / SINSstate.OdometerData_RoughtScale;
                SINSstate.OdometerData.odometer_right.isReady = Convert.ToInt32(dataArray2[21]);


                // --- ЗАГЛУШКА для файла со стоянкой, поскольку здесь всегда для одометра isReady = 0 ---
                if (SINSstate.DataInFileName.Contains("GCEF_format_Azimuth10B_450612_48H_17-Feb-2016") == true && SINSstate.Count % 5 == 0)
                    SINSstate.OdometerData.odometer_left.isReady = 1;
                // --- ЗАГЛУШКА ---


                SINSstate.Input_nMode = Convert.ToInt32(dataArray2[22]);


                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    // Проверка на случай, когда одометр начинает имзерять растояние не с нуля
                    if (SINSstate.firstNotNullOdoValue_flg == 0 && SINSstate.OdometerData.odometer_left.isReady == 1)
                    {
                        SINSstate.firstNotNullOdoValue_flg = 1;
                        if (SINSstate.OdometerData.odometer_left.Value > 1.0)
                        {
                            SINSstate.OdometerData.odometer_left.Value_Correction = SINSstate.OdometerData.odometer_left.Value;
                            SINSstate.OdometerData.odometer_left.Value -= SINSstate.OdometerData.odometer_left.Value_Correction;
                        }
                    }

                    SINSstate.OdometerData.odometer_left.Value_prev = SINSstate.OdometerData.odometer_left.Value;

                    SINSstate.OdoLimitMeasuresNum_Count++;
                    if (SINSstate.OdoLimitMeasuresNum_Count < SINSstate.OdoLimitMeasuresNum)
                    {
                        SINSstate.OdometerData.odometer_left.isReady = 2;
                        SINSstate.OdometerData.odometer_right.isReady = 2;
                    }
                    else
                        SINSstate.OdoLimitMeasuresNum_Count = 0;
                }


                // --- Делаем поправку на введенное значение ошибки масштаба одометра
                SINSstate.OdometerData.odometer_left.Value /= (1.0 + SINSstate.alpha_scaleError);
                SINSstate.OdometerData.odometer_right.Value /= (1.0 + SINSstate.alpha_scaleError);


                // --- Сохраняем оригинальные считанные значение, если проставлен флаг вывода в GRTV ---//
                if (SINSstate.flag_GRTV_output)
                {
                    for (int y = 0; y < 3; y++)
                    {
                        SINSstate.F_z_orig[y] = SINSstate.F_z[y] / 9.81;
                        SINSstate.W_z_orig[y] = SINSstate.W_z[y];
                    }

                    SINSstate.OdometerData.odometer_left.Value_orig = Convert.ToDouble(dataArray2[18]);
                    SINSstate.OdometerData.odometer_left.isReady_orig = Convert.ToInt32(dataArray2[19]);
                    if (SINSstate.OdometerData.odometer_left.isReady_orig != 1)
                    {
                        SINSstate.OdometerData.odometer_left.isReady_orig = 0;
                        SINSstate.OdometerData.odometer_left.Value_orig = SINSstate.OdometerData.odometer_left.Value_prev;
                    }

                    SINSstate.GPS_Data.gps_Vn.Value_orig = Convert.ToDouble(dataArray2[13]);
                    SINSstate.GPS_Data.gps_Vn.isReady_orig = Convert.ToInt32(dataArray2[14]);
                    if (SINSstate.GPS_Data.gps_Vn.isReady_orig != 1)
                    {
                        SINSstate.GPS_Data.gps_Vn.isReady_orig = 0;
                        SINSstate.GPS_Data.gps_Vn.Value_orig = SINSstate.GPS_Data.gps_Vn.Value_prev;
                    }
                    SINSstate.GPS_Data.gps_Ve.Value_orig = Convert.ToDouble(dataArray2[15]);
                    SINSstate.GPS_Data.gps_Ve.isReady_orig = Convert.ToInt32(dataArray2[16]);
                    if (SINSstate.GPS_Data.gps_Ve.isReady_orig != 1)
                    {
                        SINSstate.GPS_Data.gps_Ve.isReady_orig = 0;
                        SINSstate.GPS_Data.gps_Ve.Value_orig = SINSstate.GPS_Data.gps_Ve.Value_prev;
                    }



                    SINSstate.GPS_Data.gps_Latitude.Value_orig = Convert.ToDouble(dataArray2[7]);
                    SINSstate.GPS_Data.gps_Longitude.Value_orig = Convert.ToDouble(dataArray2[9]);
                    SINSstate.GPS_Data.gps_Altitude.Value_orig = Convert.ToDouble(dataArray2[11]);

                    if (SINSstate.GPS_Data.gps_Latitude.isReady != 1)
                    {
                        SINSstate.GPS_Data.gps_Latitude.Value_orig = SINSstate.GPS_Data.gps_Latitude_prev.Value;
                        SINSstate.GPS_Data.gps_Longitude.Value_orig = SINSstate.GPS_Data.gps_Longitude_prev.Value;
                        SINSstate.GPS_Data.gps_Altitude.Value_orig = SINSstate.GPS_Data.gps_Altitude_prev.Value;
                    }

                    SINSstate.GPS_Data.gps_Latitude.isReady_orig = Convert.ToInt32(dataArray2[8]);
                    SINSstate.GPS_Data.gps_Longitude.isReady_orig = Convert.ToInt32(dataArray2[10]);
                    SINSstate.GPS_Data.gps_Altitude.isReady_orig = Convert.ToInt32(dataArray2[12]);

                    if (SINSstate.GPS_Data.gps_Latitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Latitude.isReady_orig = 0;
                    if (SINSstate.GPS_Data.gps_Longitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Longitude.isReady_orig = 0;
                    if (SINSstate.GPS_Data.gps_Altitude.isReady_orig != 1)
                        SINSstate.GPS_Data.gps_Altitude.isReady_orig = 0;
                }

                SINSstate.GPS_Data.gps_Latitude_pnppk_sol.Value = Convert.ToDouble(dataArray2[25]);
                SINSstate.GPS_Data.gps_Longitude_pnppk_sol.Value = Convert.ToDouble(dataArray2[26]);
                SINSstate.GPS_Data.gps_Altitude_pnppk_sol.Value = Convert.ToDouble(dataArray2[27]);


                // --- Проставляем флаги, что перваястрока из файла с данными считана
                if (SINSstate.firstLineRead == false)
                {
                    SINSstate.Roll_prev = SINSstate.Roll;
                    SimpleOperations.CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
                    SimpleOperations.CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
                    SINSstate.OdometerStartValue = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.firstLineRead = true;
                }
            }
        }


        public static void OutPutInfo(int i, int start_i, Proc_Help ProcHelp, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars,
                StreamWriter Nav_FeedbackSolution,
                StreamWriter Nav_StateErrorsVector,
                StreamWriter Nav_Errors,
                StreamWriter GRTV_output,
                StreamWriter Nav_CovarianceDiagonal,
                StreamWriter KMLFileOut, StreamWriter KMLFileOut_PNPPK
            )
        {
            double Lat = 0.0, Long = 0.0;
            double[] Vx_0 = new double[3];

            Lat = SINSstate.Latitude;
            Long = SINSstate.Longitude;
            SimpleOperations.CopyArray(Vx_0, SINSstate.Vx_0);

            ProcHelp.distance = Math.Sqrt(Math.Pow((Lat - ProcHelp.LatSNS * SimpleData.ToRadian) * SimpleOperations.RadiusN(Lat, SINSstate.Height), 2) +
                                 Math.Pow((Long - ProcHelp.LongSNS * SimpleData.ToRadian) * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat), 2));
            ProcHelp.distance_from_start = Math.Sqrt(Math.Pow((Lat - SINSstate.Latitude_Start) * SimpleOperations.RadiusN(Lat, SINSstate.Height), 2) +
                                 Math.Pow((Long - SINSstate.Longitude_Start) * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat), 2));


            int iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_alphaBeta = SINSstate.value_iMx_alphaBeta;


            // --- Вывод в файл данных, необходимых для формирования GRTV бинарного фалйа --- //
            if (SINSstate.flag_GRTV_output == true)
            {
                if (i == start_i)
                {
                    StreamWriter GRTV_init_output = new StreamWriter(SimpleData.PathOutputString + "\\TXT for GRTV\\S_GRTV_init_output.txt");

                    GRTV_init_output.WriteLine(
                        "fSamplingInterval " + SINSstate.timeStep + "\n"
                        + "nInstallationModel 0\n"                               // 0 – продольная ось прибора совпадает с продольной осью объекта
                        + "flLatitudeID " + SINSstate.Latitude_Start + " 1\n"
                        + "flLongitudeID " + SINSstate.Longitude_Start + " 1\n"
                        + "flHeightID " + SINSstate.Altitude_Start + " 1\n"
                        + "flTrueHeadID " + SINSstate.Heading + " 1\n"     //начальная долгота, заданная с пульта [рад]
                        + "flAzimuthMisalignment 0.0 0\n"                          // Угол азимутального рассогласования
                        + "flElevation 0.0 0"                                    //начальный угол возвышения ИНС [рад]
                        );

                    GRTV_init_output.Close();
                }

                int modeGRTV = 16;
                if (i < ProcHelp.AlignmentCounts) modeGRTV = 8;

                GRTV_output.WriteLine(
                    SINSstate.Count
                    + " " + modeGRTV + " "
                    + " " + SINSstate.F_z_orig[1] + " " + SINSstate.F_z_orig[2] + " " + SINSstate.F_z_orig[0]
                    + " " + SINSstate.W_z_orig[1] + " " + SINSstate.W_z_orig[2] + " " + SINSstate.W_z_orig[0]

                    + " " + SINSstate.Latitude + " " + SINSstate.Longitude + " " + SINSstate.Height
                    + " " + SINSstate.Vx_0[1] + " " + SINSstate.Vx_0[0] + " " + SINSstate.Vx_0[2]

                    + " " + SINSstate.Heading + " " + SINSstate.Pitch + " " + SINSstate.Roll
                    + " " + SINSstate.Latitude + " 1 " + SINSstate.Longitude + " 1 " + SINSstate.Height + " 1"
                    + " " + SINSstate.Vx_0[1] + " 1 " + SINSstate.Vx_0[0] + " 1 " + SINSstate.Vx_0[2] + " 1"

                    + " " + SINSstate.OdometerData.odometer_left.Value_orig + " " + SINSstate.OdometerData.odometer_left.isReady_orig

                    // СНС-данные
                    //+ " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Latitude.Value_orig + " " + SINSstate.GPS_Data.gps_Latitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Longitude.Value_orig + " " + SINSstate.GPS_Data.gps_Longitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Altitude.Value_orig + " " + SINSstate.GPS_Data.gps_Altitude.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Vn.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                    //+ " " + SINSstate.GPS_Data.gps_Ve.Value_orig + " " + SINSstate.GPS_Data.gps_Vn.isReady_orig
                    // или пока заглушка:
                    + " 0" + " 0" + " 0" + " 0" + " 0" + " 0" + " 0" + " 0" + " 0" + " 0" + " 0"


                    + " " + " 0 0" //Скорость GPS вертикальная
                    );
            }


            /*----------------------------------OUTPUT FEEDBACK------------------------------------------------------*/
            if (i % SINSstate.FreqOutput == 0)
            {
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment) + " " + SINSstate.Count
                                    + " " + SINSstate.OdoTimeStepCount + " " + SimpleOperations.AbsoluteVectorValue(SINSstate.OdoSpeed_s)
                                    + " " + Math.Round(((SINSstate.Latitude - SINSstate.Latitude_Start) * SINSstate.R_n), 2)
                                    + " " + Math.Round(((SINSstate.Longitude - SINSstate.Longitude_Start) * SINSstate.R_e * Math.Cos(SINSstate.Latitude)), 2) + " " + SINSstate.Height
                                    + " " + ((SINSstate.Latitude) * SimpleData.ToDegree) + " " + ((SINSstate.Longitude) * SimpleData.ToDegree)
                                    + " " + Math.Round(SINSstate.Vx_0[0], 3) + " " + Math.Round(SINSstate.Vx_0[1], 3) + " " + Math.Round(SINSstate.Vx_0[2], 5)
                                    + " " + Math.Round((SINSstate.Heading * SimpleData.ToDegree), 8)
                                    + " " + Math.Round((SINSstate.Roll * SimpleData.ToDegree), 8) + " " + Math.Round((SINSstate.Pitch * SimpleData.ToDegree), 8)
                                    + " " + ProcHelp.distance
                                    + " " + (SINSstate.Height - ProcHelp.AltSNS)
                                    + " " + SINSstate.OdometerData.odometer_left.Value
                                    + " " + SINSstate.GPS_Data.gps_Latitude_pnppk_sol.Value * SimpleData.ToDegree
                                    + " " + SINSstate.GPS_Data.gps_Longitude_pnppk_sol.Value * SimpleData.ToDegree
                                    + " " + SINSstate.GPS_Data.gps_Altitude_pnppk_sol.Value
                                    ;

                Nav_FeedbackSolution.WriteLine(ProcHelp.datastring);
            }




            if (i % SINSstate.FreqOutput == 0)
            {
                /*----------------------------------OUTPUT ERRORS------------------------------------------------------*/
                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                            + " " + SINSstate.Cumulative_StateErrorVector[0] * SimpleOperations.RadiusN(Lat, SINSstate.Height)
                            + " " + SINSstate.Cumulative_StateErrorVector[1] * SimpleOperations.RadiusE(Lat, SINSstate.Height) * Math.Cos(Lat)
                            + " " + SINSstate.Cumulative_StateErrorVector[2]
                            + " " + SINSstate.Cumulative_StateErrorVector[3]
                            + " " + SINSstate.Cumulative_StateErrorVector[4]
                            + " " + SINSstate.Cumulative_StateErrorVector[5]
                            + " " + SINSstate.Cumulative_StateErrorVector[6] * SimpleData.ToDegree
                            + " " + SINSstate.Cumulative_StateErrorVector[7] * SimpleData.ToDegree
                            + " " + SINSstate.Cumulative_StateErrorVector[8] * SimpleData.ToDegree
                            ;

                Nav_Errors.WriteLine(ProcHelp.datastring);




                /*----------------------------------OUTPUT StateErrors------------------------------------------------------*/
                int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

                int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                    iMx_Nu0 = SINSstate.value_iMx_Nu0,
                    f0_12 = SINSstate.value_iMx_f0_12,
                    f0_3 = SINSstate.value_iMx_f0_3
                    ;

                double[] Vertical_ErrorConditionVector = new double[KalmanVars.Vertical_ErrorConditionVector_p.Length];
                SimpleOperations.CopyArray(Vertical_ErrorConditionVector, SINSstate.Vertical_Cumulative_KalmanErrorVector);

                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                    + " " + SINSstate.Cumulative_KalmanErrorVector[0]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[1]
                    + " " + Vertical_ErrorConditionVector[0]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_dV_12 + 0)]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_dV_12 + 1)]
                    + " " + Vertical_ErrorConditionVector[1]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 0)] * SimpleData.ToDegree
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 1)] * SimpleData.ToDegree
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_alphaBeta + 2)] * SimpleData.ToDegree
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 0)] * SimpleData.ToDegree * 3600.0
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 1)] * SimpleData.ToDegree * 3600.0
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(iMx_Nu0 + 2)] * SimpleData.ToDegree * 3600.0
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(f0_12 + 0)]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[(f0_12 + 1)]
                    + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_f0_3 + 0]
                    ;

                ProcHelp.datastring = ProcHelp.datastring
                    + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_r12_odo]
                    + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_r12_odo + 1]
                    + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_rOdo3]
                    ;

                if (SINSstate.Vertical_kappa1 > 0)
                    ProcHelp.datastring = ProcHelp.datastring + " " + Vertical_ErrorConditionVector[SINSstate.Vertical_kappa1] * SimpleData.ToDegree;
                else
                    ProcHelp.datastring = ProcHelp.datastring + " 0";

                if (iMx_kappa_3_ds > 0)
                    ProcHelp.datastring = ProcHelp.datastring
                        + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 0] * SimpleData.ToDegree
                        + " " + SINSstate.Cumulative_KalmanErrorVector[iMx_kappa_3_ds + 1];
                else
                    ProcHelp.datastring = ProcHelp.datastring + " 0 0";

                Nav_StateErrorsVector.WriteLine(ProcHelp.datastring);


                // ----------------------------------------------------------//
                // ----------Вывод в файл для GoogleEarth---------------------//
                // ----------------------------------------------------------//
                double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(SINSstate.Latitude, SINSstate.Longitude, SINSstate.Height, 1);

                KMLFileOut.WriteLine(PhiLambdaH_WGS84[1] * SimpleData.ToDegree
                        + "," + PhiLambdaH_WGS84[0] * SimpleData.ToDegree
                        + "," + (SINSstate.Time + SINSstate.Time_Alignment)
                        );

                double[] PhiLambdaH_WGS84_PNPPK = GeodesicVsGreenwich.Geodesic2Geodesic(SINSstate.GPS_Data.gps_Latitude_pnppk_sol.Value, SINSstate.GPS_Data.gps_Longitude_pnppk_sol.Value, SINSstate.GPS_Data.gps_Altitude_pnppk_sol.Value, 1);

                KMLFileOut_PNPPK.WriteLine(PhiLambdaH_WGS84_PNPPK[1] * SimpleData.ToDegree
                        + "," + PhiLambdaH_WGS84_PNPPK[0] * SimpleData.ToDegree
                        + "," + (SINSstate.Time + SINSstate.Time_Alignment)
                        );

            }


            if (i % Convert.ToInt32(Math.Round(1.0/SINSstate.Freq)) == 0)
            {
                int iMx = SimpleData.iMx, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;
                int iMx_dV_12 = SINSstate.value_iMx_dV_12, iMx_Nu0 = SINSstate.value_iMx_Nu0, f0_12 = SINSstate.value_iMx_f0_12, f0_3 = SINSstate.value_iMx_f0_3;
                int iMxV = SimpleData.iMx_Vertical, vert_f0_3 = SINSstate.Vertical_f0_3, vert_kappa1 = SINSstate.Vertical_kappa1, Vertical_rOdo3 = SINSstate.Vertical_rOdo3;

                ProcHelp.datastring = (SINSstate.Time + SINSstate.Time_Alignment)
                    + " " + KalmanVars.CovarianceMatrixS_p[0 * iMx + 0] 
                    + " " + KalmanVars.CovarianceMatrixS_p[1 * iMx + 1]

                    + " " + KalmanVars.Vertical_CovarianceMatrixS_m[0 * iMxV + 0]

                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_r12_odo + 0) * iMx + (iMx_r12_odo + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_r12_odo + 1) * iMx + (iMx_r12_odo + 1)]

                    + " " + KalmanVars.Vertical_CovarianceMatrixS_m[Vertical_rOdo3 * iMxV + Vertical_rOdo3]

                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 1)]

                    + " " + KalmanVars.Vertical_CovarianceMatrixS_m[1 * iMxV + 1]

                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 1)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 2)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 0) * iMx + (iMx_Nu0 + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 1) * iMx + (iMx_Nu0 + 1)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 2) * iMx + (iMx_Nu0 + 2)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(f0_12 + 0) * iMx + (f0_12 + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(f0_12 + 1) * iMx + (f0_12 + 1)]

                    + " " + KalmanVars.Vertical_CovarianceMatrixS_m[(vert_f0_3 + 0) * iMxV + (vert_f0_3 + 0)]
                    + " " + KalmanVars.Vertical_CovarianceMatrixS_m[(vert_kappa1 + 0) * iMxV + (vert_kappa1 + 0)]

                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 0) * iMx + (iMx_kappa_3_ds + 0)]
                    + " " + KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 1) * iMx + (iMx_kappa_3_ds + 1)]
                    ;
                Nav_CovarianceDiagonal.WriteLine(ProcHelp.datastring);
            }

        }

        public static void FillKMLOutputFile(SINS_State SINSstate, StreamWriter KMLFileOut, string Part, string append)
        {
            if (Part == "Start")
            {
                KMLFileOut.WriteLine("<?xml version='1.0' encoding='UTF-8'?>                                                 ");
                KMLFileOut.WriteLine("<kml xmlns='http://earth.google.com/kml/2.2'>                                          ");
                KMLFileOut.WriteLine("<Document>                                                                             ");
                KMLFileOut.WriteLine("<name>" + append + SINSstate.DataInFileName + "</name>");
                KMLFileOut.WriteLine("<visibility>1</visibility>                                                             ");
                KMLFileOut.WriteLine("<open>1</open>                                                                         ");
                KMLFileOut.WriteLine("<Style id='MarkerIcon'>                                                                ");
                KMLFileOut.WriteLine("        <IconStyle>                                                                    ");
                KMLFileOut.WriteLine("        <scale>1</scale>                                                               ");
                KMLFileOut.WriteLine("            <Icon>                                                                     ");
                KMLFileOut.WriteLine("                <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs.png</href>");
                KMLFileOut.WriteLine("            </Icon>                                                                    ");
                KMLFileOut.WriteLine("        </IconStyle>                                                                   ");
                KMLFileOut.WriteLine("</Style>                                                                               ");
                KMLFileOut.WriteLine("<Style id='MarkerLine'>                                                                ");
                KMLFileOut.WriteLine("        <LineStyle>                                                                    ");
                KMLFileOut.WriteLine("                <color>ff000000</color>                                                ");
                KMLFileOut.WriteLine("                <width>2</width>                                                       ");
                KMLFileOut.WriteLine("        </LineStyle>                                                                   ");
                KMLFileOut.WriteLine("</Style>                                                                               ");
                KMLFileOut.WriteLine("<Folder>                                                                               ");
                KMLFileOut.WriteLine("        <name>Path</name>                                                              ");
                KMLFileOut.WriteLine("        <visibility>1</visibility>                                                     ");
                KMLFileOut.WriteLine("        <open>0</open>                                                                 ");
                KMLFileOut.WriteLine("        <Placemark>                                                                    ");
                KMLFileOut.WriteLine("            <name>Markers</name>                                                       ");
                KMLFileOut.WriteLine("                <visibility>1</visibility>                                             ");
                KMLFileOut.WriteLine("                <description>The markers scheme</description>                          ");
                KMLFileOut.WriteLine("                <styleUrl>#MarkerLine</styleUrl>                                       ");
                KMLFileOut.WriteLine("                <LineString>                                                           ");
                KMLFileOut.WriteLine("                    <extrude>0</extrude>                                               ");
                KMLFileOut.WriteLine("                    <tessellate>1</tessellate>                                         ");
                KMLFileOut.WriteLine("                    <altitudeMode>clampToGround</altitudeMode>                         ");
                KMLFileOut.WriteLine("                    <coordinates>                                                      ");
            }
            else if (Part == "End")
            {
                KMLFileOut.WriteLine("                   </coordinates>");
                KMLFileOut.WriteLine("              </LineString>       ");
                KMLFileOut.WriteLine("          </Placemark>               ");
                KMLFileOut.WriteLine("</Folder>                        ");
                KMLFileOut.WriteLine("</Document>                      ");
                KMLFileOut.WriteLine("</kml>                           ");

            }
        }
    }
}
