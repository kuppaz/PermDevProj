using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace SINSProcessingModes
{
    public class SINS_Corrected
    {
        public static StreamWriter Nav_FeedbackSolution, Nav_Errors, Nav_StateErrorsVector;
        public static StreamWriter Nav_CovarianceDiagonal, KMLFileOut, KMLFileOut_PNPPK;


        public static void SINS_Corrected_Processing(int l, bool NowSmoothing, StreamReader myFile, SINS_State SINSstate, Kalman_Vars KalmanVars, Proc_Help ProcHelp, SINS_State SINSstate_OdoMod, StreamWriter GRTV_output)
        {
            int t = 0;

            SINSstate.firstLineRead = false;

            Nav_Errors = new StreamWriter(SimpleData.PathOutputString + "\\S_Errors.txt");
            Nav_StateErrorsVector = new StreamWriter(SimpleData.PathOutputString + "\\S_ErrVect.txt");
            Nav_FeedbackSolution = new StreamWriter(SimpleData.PathOutputString + "\\S_SlnFeedBack.txt");
            KMLFileOut = new StreamWriter(SimpleData.PathOutputString + "\\KMLout\\KML_" + SINSstate.DataInFileName + ".kml");
            KMLFileOut_PNPPK = new StreamWriter(SimpleData.PathOutputString + "\\KMLout\\KML_PNPPK_" + SINSstate.DataInFileName + ".kml");

            ProcessingHelp.FillKMLOutputFile(SINSstate, KMLFileOut, "Start", "");
            ProcessingHelp.FillKMLOutputFile(SINSstate, KMLFileOut_PNPPK, "Start", "PNPPK_");

            Nav_CovarianceDiagonal = new StreamWriter(SimpleData.PathOutputString + "\\S_CovarianceDiagonal.txt");
            Nav_CovarianceDiagonal.WriteLine("Time dR1 dR2 dR3 dRodo1 dRodo2 dRodo3 dV1 dV2 dV3 alpha1 alpha2 beta3 Nu1 Nu2 Nu3 dF1 dF2 dF3 kappa1 kappa3 ScaleErr");

            // --- Хидер для файла с ошибками горизонтального канала --- //
            string str = "count  dr1 dr2 dr3 dV1 dV2 dV3 Alpha1_grad Alpha2_grad Beta3_grad Nu_1_grad Nu_2_grad/h Nu_3_grad/h dF_1 dF_2 dF_3 odo_dr1 odo_dr2 odo_dr3 kappa1_grad kappa3_grad Scale";
            Nav_StateErrorsVector.WriteLine(str);

            // --- Хидер для файла с ошибками в большом --- //
            Nav_Errors.WriteLine("Time dLat  dLong dAltitude  dV_x1  dV_x2  dV_x3  dHeading_Grad  dRoll_Grad  dPitch_Grad");

            // --- Хидер для файла с параметрами состояния системы --- //
            Nav_FeedbackSolution.WriteLine("time  count  OdoCnt  OdoV  LatRelStart  LongRelStart Altitude Latitude  Longitude V_x1  V_x2  V_x3  Yaw  Roll  Pitch PositError AltError OdometerValue Lat_PNPPK_Solut Long_PNPPK_Solut H_PNPPK_Solut");


            int start_i = l;
            //=========================================================================//
            //=========================================================================//
            for (int i = start_i; ; i++)
            {
                // --- считываем строку из файла с входными данными и присваиваем значения соответствующим переменным
                ProcessingHelp.ReadSINSStateFromString(ProcHelp, myFile, SINSstate, SINSstate_OdoMod);

                if (myFile.EndOfStream == true)
                    break;

                if (t == 0)
                {
                    t = 1;
                    start_i = i;
                }

                ProcessingHelp.DefSNSData(ProcHelp, SINSstate);

                // --- SINSstate.OdoTimeStepCount - счетчик тактов БИНС между обновлениями показаний одометра
                SINSstate.OdoTimeStepCount++;

                //---------------- Формируем вектора измерений одометра ---------------//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    // -- обнуляем вектора
                    SINSstate.OdometerVector = SimpleOperations.NullingOfArray(SINSstate.OdometerVector);
                    SINSstate.OdoSpeed_s = SimpleOperations.NullingOfArray(SINSstate.OdoSpeed_s);

                    // -- задаем текущее обновленное значение по второй компоненте (продольной оси объекта)
                    SINSstate.OdometerVector[1] = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeftPrev;
                    SINSstate.OdoSpeed_s[1] = SINSstate.OdometerVector[1] / SINSstate.OdoTimeStepCount / SINSstate.timeStep;

                    //--- Берем оценку kappa_1 с вертикального канала ---//
                    if (SINSstate.Vertical_kappa1 > 0)
                        SINSstate.Cumulative_KappaEst[0] = SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_kappa1];

                    //--- Корректируем обновление данных с одометра по оценкам ошибок (обратные связи) ---//
                    SimpleOperations.CopyArray(SINSstate.OdoSpeed_s,
                        (Matrix.UnitMatrix(3) + Matrix.SkewSymmetricMatrix(SINSstate.Cumulative_KappaEst)) / (1.0 + SINSstate.Cumulative_KappaEst[1]) * SINSstate.OdoSpeed_s);
                    SimpleOperations.CopyArray(SINSstate.OdometerVector,
                        (Matrix.UnitMatrix(3) + Matrix.SkewSymmetricMatrix(SINSstate.Cumulative_KappaEst)) / (1.0 + SINSstate.Cumulative_KappaEst[1]) * SINSstate.OdometerVector);
                }


                //-------------------------- MAIN STEPS ------------------------------//
                // --- Интегрирование уравнений движения: как БИНС, так и одометрическое счисление
                SINSprocessing.StateIntegration_AT(SINSstate, KalmanVars, SINSstate_OdoMod);

                if (SINSstate.flag_AutonomouseSolution == false)
                {
                    // --- Формируется матрица А для фильтра Калмана
                    SINSprocessing.Make_A(SINSstate, KalmanVars, SINSstate_OdoMod);
                    // --- Формируется матрица шумов Q для фильтра
                    SINSprocessing.MatrixNoise_ReDef(SINSstate, KalmanVars);

                    // --- Формирование переходной матрицы F
                    KalmanProcs.Make_F(SINSstate.timeStep, KalmanVars, SINSstate);
                    // -- Прогноз фильтра
                    KalmanProcs.KalmanForecast(KalmanVars, SINSstate);
                }




                SINSstate.flag_UsingCorrection = false;
                //--------------- флаг коррекции по одометру---------------//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                    SINSstate.flag_UsingCorrection = true;

                //----------------------------ЭТАП КОРРЕКЦИИ start---------------------------------------------------------------------//
                //--- Счетчик cnt_measures заполняется по ходу, характеризует количество измерений, которые будут поданы на коррекцию ---//
                //--- Счетчики увеличиваются по мере появления доступных данных для формирования корректирующих измерений ---//
                KalmanVars.cnt_measures = 0;
                KalmanVars.Vertical_cnt_measures = 0;

                SimpleOperations.NullingOfArray(KalmanVars.Matrix_H);
                SimpleOperations.NullingOfArray(KalmanVars.Vertical_Matrix_H);

                // !!! В дальнейших функциях CheckPointProcessing и Make_H_CONTROLPOINTS формируется матрица коррекции H, путем добавления строк в случае, если они уместны

                // --- Функция CheckPointProcessing в случае, когда при запуске указан флаг коррекци по контрольным точкам, проверяет, есть ли для данного файла 
                // --- привязка контрольной точки к метке времени. Если есть, формируется корректирующее измерение и увеличиваютс ясчетчики KalmanVars.cnt_measures
                if (SINSstate.flag_using_Checkpotints)
                    CheckPointProcessing(SINSstate, SINSstate_OdoMod, KalmanVars);

                // --- функция Make_H_CONTROLPOINTS с поданными НУЛЕВЫМИ значениями широты и долготы, формирует измерение по заданной (начальной) высоте
                // --- если задан флаг коррекции по начальной высоте SINSstate.flag_first_N_meters_StartHeightCorrection.
                // --- дополнительно прореживаю коррекцию i % 5 - раз в 5 тактов
                if (SINSstate.first_N_meters_StartHeightCorrection_flag
                    && (SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerStartValue) <= SINSstate.first_N_meters_StartHeightCorrection_value
                    && i % 5 == 0
                    )
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 0.0, 0.0, SINSstate.Altitude_Start, SINSstate.Noise_Marker_PositionError);


                // --- Если запуск в режиме автономного решения БИНС, то коррекции нет --- //
                if (SINSstate.flag_AutonomouseSolution == true)
                    SINSstate.flag_UsingCorrection = false;




                bool tmp_flag_UsingCorrection = SINSstate.flag_UsingCorrection;
                int tmp_odometer_left_IsReady = SINSstate.OdometerData.odometer_left.isReady;
                if (SINSstate.Time + SINSstate.Time_Alignment < 1800.00 //1350.00
                    //&& SINSstate.Time + SINSstate.Time_Alignment > 1160.00
                    )
                {
                    //if (SINSstate.Time + SINSstate.Time_Alignment > 1160.00)
                    //    KalmanVars.Noise_Pos = 0.1;

                    if (SINSstate.flag_ControlPointCorrection == false)
                        SINSstate.flag_UsingCorrection = false;

                    SINSstate.OdometerData.odometer_left.isReady = 2;
                }
                //else
                //{
                //    KalmanVars.Noise_Pos = 1.0;
                //}

                if (SINSstate.Time + SINSstate.Time_Alignment < 1160.00)
                {
                    SINSstate.flag_UsingCorrection = true;
                    CorrectionModel.Make_H_KNS(KalmanVars, SINSstate, SINSstate_OdoMod);
                }

                if (i % 50 == 0)
                {
                    SimpleOperations.PrintMatrixToFile_TinyToZero(KalmanVars.CovarianceMatrixS_m, SimpleData.iMx, SimpleData.iMx, "CovarianceMatrixS_m");
                    SimpleOperations.PrintMatrixToFile_TinyToZero(KalmanVars.Matrix_A, SimpleData.iMx, SimpleData.iMx, "Matrix_A");
                }




                //---------------------------------------------------------------------//
                ProcHelp.corrected = 0;
                if (SINSstate.flag_UsingCorrection == true)
                {
                    //=== КОРРЕКЦИЯ В СЛУЧАЕ БИНС+ ОДОМЕТР - формируются соответствующие корректирующие измерения по разности координат БИНС и одометрического счисления ===//
                    if (SINSstate.OdometerData.odometer_left.isReady == 1)
                        CorrectionModel.Make_H_POSITION(KalmanVars, SINSstate, SINSstate_OdoMod, ProcHelp);

                    // --- Этап коррекции фильтра
                    KalmanProcs.KalmanCorrection(KalmanVars, SINSstate, SINSstate_OdoMod);
                    ProcHelp.corrected = 1;
                }
                /*----------------------------------------------------------------------------------------*/


                //--- Расчет корректирующего вектора состояния ---//
                if (SINSstate.flag_UsingCorrection == true)
                {
                    // --- Вычисление поправок в БОЛЬШОМ к параметрам состояния системы 
                    SINSprocessing.CalcStateErrors(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate_OdoMod, KalmanVars);
                    // --- Коррекция параметров состояния системы + рассчет кумулятивных величин-оценок дитчиков и ошибок, связанных с одометром
                    SINSprocessing.StateCorrection(KalmanVars.ErrorConditionVector_p, SINSstate, SINSstate_OdoMod, KalmanVars);
                }
                //----------------------------ЭТАП КОРРЕКЦИИ END---------------------------------//


                /*------------------------------------OUTPUT-------------------------------------------------*/
                //--- ВЫВОД данных в файлы ---//
                if (i != (SINSstate.LastCountForRead - 1))
                    ProcessingHelp.OutPutInfo(i, start_i, ProcHelp, SINSstate, SINSstate_OdoMod, KalmanVars, Nav_FeedbackSolution,
                        Nav_StateErrorsVector, Nav_Errors, GRTV_output, Nav_CovarianceDiagonal, KMLFileOut, KMLFileOut_PNPPK);

                //--- OUTPUT в консоль ---//
                // --- ProcHelp.distance - растояние от последнего GPS измерения/маркерной точки ---
                if (i % 2000 == 0)
                    Console.WriteLine(SINSstate.Count.ToString()
                        + ",  FromSNS=" + Math.Round(ProcHelp.distance, 2) + " м" + ",  FromStart=" + Math.Round(ProcHelp.distance_from_start, 2) + " м"
                        + ",  Vx_1=" + Math.Round(SINSstate.Vx_0[0], 2) + ",  Vx_2=" + Math.Round(SINSstate.Vx_0[1], 3));

                //--- РЕСТАРТ вектора состояния уравнений ошибок только если имел место шаг коррекции ---//
                if (SINSstate.flag_UsingCorrection == true)
                    SINSprocessing.NullingOfCorrectedErrors(SINSstate, KalmanVars);


                SINSstate.flag_UsingCorrection = tmp_flag_UsingCorrection;
                SINSstate.OdometerData.odometer_left.isReady = tmp_odometer_left_IsReady;


                //--- Переопределение значений данных одометра ---
                SINSprocessing.Redifinition_OdoCounts(SINSstate, SINSstate_OdoMod);
            }
            /*----------------------------------------END---------------------------------------------*/
            /*----------------------------------------------------------------------------------------*/

            ProcessingHelp.FillKMLOutputFile(SINSstate, KMLFileOut, "End", "");
            ProcessingHelp.FillKMLOutputFile(SINSstate, KMLFileOut_PNPPK, "End", "");

            Nav_StateErrorsVector.Close();
            Nav_FeedbackSolution.Close();
            Nav_Errors.Close();
            KMLFileOut.Close();
            KMLFileOut_PNPPK.Close();
        }







        //--- КОНТРОЛЬНЫЕ ТОЧКИ --- //
        public static void CheckPointProcessing(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            SINSstate.flag_ControlPointCorrection = false;

            // --- В качестве контрольных точем можно использовать GPS позиционную информация для коррекции БИНСового и одометрического счисления --- //
            if (SINSstate.GPS_Data.gps_Latitude.isReady == 1 && SINSstate.GPS_CounterOfPoints % 5 == 0)
            {
                //double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value, 0);
                //CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod
                //    , SINSstate.GPS_Data.gps_Latitude.Value, SINSstate.GPS_Data.gps_Longitude.Value, SINSstate.GPS_Data.gps_Altitude.Value, SINSstate.Noise_GPS_PositionError);
            }

            if (SINSstate.DataInFileName == "630 отрезки  19.10.16_19-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1350.00) < 0.01)
                {
                    SINSstate.flag_ControlPointCorrection = true;
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1800.00) < 0.01)
                {
                    SINSstate.flag_ControlPointCorrection = true;
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2300.00) < 0.01)
                {
                    SINSstate.flag_ControlPointCorrection = true;
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2750.00) < 0.01)
                {
                    SINSstate.flag_ControlPointCorrection = true;
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3215.00) < 0.01)
                {
                    SINSstate.flag_ControlPointCorrection = true;
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "630 отрезки 18.10.16_18-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1370.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1800.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2300.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2650.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3080.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "630 отрезки нов прошив 17.10.16_17-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1250.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1700.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2150.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2600.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3000.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "630 по точкам  19.10.16_19-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4900.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 7100.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "630 по точкам 18.10.16_18-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4950.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 7350.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "630 по точкам нов прошив 17.10.16_17-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4900.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 7300.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }
            if (SINSstate.DataInFileName == "630 с завода  19.10.16_19-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5000.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
            }
            if (SINSstate.DataInFileName == "630 с завода 18.10.16_18-oct-2016.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5000.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
            }





            if (SINSstate.DataInFileName == "636_ c завода 03.10.16_03-oct-2016,09-17-23.dat.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5315.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5565.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6000.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6500.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6950.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 7400.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "636_ c завода 29.09.16_29-sep-2016,09-20-30.dat.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4850.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5100.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 5550.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6000.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6425.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6434091, 10485904);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 187, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6900.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6433925, 10487154);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 176, SINSstate.Noise_Marker_PositionError);
                }
            }


            if (SINSstate.DataInFileName == "636_ c куликовки  03.10.16_03-oct-2016,12-42-16.dat.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4080.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6550.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "636_ c куликовки  29.09.16_29-sep-2016,12-13-15.dat.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4490.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6850.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "636_ c куликовки тц-2  30.09.16_30-sep-2016,13-22-00.dat.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4250.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6441129, 10465942);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 170, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 6800.00) < 0.01)
                {
                    double[] PhiLambdaH_SK42 = GeodesicVsGreenwich.Phi_Lambda_GAUSS_KRUGER(6431440, 10456655);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_SK42[0], PhiLambdaH_SK42[1], 160, SINSstate.Noise_Marker_PositionError);
                }
            }



            if (SINSstate.DataInFileName == "GCEF_format_Azimuth10B_450612_48H_17-Feb-2016.txt"
                || SINSstate.DataInFileName == "GCEF_format_Azimuth10B_450612_48H_17-Feb-2016_cut_5h.txt")
            {
                if (Convert.ToInt32(SINSstate.Count) % 100 == 0)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, SINSstate.Latitude_Start, SINSstate.Longitude_Start, SINSstate.Altitude_Start, SINSstate.Noise_Marker_PositionError);
            }

            // --- Суть следующегоо -- коррекция по конкретным точкам, если это возможно и нужно --- //
            if (SINSstate.DataInFileName == "GRTVout_GCEF_format (070715выезд завод).txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2642.15) < 0.01)
                {
                    double Lat = 58 + (1.0 + 37.80 / 60.0) / 60.0;
                    double Long = 56 + (30.0 + 33.86 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 224, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 224.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2851.00) < 0.01)
                {
                    double Lat = 58 + (2.0 + 58.48 / 60.0) / 60.0;
                    double Long = 56 + (33.0 + 12.72 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 202, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 202.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3995.17) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 58.02398 * SimpleData.ToRadian, 56.76146 * SimpleData.ToRadian, 187.0, SINSstate.Noise_Marker_PositionError);
            }

            if (SINSstate.DataInFileName == "GRTVout_GCEF_format (070715выезд куликовка).txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1356.94) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(58.0211888888 * SimpleData.ToRadian, 56.6468888888 * SimpleData.ToRadian, 203, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 203.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1865.32) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(58.0496305555 * SimpleData.ToRadian, 56.554555555 * SimpleData.ToRadian, 205, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 205.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2463.37) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(58.044925 * SimpleData.ToRadian, 56.4303305555 * SimpleData.ToRadian, 200, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 0.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2775.49) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(58.001077777 * SimpleData.ToRadian, 56.388552777 * SimpleData.ToRadian, 211, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 211.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3041.17) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(58.00486111 * SimpleData.ToRadian, 56.3199361 * SimpleData.ToRadian, 149, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 149.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3207.85) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.9903757777 * SimpleData.ToRadian, 56.2972866666 * SimpleData.ToRadian, 167, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 167.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3923.39) < 0.01)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.998705555 * SimpleData.ToRadian, 56.26555 * SimpleData.ToRadian, 160, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 160.0, SINSstate.Noise_Marker_PositionError);
                }
            }

            if (SINSstate.DataInFileName == "GRTV_Ekat_151029_1_zaezd.txt")
            {
                double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.062705555555 * SimpleData.ToRadian, 60.71558888888 * SimpleData.ToRadian, 306, 0);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 517.00) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 306.0, SINSstate.Noise_Marker_PositionError);

                //57.07005277777777, 60.7294472222222
                PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.07005277777777 * SimpleData.ToRadian, 60.7294472222222 * SimpleData.ToRadian, 296.0, 0);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 976.53) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1298.56) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1595.14) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1882.15) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2179.11) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2455.18) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2731.26) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3003.72) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3269.57) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3530.79) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3788.01) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4043.94) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);

                //57.0652472222222, 60.7137888888888
                PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.0652472222222 * SimpleData.ToRadian, 60.7137888888888 * SimpleData.ToRadian, 301.0, 0);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1093.95) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1401.97) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1694.35) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1982.84) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2276.19) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2551.57) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2825.05) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3094.27) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3359.65) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3619.96) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3876.01) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4137.07) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 301.0, SINSstate.Noise_Marker_PositionError);

                PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(57.062705555555 * SimpleData.ToRadian, 60.71558888888 * SimpleData.ToRadian, 306, 0);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4279.14) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 306.0, SINSstate.Noise_Marker_PositionError);
            }


            if (SINSstate.DataInFileName == "GRTV_Ekat_151029_2_zaezd.txt")
            {
                double Lat = 57 + (4.0 + 12.07 / 60.0) / 60.0;
                double Long = 60 + (43.0 + 45.77 / 60.0) / 60.0;
                double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 296.0, 0);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 631.75) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 999.50) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1391.70) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1842.00) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2242.10) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2642.60) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3025.55) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3381.75) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3668.25) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 296.0, SINSstate.Noise_Marker_PositionError);

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4003.00) < 0.01)
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 57.06235 * SimpleData.ToRadian, 60.71691 * SimpleData.ToRadian, 306.0, SINSstate.Noise_Marker_PositionError);
            }



            if (SINSstate.DataInFileName == "GRTV_ktn004_marsh16_afterbdnwin_20032012.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4400.0) < 0.02)
                {
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(56.28916 * SimpleData.ToRadian, 43.08689 * SimpleData.ToRadian, 96, 1);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 96.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2497.0) < 0.02)
                {
                    double Lat = 56 + (18.0 + 16.04 / 60.0) / 60.0;
                    double Long = 43 + (6.0 + 59.69 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 93, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 93.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1767.27) < 0.01)
                {
                    double Lat = 56 + (19.0 + 4.76 / 60.0) / 60.0;
                    double Long = 43 + (6.0 + 42.8 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 99, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 99.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2913.00) < 0.01)
                {
                    double Lat = 56 + (19.0 + 18.33 / 60.0) / 60.0;
                    double Long = 43 + (7.0 + 36.44 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 99, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 99.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4164.50) < 0.01)
                {
                    double Lat = 56 + (17.0 + 20.72 / 60.0) / 60.0;
                    double Long = 43 + (4.0 + 53.61 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 98, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 98.0, SINSstate.Noise_Marker_PositionError);
                }
            }
            if (SINSstate.DataInFileName == "GRTV_ktn004_marsh16_repeat_21032012.txt")
            {
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4250.0) < 0.02)
                {
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, 56.28916 * SimpleData.ToRadian, 43.08689 * SimpleData.ToRadian, 96.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 2142.0) < 0.01)
                {
                    double Lat = 56 + (18.0 + 16.04 / 60.0) / 60.0;
                    double Long = 43 + (6.0 + 59.69 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 93, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 93.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1446.55) < 0.01 || Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 3226.02) < 0.01)
                {
                    double Lat = 56 + (19.0 + 4.80 / 60.0) / 60.0;
                    double Long = 43 + (6.0 + 43.05 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 99, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 99.0, SINSstate.Noise_Marker_PositionError);
                }

                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1670.00) < 0.01)
                {
                    double Lat = 56 + (19.0 + 18.32 / 60.0) / 60.0;
                    double Long = 43 + (7.0 + 36.30 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 99, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 99.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 1003.70) < 0.01)
                {
                    double Lat = 56 + (18.0 + 16.90 / 60.0) / 60.0;
                    double Long = 43 + (5.0 + 3.72 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 90, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 90.0, SINSstate.Noise_Marker_PositionError);
                }
                if (Math.Abs(SINSstate.Time + SINSstate.Time_Alignment - 4011.05) < 0.01)
                {
                    double Lat = 56 + (17.0 + 20.73 / 60.0) / 60.0;
                    double Long = 43 + (4.0 + 53.74 / 60.0) / 60.0;
                    double[] PhiLambdaH_WGS84 = GeodesicVsGreenwich.Geodesic2Geodesic(Lat * SimpleData.ToRadian, Long * SimpleData.ToRadian, 98, 0);
                    CorrectionModel.Make_H_CONTROLPOINTS(KalmanVars, SINSstate, SINSstate_OdoMod, PhiLambdaH_WGS84[0], PhiLambdaH_WGS84[1], 98.0, SINSstate.Noise_Marker_PositionError);
                }
            }
        }

    }
}
