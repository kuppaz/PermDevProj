using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Common_Namespace;

namespace Common_Namespace
{
    public class SINSprocessing : SimpleOperations
    {
        public static double dVh_global = 0.0;
        public static int Can = 0;

        public static void Redifinition_OdoCounts(SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            if (SINSstate.OdometerData.odometer_left.isReady == 1)
            {
                if (SINSstate.flag_UsingCorrection == true || SINSstate.flag_AutonomouseSolution == true)
                {
                    SINSstate.OdometerLeftPrev = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerRightPrev = SINSstate.OdometerData.odometer_right.Value;
                    SINSstate.OdoTimeStepCount = 0;

                    SINSstate.odotime_prev = SINSstate.Time;

                    SINSstate.Latitude_prev = SINSstate.Latitude;
                    SINSstate.Longitude_prev = SINSstate.Longitude;
                    SINSstate.Altitude_prev = SINSstate.Height;

                    /* --- Запоминаем предыдущие значения показания одометра --- */
                    for (int i = 0; i < SINSstate.OdometerLeft_ArrayOfPrev.Length - 1; i++)
                    {
                        SINSstate.OdometerLeft_ArrayOfPrev[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i] 
                            = SINSstate.OdometerLeft_ArrayOfPrev[SINSstate.OdometerLeft_ArrayOfPrev.Length -1 - i - 1];
                        SINSstate.OdometerLeft_ArrayOfPrevTime[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i]
                            = SINSstate.OdometerLeft_ArrayOfPrevTime[SINSstate.OdometerLeft_ArrayOfPrev.Length - 1 - i - 1];
                    }
                    SINSstate.OdometerLeft_ArrayOfPrev[0] = SINSstate.OdometerData.odometer_left.Value;
                    SINSstate.OdometerLeft_ArrayOfPrevTime[0] = SINSstate.Time + SINSstate.Time_Alignment;
                }
            }
        }






        public static void ApplyCompensatedErrorsToSolution(SINS_State SINSstate)
        {
            if (SINSstate.Heading > Math.PI) SINSstate.Heading = SINSstate.Heading - 2.0 * Math.PI;
            if (SINSstate.Heading < -Math.PI) SINSstate.Heading = SINSstate.Heading + 2.0 * Math.PI;

            //корректированная матрица ориентации
            SINSstate.A_sx0 = A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            SINSstate.Omega_x[0] = -(SINSstate.Vx_0[1] + SINSstate.Vx_0_prev[1]) / 2.0 / SINSstate.R_n;
            SINSstate.Omega_x[1] = (SINSstate.Vx_0[0] + SINSstate.Vx_0_prev[0]) / 2.0 / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            SINSstate.g = 9.78049 * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014;
            SINSstate.g -= 2 * 0.000001538 * SINSstate.Height;
        }



        public static void CalcStateErrors(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            SINSstate.DeltaLatitude = ErrorVector[1] / SINSstate.R_n;
            SINSstate.DeltaLongitude = ErrorVector[0] / SINSstate.R_e / Math.Cos(SINSstate.Latitude);

            SINSstate.DeltaV_1 = ErrorVector[iMx_dV_12 + 0];
            SINSstate.DeltaV_2 = ErrorVector[iMx_dV_12 + 1];

            SINSstate.DeltaRoll = -(ErrorVector[iMx_alphaBeta + 0] * Math.Sin(SINSstate.Heading) + ErrorVector[iMx_alphaBeta + 1] * Math.Cos(SINSstate.Heading)) / Math.Cos(SINSstate.Pitch);
            SINSstate.DeltaPitch = -ErrorVector[iMx_alphaBeta + 0] * Math.Cos(SINSstate.Heading) + ErrorVector[iMx_alphaBeta + 1] * Math.Sin(SINSstate.Heading);
            SINSstate.DeltaHeading = ErrorVector[iMx_alphaBeta + 2] + SINSstate.DeltaRoll * Math.Sin(SINSstate.Pitch);

            //--- Поправки к одометрическому счислению ---//
            SINSstate_OdoMod.DeltaLatitude = ErrorVector[iMx_r12_odo + 1] / SINSstate_OdoMod.R_n;
            SINSstate_OdoMod.DeltaLongitude = ErrorVector[iMx_r12_odo + 0] / SINSstate_OdoMod.R_e / Math.Cos(SINSstate_OdoMod.Latitude);
     

            // ----------------------------------------------------------//
            // -------Поправки на основе оценок вертикального канала-----------//
            // ----------------------------------------------------------//
            SINSstate.DeltaAltitude = KalmanVars.Vertical_ErrorConditionVector_p[0];
            SINSstate.DeltaV_3 = KalmanVars.Vertical_ErrorConditionVector_p[1];

            SINSstate_OdoMod.DeltaAltitude = KalmanVars.Vertical_ErrorConditionVector_p[SINSstate.Vertical_rOdo3];
        }



        public static void StateCorrection(double[] ErrorVector, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars)
        {
            SINSstate.Latitude = SINSstate.Latitude - SINSstate.DeltaLatitude;
            SINSstate.Longitude = SINSstate.Longitude - SINSstate.DeltaLongitude;


            SINSstate.Vx_0[0] = SINSstate.Vx_0[0] - SINSstate.DeltaV_1;
            SINSstate.Vx_0[1] = SINSstate.Vx_0[1] - SINSstate.DeltaV_2;

            SINSstate.Roll = SINSstate.Roll - SINSstate.DeltaRoll;
            SINSstate.Pitch = SINSstate.Pitch - SINSstate.DeltaPitch;
            SINSstate.Heading = SINSstate.Heading - SINSstate.DeltaHeading;

            // --- Применение обновленных значений углов ориентации (переопределение матриц ориентации)
            SINSprocessing.ApplyCompensatedErrorsToSolution(SINSstate);

            //--- Ведем расчет оценки ошибок модели одометра в случае обратных связей ---//
            if (SINSstate.value_iMx_kappa_3_ds > 0)
            {
                SINSstate.Cumulative_KappaEst[2] += ErrorVector[SINSstate.value_iMx_kappa_3_ds + 0];
                SINSstate.Cumulative_KappaEst[1] += ErrorVector[SINSstate.value_iMx_kappa_3_ds + 1];
            }

            //--- Кумулируем ошибки вектора ошибок "x" для вывода в файлы ---//
            //--- Суммируются все компоненты вектора ошибок x, но используются только те компоненты, которые соответствуют датчикам и одометру
            for (int i = 0; i < SimpleData.iMx; i++)
                SINSstate.Cumulative_KalmanErrorVector[i] += ErrorVector[i];

            //--- Кумулируем ошибки в большом для вывода в файлы, чисто для возможности визуализации---//
            SINSstate.Cumulative_StateErrorVector[0] += SINSstate.DeltaLatitude;
            SINSstate.Cumulative_StateErrorVector[1] += SINSstate.DeltaLongitude;
            SINSstate.Cumulative_StateErrorVector[2] += SINSstate.DeltaAltitude;
            SINSstate.Cumulative_StateErrorVector[3] += SINSstate.DeltaV_1;
            SINSstate.Cumulative_StateErrorVector[4] += SINSstate.DeltaV_2;
            SINSstate.Cumulative_StateErrorVector[5] += SINSstate.DeltaV_3;
            SINSstate.Cumulative_StateErrorVector[6] += SINSstate.DeltaHeading;
            SINSstate.Cumulative_StateErrorVector[7] += SINSstate.DeltaRoll;
            SINSstate.Cumulative_StateErrorVector[8] += SINSstate.DeltaPitch;



            //--- Поправки к одометрическому счисления ---
            SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude - SINSstate_OdoMod.DeltaLatitude;
            SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude - SINSstate_OdoMod.DeltaLongitude;

            SINSstate_OdoMod.A_x0n = A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
            SINSstate_OdoMod.A_nx0 = SINSstate_OdoMod.A_x0n.Transpose();
           

            // ----------------------------------------------------------//
            // --------------Поправки к вертикальному каналу----------------//
            // ----------------------------------------------------------//
            SINSstate.Vx_0[2] = SINSstate.Vx_0[2] - SINSstate.DeltaV_3;
            SINSstate.Height = SINSstate.Height - SINSstate.DeltaAltitude;

            SINSstate_OdoMod.Height = SINSstate_OdoMod.Height - SINSstate_OdoMod.DeltaAltitude;

            for (int i = 0; i < SimpleData.iMx_Vertical; i++)
                SINSstate.Vertical_Cumulative_KalmanErrorVector[i] += KalmanVars.Vertical_ErrorConditionVector_p[i];
        }



        public static void NullingOfCorrectedErrors(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {

            for (int i = 0; i < SimpleData.iMx; i++)
            {
                KalmanVars.ErrorConditionVector_p[i] = 0.0;
                KalmanVars.ErrorConditionVector_m[i] = 0.0;
            }

            // ----------------------------------------------------------//
            // -------------Для вертикального канала----------------------//
            for (int i = 0; i < SimpleData.iMx_Vertical; i++)
            {
                KalmanVars.Vertical_ErrorConditionVector_p[i] = 0.0;
                KalmanVars.Vertical_ErrorConditionVector_m[i] = 0.0;
            }
        }







        public static void InitOfCovarianceMatrixes(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrixS_m);
            SimpleOperations.NullingOfArray(KalmanVars.CovarianceMatrixS_p);


            // --- нач. ковариации для ошибки координат --- //
            KalmanVars.CovarianceMatrixS_m[0 * iMx + 0] = KalmanVars.CovarianceMatrixS_p[0 * iMx + 0] = SINSstate.stdR;    // позиционные ошибки
            KalmanVars.CovarianceMatrixS_m[1 * iMx + 1] = KalmanVars.CovarianceMatrixS_p[1 * iMx + 1] = SINSstate.stdR;

            // --- нач. ковариации для ошибки скорости --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 0)] = SINSstate.stdV;   // 0.01 м/с
            KalmanVars.CovarianceMatrixS_m[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 1)] = SINSstate.stdV;

            // --- нач. ковариации для ошибок углов ориентации --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 0)]
                = Math.Sign(SINSstate.stdAlpha1) * Math.Max(Math.Abs(SINSstate.stdAlpha1), 1E-6);  // 5 угл. минут
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 1)]
                = Math.Sign(SINSstate.stdAlpha2) * Math.Max(Math.Abs(SINSstate.stdAlpha2), 1E-6);
            KalmanVars.CovarianceMatrixS_m[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 2)] = KalmanVars.CovarianceMatrixS_p[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 2)]
                = Math.Sign(SINSstate.stdBeta3) * Math.Max(Math.Abs(SINSstate.stdBeta3), 1E-6);

            // --- нач. ковариации для дрейфов ДУС --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 0) * iMx + (iMx_Nu0 + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 0) * iMx + (iMx_Nu0 + 0)]
                = Math.Sign(SINSstate.stdNu) * Math.Max(Math.Abs(SINSstate.stdNu) * SimpleData.ToRadian / 3600.0, 1E-10); 
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 1) * iMx + (iMx_Nu0 + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 1) * iMx + (iMx_Nu0 + 1)]
                = Math.Sign(SINSstate.stdNu) * Math.Max(Math.Abs(SINSstate.stdNu) * SimpleData.ToRadian / 3600.0, 1E-10);
            KalmanVars.CovarianceMatrixS_m[(iMx_Nu0 + 2) * iMx + (iMx_Nu0 + 2)] = KalmanVars.CovarianceMatrixS_p[(iMx_Nu0 + 2) * iMx + (iMx_Nu0 + 2)]
                = Math.Sign(SINSstate.stdNu) * Math.Max(Math.Abs(SINSstate.stdNu) * SimpleData.ToRadian / 3600.0, 1E-10);

            // --- нач. ковариации для горизонтальных ньютонометров --- //
            KalmanVars.CovarianceMatrixS_m[(f0_12 + 0) * iMx + (f0_12 + 0)] = KalmanVars.CovarianceMatrixS_p[(f0_12 + 0) * iMx + (f0_12 + 0)]
                = Math.Sign(SINSstate.stdF[0]) * Math.Max(Math.Abs(SINSstate.stdF[0]), 1E-6);    // м/с^2
            KalmanVars.CovarianceMatrixS_m[(f0_12 + 1) * iMx + (f0_12 + 1)] = KalmanVars.CovarianceMatrixS_p[(f0_12 + 1) * iMx + (f0_12 + 1)]
                = Math.Sign(SINSstate.stdF[1]) * Math.Max(Math.Abs(SINSstate.stdF[1]), 1E-6);

            // --- нач. ковариации для вертикального ньютонометра, если он включен в вектор ошибок --- //
            if (SINSstate.value_iMx_f0_3 > 0)
                KalmanVars.CovarianceMatrixS_m[(f0_3 + 0) * iMx + (f0_3 + 0)] = KalmanVars.CovarianceMatrixS_p[(f0_3 + 0) * iMx + (f0_3 + 0)]
                    = Math.Sign(SINSstate.stdF[2]) * Math.Max(Math.Abs(SINSstate.stdF[2]), 1E-6);

            // --- нач. ковариации для ошибок масштаба и ошибок углов установки БИНС на корпусе --- //
            if (iMx_kappa_3_ds > 0)
            {
                KalmanVars.CovarianceMatrixS_m[(iMx_kappa_3_ds + 0) * iMx + (iMx_kappa_3_ds + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 0) * iMx + (iMx_kappa_3_ds + 0)] = SINSstate.stdKappa3 * SimpleData.ToRadian_min;
                KalmanVars.CovarianceMatrixS_m[(iMx_kappa_3_ds + 1) * iMx + (iMx_kappa_3_ds + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_kappa_3_ds + 1) * iMx + (iMx_kappa_3_ds + 1)] = SINSstate.stdScale;
            }

            // --- нач. ковариации горизонтальных ошибок координат одометрического счисления --- //
            KalmanVars.CovarianceMatrixS_m[(iMx_r12_odo + 0) * iMx + (iMx_r12_odo + 0)] = KalmanVars.CovarianceMatrixS_p[(iMx_r12_odo + 0) * iMx + (iMx_r12_odo + 0)] = SINSstate.stdOdoR;
            KalmanVars.CovarianceMatrixS_m[(iMx_r12_odo + 1) * iMx + (iMx_r12_odo + 1)] = KalmanVars.CovarianceMatrixS_p[(iMx_r12_odo + 1) * iMx + (iMx_r12_odo + 1)] = SINSstate.stdOdoR;


            SimpleOperations.PrintMatrixToFile(KalmanVars.CovarianceMatrixS_m, SimpleData.iMx, SimpleData.iMx, "StartCovariance");


            // ---------------------ВЕРТИКАЛЬНЫЙ КАНАЛ ОТДЕЛЬНО-----------------------------//
            int iMxV = SimpleData.iMx_Vertical,
                vert_f0_3 = SINSstate.Vertical_f0_3,
                Vertical_kappa1 = SINSstate.Vertical_kappa1,
                Vertical_rOdo3 = SINSstate.Vertical_rOdo3
                ;

            SimpleOperations.NullingOfArray(KalmanVars.Vertical_CovarianceMatrixS_m);
            SimpleOperations.NullingOfArray(KalmanVars.Vertical_CovarianceMatrixS_p);

            // --- нач. ковариации ошибок высоты и верт. скорости --- //
            KalmanVars.Vertical_CovarianceMatrixS_m[0 * iMxV + 0] = KalmanVars.Vertical_CovarianceMatrixS_p[0 * iMxV + 0] = SINSstate.stdR;
            KalmanVars.Vertical_CovarianceMatrixS_m[1 * iMxV + 1] = KalmanVars.Vertical_CovarianceMatrixS_p[1 * iMxV + 1] = SINSstate.stdV;

            // --- нач. ковариации ошибки верт. ньютонометра --- //
            KalmanVars.Vertical_CovarianceMatrixS_m[vert_f0_3 * iMxV + vert_f0_3] = KalmanVars.Vertical_CovarianceMatrixS_p[vert_f0_3 * iMxV + vert_f0_3]
                = Math.Sign(SINSstate.stdF[2]) * Math.Max(Math.Abs(SINSstate.stdF[2]), 1E-6);

            // --- нач. ковариации ошибок мастаба одометра и углов установки БИНС на корпусе --- //
            if (Vertical_kappa1 > 0)
                KalmanVars.Vertical_CovarianceMatrixS_m[Vertical_kappa1 * iMxV + Vertical_kappa1]
                    = KalmanVars.Vertical_CovarianceMatrixS_p[Vertical_kappa1 * iMxV + Vertical_kappa1] = SINSstate.stdKappa1 * SimpleData.ToRadian_min;

            KalmanVars.Vertical_CovarianceMatrixS_m[Vertical_rOdo3 * iMxV + Vertical_rOdo3]
                = KalmanVars.Vertical_CovarianceMatrixS_p[Vertical_rOdo3 * iMxV + Vertical_rOdo3] = SINSstate.stdOdoR;

        }






        public static void MatrixNoise_ReDef(SINS_State SINSstate, Kalman_Vars KalmanVars)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;
            int iMx_dV_12 = SINSstate.value_iMx_dV_12, iMx_alphaBeta = SINSstate.value_iMx_alphaBeta, iMx_Nu0 = SINSstate.value_iMx_Nu0;

            double sqrt_freq_vert = Math.Sqrt(Math.Abs(SINSstate.Freq));
            double sqrt_freq = Math.Sqrt(Math.Abs(SINSstate.Freq));

            sqrt_freq = 1.0;
            sqrt_freq_vert = 1.0;

            double[] Noise_Vel_in_Mx = new double[3], Noise_Angl_in_Mx = new double[3];

            for (int i = 0; i < iMx * iMq; i++)
                KalmanVars.CovarianceMatrixNoise[i] = 0.0;

            // --- На основе шумовых параметров, полученных на выставке в приборной системе координат, формируем шумовые параметры в проекции на географическую СК
            for (int j = 0; j < 3; j++)
            {
                Noise_Vel_in_Mx[j] = Math.Sqrt(Math.Pow(SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Vel[0], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Vel[1], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Vel[2], 2));

                Noise_Angl_in_Mx[j] = Math.Sqrt(Math.Pow(SINSstate.A_x0s[j, 0] * KalmanVars.Noise_Angl[0], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 1] * KalmanVars.Noise_Angl[1], 2) +
                                               Math.Pow(SINSstate.A_x0s[j, 2] * KalmanVars.Noise_Angl[2], 2));
            }

            // --- шумы по горизонтальному каналу БИНС
            KalmanVars.CovarianceMatrixNoise[0 * iMq + 0] = KalmanVars.Noise_Pos * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[1 * iMq + 1] = KalmanVars.Noise_Pos * sqrt_freq;

            // --- Проставляются параметры шумов датчиков в матриц Q //
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 0) * iMq + iMx_dV_12 + 0] = Noise_Vel_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 0) * iMq + iMx_alphaBeta + 0] = SINSstate.Vx_0[1] * Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 1) * iMq + iMx_dV_12 + 1] = Noise_Vel_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_dV_12 + 1) * iMq + iMx_alphaBeta + 1] = SINSstate.Vx_0[0] * Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 0) * iMq + iMx_alphaBeta + 0] = Noise_Angl_in_Mx[0] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 1) * iMq + iMx_alphaBeta + 1] = Noise_Angl_in_Mx[1] * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_alphaBeta + 2) * iMq + iMx_alphaBeta + 2] = Noise_Angl_in_Mx[2] * sqrt_freq;

            KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 0) * iMq + iMx_Nu0 + 0] = 0.0005 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 1) * iMq + iMx_Nu0 + 1] = 0.0005 * SimpleData.ToRadian / 3600.0;
            KalmanVars.CovarianceMatrixNoise[(iMx_Nu0 + 2) * iMq + iMx_Nu0 + 2] = 0.0005 * SimpleData.ToRadian / 3600.0;

            // --- шумы по горизонтальному одометрическому решению
            KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 0) * iMq + iMx_r12_odo + 0] = KalmanVars.Noise_Pos * sqrt_freq;
            KalmanVars.CovarianceMatrixNoise[(iMx_r12_odo + 1) * iMq + iMx_r12_odo + 1] = KalmanVars.Noise_Pos * sqrt_freq;



            // ----------------------------------------------------------//
            // --- Матрица шумов для вертикального канала ---
            SimpleOperations.NullingOfArray(KalmanVars.Vertical_CovarianceMatrixNoise);

            KalmanVars.Vertical_CovarianceMatrixNoise[0 * SimpleData.iMq_Vertical + 0] = KalmanVars.Noise_Pos_Vertical * sqrt_freq_vert;
            KalmanVars.Vertical_CovarianceMatrixNoise[1 * SimpleData.iMq_Vertical + 1] = Noise_Vel_in_Mx[2] * sqrt_freq_vert;
            KalmanVars.Vertical_CovarianceMatrixNoise[SINSstate.Vertical_rOdo3 * SimpleData.iMq_Vertical + SINSstate.Vertical_rOdo3] = KalmanVars.Noise_Pos_Vertical * sqrt_freq_vert;

        }





        public static void Make_A(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds,iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            for (int i = 0; i < iMx * iMx; i++)
                KalmanVars.Matrix_A[i] = 0;

            SINSstate.W_x[0] = SINSstate.Omega_x[0];
            SINSstate.W_x[1] = SINSstate.Omega_x[1] + SimpleData.U * Math.Cos(SINSstate.Latitude);
            SINSstate.W_x[2] = SINSstate.Omega_x[2] + SimpleData.U * Math.Sin(SINSstate.Latitude);

            /*----------- Далее компоненты для матрицы части БИНС в горизонтальном канале ----------------*/

            // --- блок по позиционным ошибкам БИНС
            KalmanVars.Matrix_A[0 * iMx + 1] = SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[0 * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_A[0 * iMx + (iMx_alphaBeta + 2)] = SINSstate.Vx_0[1];

            KalmanVars.Matrix_A[1 * iMx + 0] = -SINSstate.Omega_x[2];
            KalmanVars.Matrix_A[1 * iMx + (iMx_dV_12 + 1)] = 1.0;
            KalmanVars.Matrix_A[1 * iMx + (iMx_alphaBeta + 2)] = -SINSstate.Vx_0[0];

            // --- блок по скоростным ошибкам
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + 1] = SINSstate.u_x[1] * SINSstate.Vx_0[1] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_dV_12 + 1)] = SINSstate.Omega_x[2] + 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 0)] = SINSstate.u_x[1] * SINSstate.Vx_0[1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 1)] = -SINSstate.g;
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 0)] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 1)] = -SINSstate.Vx_0[1] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_12 + 0)] = SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_12 + 1)] = SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (f0_3 + 0)] = SINSstate.A_x0s[0, 2];


            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + 1] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_dV_12 + 0)] = -SINSstate.Omega_x[2] - 2 * SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 0)] = -SINSstate.u_x[1] * SINSstate.Vx_0[0] + SINSstate.g;
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 0)] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 1)] = SINSstate.Vx_0[0] * SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_12 + 0)] = SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_12 + 1)] = SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (f0_3 + 0)] = SINSstate.A_x0s[1, 2];


            // --- блок по угловым ошибкам ориентации
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + 0] = -SINSstate.u_x[2] / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_dV_12 + 1)] = -1.0 / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 1)] = SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_alphaBeta + 2)] = -SINSstate.u_x[1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[0, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[0, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 0) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[0, 2];

            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + 1] = -SINSstate.u_x[2] / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_dV_12 + 0)] = 1.0 / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_alphaBeta + 0)] = -SINSstate.u_x[2];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[1, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[1, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 1) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[1, 2];

            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + 0] = SINSstate.Omega_x[0] / SINSstate.R_e;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + 1] = (SINSstate.Omega_x[1] + SINSstate.u_x[1]) / SINSstate.R_n;
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 0)] = SINSstate.Omega_x[1] + SINSstate.u_x[1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_alphaBeta + 1)] = -SINSstate.Omega_x[0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 0)] = -SINSstate.A_x0s[2, 0];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 1)] = -SINSstate.A_x0s[2, 1];
            KalmanVars.Matrix_A[(iMx_alphaBeta + 2) * iMx + (iMx_Nu0 + 2)] = -SINSstate.A_x0s[2, 2];



            /*-----------Компоненты при ошибках одометрического счисления----------------*/

            // --- блок по горизонтальным ошибкам одометрического счисления
            KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + (iMx_alphaBeta + 2)] = SINSstate_OdoMod.Vx_0[1];
            KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + iMx_r12_odo + 1] = SINSstate_OdoMod.Omega_x[2];
            if (iMx_kappa_3_ds > 0)
            {
                KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + iMx_kappa_3_ds + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[0, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[0, 1];
                KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + iMx_kappa_3_ds + 1] = SINSstate_OdoMod.OdoSpeed_x0[0];
            }

            KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 2)] = -SINSstate_OdoMod.Vx_0[0];
            KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_r12_odo + 0] = -SINSstate_OdoMod.Omega_x[2];
            if (iMx_kappa_3_ds > 0)
            {
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_kappa_3_ds + 0] = -SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[1, 0] + SINSstate_OdoMod.OdoSpeed_s[0] * SINSstate_OdoMod.A_x0s[1, 1];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + iMx_kappa_3_ds + 1] = SINSstate_OdoMod.OdoSpeed_x0[1];
            }




            // ----------------------------------------------------------//
            // ----------------ВЕРТИКАЛЬНЫЙ КАНАЛ ОТДЕЛЬНО----------------------//
            // ----------------------------------------------------------//
            int iMxV = SimpleData.iMx_Vertical, Vertical_rOdo3 = SINSstate.Vertical_rOdo3, Vertical_kappa1 = SINSstate.Vertical_kappa1;

            SimpleOperations.NullingOfArray(KalmanVars.Vertical_Matrix_A);

            KalmanVars.Vertical_Matrix_A[0 * iMxV + 1] = 1.0;
            KalmanVars.Vertical_Matrix_A[1 * iMxV + 0] = 2 * 0.000001538;

            KalmanVars.Vertical_Matrix_A[1 * iMxV + SINSstate.Vertical_f0_3] = SINSstate.A_x0s[2, 2];

            if (Vertical_kappa1 > 0)
                KalmanVars.Vertical_Matrix_A[Vertical_rOdo3 * iMxV + Vertical_kappa1] 
                    = SINSstate_OdoMod.OdoSpeed_s[1] * SINSstate_OdoMod.A_x0s[2, 2] - SINSstate_OdoMod.OdoSpeed_s[2] * SINSstate_OdoMod.A_x0s[2, 1];


            // ----------------------------------------------------------//
            // --- Дополняем горизонтальный канал компонентами, при которых стоят либо высота, либо вертикальная скорость --- //
            if (true)
            {
                KalmanVars.Matrix_A[0 * iMx + 0] += SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[0 * iMx + (iMx_alphaBeta + 1)] += -SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[1 * iMx + 1] += SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[1 * iMx + (iMx_alphaBeta + 0)] += SINSstate.Vx_0[2];

                KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + 1] += SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_n;
                KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_alphaBeta + 0)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 0)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 0];
                KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 1)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 1];
                KalmanVars.Matrix_A[(iMx_dV_12 + 0) * iMx + (iMx_Nu0 + 2)] += SINSstate.Vx_0[2] * SINSstate.A_x0s[1, 2];

                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + 0] += -SINSstate.u_x[2] * SINSstate.Vx_0[2] / SINSstate.R_e;
                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 1)] += SINSstate.u_x[2] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_alphaBeta + 2)] += -SINSstate.u_x[1] * SINSstate.Vx_0[2];
                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 0)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 0];
                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 1)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 1];
                KalmanVars.Matrix_A[(iMx_dV_12 + 1) * iMx + (iMx_Nu0 + 2)] += -SINSstate.Vx_0[2] * SINSstate.A_x0s[0, 2];

                KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + 0] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_e;
                KalmanVars.Matrix_A[(iMx_r12_odo + 0) * iMx + (iMx_alphaBeta + 1)] += -SINSstate_OdoMod.Vx_0[2];
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + 1] += SINSstate_OdoMod.Vx_0[2] / SINSstate_OdoMod.R_n;
                KalmanVars.Matrix_A[(iMx_r12_odo + 1) * iMx + (iMx_alphaBeta + 0)] += SINSstate_OdoMod.Vx_0[2];
            }
        }





        public static void StateIntegration_AT(SINS_State SINSstate, Kalman_Vars KalmanVars, SINS_State SINSstate_OdoMod)
        {
            double[] fz = new double[3], Wz = new double[3], u = new double[3], tempV = new double[3], Wz_avg = new double[3];
            double[] Vx_0 = new double[3], Vx_0_prev = new double[3];

            Matrix AT_z_xi = new Matrix(3, 3); Matrix B_x_eta = new Matrix(3, 3);
            Matrix dAT = new Matrix(3, 3); Matrix D_x_z = new Matrix(3, 3);
            Matrix W_x_xi = new Matrix(3, 3); Matrix C_eta_xi = new Matrix(3, 3);

            Matrix Hat1 = new Matrix(3, 3);
            Matrix Hat2 = new Matrix(3, 3);
            Matrix E = Matrix.UnitMatrix(3);
            Matrix dMatrix = new Matrix(3, 3);

            double W_z_abs, Omega_x_abs, dlt, dlt2, Altitude, Altitude_prev, dh, dVx, dVy, dVh, Azimth;

            CopyMatrix(AT_z_xi, SINSstate.AT);
            CopyMatrix(B_x_eta, SINSstate.A_x0n);

            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            Altitude = SINSstate.Height;
            Altitude_prev = SINSstate.Altitude_prev;

            fz[1] = SINSstate.F_z[1];
            fz[2] = SINSstate.F_z[2];
            fz[0] = SINSstate.F_z[0];
            Wz[1] = SINSstate.W_z[1];
            Wz[2] = SINSstate.W_z[2];
            Wz[0] = SINSstate.W_z[0];


            // --- в обратной связи используется накопленные оценки ошибок ньютонометров в горизонте
            fz[0] -= SINSstate.Cumulative_KalmanErrorVector[(SINSstate.value_iMx_f0_12 + 0)];
            fz[1] -= SINSstate.Cumulative_KalmanErrorVector[(SINSstate.value_iMx_f0_12 + 1)];
            // --- в обратной связи используется накопленные оценка ошибки ньютонометра f_3 в вертикальном канале
            fz[2] -= SINSstate.Vertical_Cumulative_KalmanErrorVector[SINSstate.Vertical_f0_3];

            // --- то же самое для ДУС, только все с горизонтального канала
            for (int i = 0; i < 3; i++)
                Wz[i] += SINSstate.Cumulative_KalmanErrorVector[SINSstate.value_iMx_Nu0 + i];

            // --- вычитание из показаний датчиков значений нулей, полученных алгебраической выставкой на начальной выставке
            for (int i = 0; i < 3; i++)
                fz[i] = fz[i] - SINSstate.AlignAlgebraZeroF[i];
            for (int i = 0; i < 3; i++)
                Wz[i] = Wz[i] + SINSstate.AlignAlgebraDrifts[i];
            


            CopyArray(SINSstate.F_z, fz);
            CopyArray(SINSstate.W_z, Wz);
            CopyArray(Vx_0, SINSstate.Vx_0);
            CopyArray(Vx_0_prev, SINSstate.Vx_0_prev);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);

            SINSstate.u_x = U_x0(SINSstate.Latitude);

            u[0] = 0.0;
            u[1] = SimpleData.U * Math.Cos(SINSstate.Latitude);
            u[2] = SimpleData.U * Math.Sin(SINSstate.Latitude);


            //-------------ИНТЕГРИРОВАНИЕ МАТРИЦЫ AT_Z_XI И ПЕРВОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z---------
            for (int i = 0; i < 3; i++)
            {
                fz[i] = (fz[i] + SINSstate.F_z_prev[i]) / 2.0;
                Wz[i] = (Wz[i] + SINSstate.W_z_prev[i]) / 2.0;
            }


            W_z_abs = Math.Sqrt(Wz[0] * Wz[0] + Wz[1] * Wz[1] + Wz[2] * Wz[2]);
            dlt = Math.Sin(W_z_abs * SINSstate.timeStep) / W_z_abs;
            dlt2 = (1.0 - Math.Cos(W_z_abs * SINSstate.timeStep)) / (W_z_abs * W_z_abs);

            Hat1 = Matrix.SkewSymmetricMatrix(Wz);
            Hat2 = Matrix.SkewSymmetricMatrixSquare(Wz);

            // --- Интегрирование матрицы ориентации AT_z_xi приборного относительно инерциального трехгранника
            CopyMatrix(dMatrix, (E + Hat1 * dlt + Hat2 * dlt2));
            CopyMatrix(AT_z_xi, (dMatrix * AT_z_xi));

            //Нормировка
            for (int i = 0; i < 3; i++)
            {
                tempV[i] = Math.Sqrt(AT_z_xi[i, 0] * AT_z_xi[i, 0] + AT_z_xi[i, 1] * AT_z_xi[i, 1] + AT_z_xi[i, 2] * AT_z_xi[i, 2]);
                for (int j = 0; j < 3; j++)
                    AT_z_xi[i, j] = AT_z_xi[i, j] / tempV[i];
            }


            CopyMatrix(SINSstate.AT, AT_z_xi);

            // --- Вычисление матрицы ориентации D_x_z приборного относительно географии
            CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());
            //--------------------------------------------------------------------------------------



            //---------------------------------ИНТЕГРИРОВАНИЕ СКОРОСТЕЙ----------------------------
            CopyArray(SINSstate.F_x, D_x_z * fz);
            SINSstate.g = SimpleData.Gravity_Normal * (1.0 + 0.0053020 * Math.Pow(Math.Sin(SINSstate.Latitude), 2) - 0.000007 * Math.Pow(Math.Sin(2 * SINSstate.Latitude), 2)) - 0.00014 - 2 * 0.000001538 * Altitude;

            dVx = SINSstate.F_x[0] + Vx_0[1] * (2.0 * u[2] + SINSstate.Omega_x[2]) - Vx_0[2] * (2.0 * u[1] + SINSstate.Omega_x[1]);
            dVy = SINSstate.F_x[1] - Vx_0[0] * (2.0 * u[2] + SINSstate.Omega_x[2]) + Vx_0[2] * (2.0 * u[0] + SINSstate.Omega_x[0]);

            Vx_0[0] += dVx * SINSstate.timeStep;
            Vx_0[1] += dVy * SINSstate.timeStep;

            if (SimpleOperations.AbsoluteVectorValue(Vx_0) > 0.1)
            {
                double[] Vs = new double[3], Vx0 = new double[3];
                Vx0[0] = Vx_0[0];
                Vx0[1] = Vx_0[1];
                SimpleOperations.CopyArray(Vs, SINSstate.A_sx0 * Vx0);
                SINSstate.distance_by_SINS += Vs[1] * SINSstate.timeStep;
            }

            //--------------------------------------------------------------------------------------


            //--- Интегрируем вертикальную скорость ---//
            if (SINSstate.flag_AutonomouseSolution == false)
            {
                dVh = SINSstate.F_x[2] - SINSstate.g + (Vx_0[0] + Vx_0_prev[0]) / 2.0 * (2 * u[1] + SINSstate.Omega_x[1]) - (Vx_0[1] + Vx_0_prev[1]) / 2.0 * (2 * u[0] + SINSstate.Omega_x[0]);
                Vx_0[2] += dVh * SINSstate.timeStep;

                dh = (Vx_0[2] + Vx_0_prev[2]) / 2.0;
                Altitude += dh * SINSstate.timeStep;
            }




            //---------ИНТЕГРИРОВАНИЕ МАТРИЦЫ B_X_ETA И ВТОРОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z--------------
            SINSstate.Omega_x[0] = -(Vx_0[1] + Vx_0_prev[1]) / 2.0 / SINSstate.R_n;
            SINSstate.Omega_x[1] = (Vx_0[0] + Vx_0_prev[0]) / 2.0 / SINSstate.R_e;
            SINSstate.Omega_x[2] = Math.Tan(SINSstate.Latitude) * SINSstate.Omega_x[1];

            Omega_x_abs = Math.Sqrt(SINSstate.Omega_x[0] * SINSstate.Omega_x[0] + SINSstate.Omega_x[1] * SINSstate.Omega_x[1] + SINSstate.Omega_x[2] * SINSstate.Omega_x[2]);
            if (Omega_x_abs != 0)
            {
                dlt = Math.Sin(Omega_x_abs * SINSstate.timeStep) / Omega_x_abs;
                dlt2 = (1.0 - Math.Cos(Omega_x_abs * SINSstate.timeStep)) / (Omega_x_abs * Omega_x_abs);
            }
            else
            {
                dlt = 1.0;
                dlt2 = 0.0;
            }

            Hat1 = Matrix.SkewSymmetricMatrix(SINSstate.Omega_x);
            Hat2 = Matrix.SkewSymmetricMatrixSquare(SINSstate.Omega_x);


            CopyMatrix(dMatrix, E + Hat1 * dlt + Hat2 * dlt2);
            CopyMatrix(B_x_eta, dMatrix * B_x_eta);

            //Нормировка
            for (int i = 0; i < 3; i++)
            {
                tempV[i] = Math.Sqrt(B_x_eta[i, 0] * B_x_eta[i, 0] + B_x_eta[i, 1] * B_x_eta[i, 1] + B_x_eta[i, 2] * B_x_eta[i, 2]);
                for (int j = 0; j < 3; j++)
                    B_x_eta[i, j] = B_x_eta[i, j] / tempV[i];
            }

            // --- Повторное вычисление матрицы ориентации D_x_z приборного относительно географии
            CopyMatrix(W_x_xi, B_x_eta * SINSstate.A_nxi);
            CopyMatrix(D_x_z, W_x_xi * SINSstate.AT.Transpose());

            //----------------Вычисление углов и переприсвоение матриц---------------------------
            CopyMatrix(SINSstate.A_sx0, D_x_z.Transpose());
            CopyMatrix(SINSstate.A_x0s, D_x_z);
            CopyMatrix(SINSstate.A_x0n, B_x_eta);
            CopyMatrix(SINSstate.A_nx0, B_x_eta.Transpose());



            //---ОПРЕДЕЛЕНИЕ ГЕОГРАФИЧЕСКИХ КООРДИНАТ---
            SINSstate.Longitude = Math.Atan2(SINSstate.A_x0n[2, 1], SINSstate.A_x0n[2, 0]);
            SINSstate.Latitude = Math.Atan2(SINSstate.A_x0n[2, 2], Math.Sqrt(SINSstate.A_x0n[0, 2] * SINSstate.A_x0n[0, 2] + SINSstate.A_x0n[1, 2] * SINSstate.A_x0n[1, 2]));
            Azimth = Math.Atan2(SINSstate.A_x0n[0, 2], SINSstate.A_x0n[1, 2]);

            SINSstate.Altitude_prev = SINSstate.Height;
            SINSstate.Height = Altitude;


            //SINSstate.Heading = gkurs - Azimth;
            SINSstate.Heading = Math.Atan2(SINSstate.A_sx0[1, 0], SINSstate.A_sx0[1, 1]);
            SINSstate.Roll = -Math.Atan2(SINSstate.A_sx0[0, 2], SINSstate.A_sx0[2, 2]);
            SINSstate.Pitch = Math.Atan2(SINSstate.A_sx0[1, 2], Math.Sqrt(SINSstate.A_sx0[0, 2] * SINSstate.A_sx0[0, 2] + SINSstate.A_sx0[2, 2] * SINSstate.A_sx0[2, 2]));
            //SINSstate.Azimth = Azimth;

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
            SINSstate.u_x = U_x0(SINSstate.Latitude);

            CopyArray(SINSstate.Vx_0_prev, SINSstate.Vx_0);
            CopyArray(SINSstate.Vx_0, Vx_0);

            CopyArray(SINSstate.F_z_prev, SINSstate.F_z);
            CopyArray(SINSstate.W_z_prev, SINSstate.W_z);
            CopyArray(SINSstate.W_x, SINSstate.A_x0s * Wz);


            //--------------------------------------------------------------------------------------
            //--------------------------------------------------------------------------------------
            //--------- ДЛЯ SINSstate_OdoModel - одометрического счисления ---------//

            SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0_prev, SINSstate_OdoMod.Vx_0);
            SimpleOperations.CopyArray(SINSstate.OdoSpeed_x0, SINSstate.A_x0s * SINSstate.OdoSpeed_s);
            SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_s, SINSstate.OdoSpeed_s);

            //--- считаем для одометрического счисления свою матрицу ориентации SINSstate_OdoMod.A_x0s ---//
            SimpleOperations.CopyMatrix(W_x_xi, SINSstate_OdoMod.A_x0n * SINSstate.A_nxi);
            SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, W_x_xi * SINSstate.AT.Transpose());

            SimpleOperations.CopyArray(SINSstate_OdoMod.OdoSpeed_x0, SINSstate_OdoMod.A_x0s * SINSstate.OdoSpeed_s);
            

            //---------ВЫЧИСЛЕНИЕ МАТРИЦЫ B_X_ETA И ВТОРОЕ ВЫЧИСЛЕНИЕ МАТРИЦЫ D_X_Z для одометрического счисления--------------
            {
                SimpleOperations.CopyArray(SINSstate_OdoMod.Vx_0, SINSstate_OdoMod.OdoSpeed_x0);

                SINSstate_OdoMod.Omega_x[0] = -(SINSstate_OdoMod.Vx_0[1] + SINSstate_OdoMod.Vx_0_prev[1]) / 2.0 / SINSstate_OdoMod.R_n;
                SINSstate_OdoMod.Omega_x[1] = (SINSstate_OdoMod.Vx_0[0] + SINSstate_OdoMod.Vx_0_prev[0]) / 2.0 / SINSstate_OdoMod.R_e;
                SINSstate_OdoMod.Omega_x[2] = Math.Tan(SINSstate_OdoMod.Latitude) * SINSstate_OdoMod.Omega_x[1];

                //--- Производим одометрическое счисление координат, если пришло обновление показаний одометра ---//
                if (SINSstate.OdometerData.odometer_left.isReady == 1)
                {
                    double[] dS_x = new double[3];
                    SimpleOperations.CopyArray(dS_x, SINSstate_OdoMod.A_x0s * SINSstate.OdometerVector);

                    SINSstate_OdoMod.Latitude = SINSstate_OdoMod.Latitude + dS_x[1] / SimpleOperations.RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
                    SINSstate_OdoMod.Longitude = SINSstate_OdoMod.Longitude + dS_x[0] / SimpleOperations.RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height) / Math.Cos(SINSstate_OdoMod.Latitude);
                    SINSstate_OdoMod.Height = SINSstate_OdoMod.Height + dS_x[2];
                }

                //----------------Вычисление углов и переприсвоение матриц---------------------------
                SINSstate_OdoMod.A_x0n = SimpleOperations.A_x0n(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Longitude);
                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_nx0, SINSstate_OdoMod.A_x0n.Transpose());

                SimpleOperations.CopyMatrix(W_x_xi, SINSstate_OdoMod.A_x0n * SINSstate.A_nxi);
                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_x0s, W_x_xi * SINSstate.AT.Transpose());
                

                SimpleOperations.CopyMatrix(SINSstate_OdoMod.A_sx0, SINSstate_OdoMod.A_x0s.Transpose());
            }

            SINSstate_OdoMod.R_e = RadiusE(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
            SINSstate_OdoMod.R_n = RadiusN(SINSstate_OdoMod.Latitude, SINSstate_OdoMod.Height);
            //--------------------------------------------------------------------------------------
        }

    }
}
