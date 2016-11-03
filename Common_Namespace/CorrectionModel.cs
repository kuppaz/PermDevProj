using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class CorrectionModel : SimpleOperations
    {
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------ПОЗИЦИОННАЯ КОРЕКЦИЯ-----------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        public static void Make_H_POSITION(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Proc_Help ProcHelp)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;


            double[] tempVect = new double[3];

            // --- шум измерения. Если объект стоит - уменьшаем
            double Noize = 1.0;
            double longOdoIncrement = SINSstate.OdometerData.odometer_left.Value - SINSstate.OdometerLeft_ArrayOfPrev[Math.Min(20, SINSstate.OdometerLeft_ArrayOfPrev.Length)];
            double longOdoIncrement_dt = SINSstate.Time + SINSstate.Time_Alignment - SINSstate.OdometerLeft_ArrayOfPrevTime[Math.Min(20, SINSstate.OdometerLeft_ArrayOfPrev.Length)];

            if (longOdoIncrement / longOdoIncrement_dt == 0.0)
                Noize = 0.01;


            //---Разбиение на три составляющие---
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = -1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = -1.0;

            // --- Формирование измерений по разности координат БИНС и одометрического счисления
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - SINSstate_OdoMod.Longitude) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - SINSstate_OdoMod.Latitude) * SINSstate.R_n;

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = Noize;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = Noize;

            KalmanVars.cnt_measures += 2;



            // ----------------------------------------------------------//
            // --------Измерение для коррекции вертикального канала-------------//
            // ----------------------------------------------------------//
            {
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = -1.0;

                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Height - SINSstate_OdoMod.Height;

                // --- шум измерения
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = Noize * SINSstate.OdoVerticalNoiseMultiplicator;

                KalmanVars.Vertical_cnt_measures += 1;
            }

            KalmanVars.counter_odoPosCorrection++;
        }



        //--------------------------------------------------------------------------
        //--------------------------ПОЗИЦИОННАЯ КОРЕКЦИЯ CHECKPOINTS-----------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_CONTROLPOINTS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, double Latitude_CP, double Longitude_CP, double Altitude_CP, double NoiseValue)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            double[] tempVect = new double[3];

            if (Latitude_CP != 0 && Longitude_CP != 0)
            {
                // -- Блок формирования измерений по разнице координат БИНС и внешней информации
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + 0] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + 1] = 1.0;
                //Формирование измерений по географическим координатам
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate.Longitude - Longitude_CP) * SINSstate.R_e * Math.Cos(SINSstate.Latitude);
                KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate.Latitude - Latitude_CP) * SINSstate.R_n;

                // --- шум измерения
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = NoiseValue;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = NoiseValue;

                KalmanVars.cnt_measures += 2;


                // -- Блок формирования измерений по разнице координат одометрического решения и внешней информации
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_r12_odo + 0] = 1.0;
                KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_r12_odo + 1] = 1.0;
                //Формирование измерений по географическим координатам
                KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = (SINSstate_OdoMod.Longitude - Longitude_CP) * SINSstate_OdoMod.R_e * Math.Cos(SINSstate_OdoMod.Latitude);
                KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = (SINSstate_OdoMod.Latitude - Latitude_CP) * SINSstate_OdoMod.R_n;

                // --- шум измерения
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = NoiseValue;
                KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = NoiseValue;

                KalmanVars.cnt_measures += 2;
            }


            // ----------------------------------------------------------//
            // --------Измерение для коррекции вертикального канала-------------//
            // ----------Если на входе ненулевая высота, то измерение формируется----------//
            if (Altitude_CP != 0.0)
            {
                double VerticalNoise = NoiseValue;

                //=== Когда идет "принудительная" коррекция по высоте по стартовой точке, меняем шум измерения ===//
                if (Latitude_CP == 0 && Longitude_CP == 0)
                    VerticalNoise = 0.01;

                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 0] = 1.0;
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 1) * SimpleData.iMx_Vertical + SINSstate.Vertical_rOdo3] = 1.0;

                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Height - Altitude_CP;
                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 1)] = SINSstate_OdoMod.Height - Altitude_CP;

                // --- шум измерения
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = VerticalNoise;
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 1)] = VerticalNoise;

                KalmanVars.Vertical_cnt_measures += 2;
            }


            if (Latitude_CP != 0 || Longitude_CP != 0 || Altitude_CP != 0.0)
                SINSstate.flag_UsingCorrection = true;
        }




        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------Коррекция дрейфа на стоянке-----------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        public static void Make_H_DRIFTS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod, Proc_Help ProcHelp)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;


            double[] tempVect = new double[3];

            // --- шум измерения. Если объект стоит - уменьшаем
            double Noize = 0.00001 * SimpleData.ToRadian;


            //---Разбиение на три составляющие---
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + iMx_Nu0 + 0] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + iMx_Nu0 + 1] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 2) * iMx + iMx_Nu0 + 2] = 1.0;

            SimpleOperations.CopyArray(SINSstate.u_s, SINSstate.A_sx0 * SINSstate.u_x);

            double[] W_z_avg = new double[3];
            W_z_avg[0] = SINSstate.forDriftMeasureWsAvg[0] / SINSstate.forDriftMeasureWsAvg_cnt;
            W_z_avg[1] = SINSstate.forDriftMeasureWsAvg[1] / SINSstate.forDriftMeasureWsAvg_cnt;
            W_z_avg[2] = SINSstate.forDriftMeasureWsAvg[2] / SINSstate.forDriftMeasureWsAvg_cnt;

            // --- Формирование измерений по разности координат БИНС и одометрического счисления
            KalmanVars.Measure[(KalmanVars.cnt_measures + 0)] = W_z_avg[0] - SINSstate.u_s[0];
            KalmanVars.Measure[(KalmanVars.cnt_measures + 1)] = W_z_avg[1] - SINSstate.u_s[1];
            KalmanVars.Measure[(KalmanVars.cnt_measures + 2)] = W_z_avg[2] - SINSstate.u_s[2];

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = Noize;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = Noize;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 2)] = Noize;

            KalmanVars.cnt_measures += 3;

            SINSstate.flag_UsingCorrection = true;
        }


        //--------------------------------------------------------------------------
        //-------------------------------КНС---------------------------------------
        //--------------------------------------------------------------------------
        public static void Make_H_KNS(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            int iMx = SimpleData.iMx, iMz = SimpleData.iMz, iMq = SimpleData.iMq, iMx_kappa_3_ds = SINSstate.value_iMx_kappa_3_ds, iMx_r12_odo = SINSstate.value_iMx_r_odo_12;

            int iMx_dV_12 = SINSstate.value_iMx_dV_12,
                iMx_alphaBeta = SINSstate.value_iMx_alphaBeta,
                iMx_Nu0 = SINSstate.value_iMx_Nu0,
                f0_12 = SINSstate.value_iMx_f0_12,
                f0_3 = SINSstate.value_iMx_f0_3
                ;

            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 0) * iMx + (iMx_dV_12 + 0)] = 1.0;
            KalmanVars.Matrix_H[(KalmanVars.cnt_measures + 1) * iMx + (iMx_dV_12 + 1)] = 1.0;

            for (int i = 0; i < 3; i++)
                KalmanVars.Measure[(KalmanVars.cnt_measures + i)] = SINSstate.Vx_0[i];

            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;
            KalmanVars.Noize_Z[(KalmanVars.cnt_measures + 1)] = KalmanVars.OdoNoise_STOP;

            KalmanVars.cnt_measures += 2;

            // ----------------------------------------------------------//
            // --------Измерение для коррекции вертикального канала-------------//
            // ----------------------------------------------------------//
            {
                KalmanVars.Vertical_Matrix_H[(KalmanVars.Vertical_cnt_measures + 0) * SimpleData.iMx_Vertical + 1] = 1.0;
                KalmanVars.Vertical_Measure[(KalmanVars.Vertical_cnt_measures + 0)] = SINSstate.Vx_0[2];
                KalmanVars.Vertical_Noize_Z[(KalmanVars.Vertical_cnt_measures + 0)] = KalmanVars.OdoNoise_STOP;

                KalmanVars.Vertical_cnt_measures += 1;
            }

            SINSstate.flag_UsingCorrection = true;
        }


    }
}
