using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Common_Namespace
{
    public class StartParameters : SimpleOperations
    {
        public static void ApplyMatrixStartCondition(SINS_State SINSstate)
        {
            SINSstate.A_sx0 = SimpleOperations.A_sx0(SINSstate);
            SINSstate.A_x0s = SINSstate.A_sx0.Transpose();
            SINSstate.A_x0n = SimpleOperations.A_x0n(SINSstate.Latitude, SINSstate.Longitude);
            SINSstate.A_nx0 = SINSstate.A_x0n.Transpose();
            SINSstate.A_nxi = SimpleOperations.A_ne(SINSstate.Time, SINSstate.Longitude_Start);
            SINSstate.AT = Matrix.Multiply(SINSstate.A_sx0, SINSstate.A_x0n);
            SINSstate.AT = Matrix.Multiply(SINSstate.AT, SINSstate.A_nxi);

            SINSstate.R_e = RadiusE(SINSstate.Latitude, SINSstate.Height);
            SINSstate.R_n = RadiusN(SINSstate.Latitude, SINSstate.Height);
        }


        public static void StartSINS_Parameters(SINS_State SINSstate, SINS_State SINSstate_OdoMod, Kalman_Vars KalmanVars, ParamToStart ParamStart, Proc_Help ProcHelp)
        {
            StreamReader configFile = new StreamReader(SimpleData.ConfigurationFileIn);

            string frequency = "", StartLatitude = "", StartLongitude = "", StartHeight = "", AlignmentDuration = "", StartCov_Kappa1 = "", StartCov_Kappa3 = ""
                , StartHeightCorrection_value = "0", StartHeightCorrection_flag = "0"
                , alpha_kappa_1 = "0"
                , alpha_kappa_3 = "0"
                , ScaleError = "0"
                , Alignment_HeadingDetermined = "0", Alignment_HeadingValue = ""
                , Alignment_RollDetermined = "0", Alignment_RollValue = ""
                , Alignment_PitchDetermined = "0", Alignment_PitchValue = ""
                , SINS_is_accurateMounted_by_kappa_1 = "0"
                , SINS_is_accurateMounted_by_kappa_3 = "0"
                , SINS_is_accurateMounted_by_scaleError = "0"
                , initError_kappa_1 = "0", initError_kappa_3 = "0", initError_ScaleError = "0"
                , CalibrationStartMode = "0"
                ;

            string AlignmentEngineIsOff = "1";
            string NoiseParamDetermin_mode = "0", NoiseParamDetermin_startTime = "0", NoiseParamDetermin_endTime = "0", NoiseParamDetermin_SigmaValueF = "0", NoiseParamDetermin_SigmaValueNu = "0";


            // --- Чтение параметров из Конфигурационного файла --- //
            for (int i = 0; ; i++)
            {
                if (configFile.EndOfStream == true) break;
                string configLine = configFile.ReadLine();

                if (configLine.Substring(0, Math.Min(2, configLine.Length)) != "//" && configLine.IndexOf('=') != -1)
                {
                    string[] tmpstr = configLine.Split('=');

                    if (tmpstr[0].Trim() == "filename")
                        SINSstate.DataInFileName = tmpstr[1];

                    if (tmpstr[0].Trim() == "frequency")
                        frequency = tmpstr[1];

                    if (tmpstr[0].Trim() == "StartLatitude")
                        StartLatitude = tmpstr[1];

                    if (tmpstr[0].Trim() == "StartLongitude")
                        StartLongitude = tmpstr[1];

                    if (tmpstr[0].Trim() == "StartHeight")
                        StartHeight = tmpstr[1];

                    if (tmpstr[0].Trim() == "AlignmentDuration")
                        AlignmentDuration = tmpstr[1];

                    if (tmpstr[0].Trim() == "AlignmentEngineIsOff")
                        AlignmentEngineIsOff = tmpstr[1];

                    if (tmpstr[0].Trim() == "Alignment_HeadingDetermined")
                        Alignment_HeadingDetermined = tmpstr[1];
                    if (tmpstr[0].Trim() == "Alignment_HeadingValue")
                        Alignment_HeadingValue = tmpstr[1];

                    if (tmpstr[0].Trim() == "NoiseParamDetermin_mode")
                        NoiseParamDetermin_mode = tmpstr[1];
                    if (tmpstr[0].Trim() == "NoiseParamDetermin_startTime")
                        NoiseParamDetermin_startTime = tmpstr[1];
                    if (tmpstr[0].Trim() == "NoiseParamDetermin_endTime")
                        NoiseParamDetermin_endTime = tmpstr[1];
                    if (tmpstr[0].Trim() == "NoiseParamDetermin_SigmaValueF")
                        NoiseParamDetermin_SigmaValueF = tmpstr[1];
                    if (tmpstr[0].Trim() == "NoiseParamDetermin_SigmaValueNu")
                        NoiseParamDetermin_SigmaValueNu = tmpstr[1];

                    if (tmpstr[0].Trim() == "Alignment_RollDetermined")
                        Alignment_RollDetermined = tmpstr[1];
                    if (tmpstr[0].Trim() == "Alignment_RollValue")
                        Alignment_RollValue = tmpstr[1];

                    if (tmpstr[0].Trim() == "Alignment_PitchDetermined")
                        Alignment_PitchDetermined = tmpstr[1];
                    if (tmpstr[0].Trim() == "Alignment_PitchValue")
                        Alignment_PitchValue = tmpstr[1];

                    if (tmpstr[0].Trim() == "SINS_is_accurateMounted_by_kappa_1")
                        SINS_is_accurateMounted_by_kappa_1 = tmpstr[1];

                    if (tmpstr[0].Trim() == "SINS_is_accurateMounted_by_kappa_3")
                        SINS_is_accurateMounted_by_kappa_3 = tmpstr[1];

                    if (tmpstr[0].Trim() == "SINS_is_accurateMounted_by_scaleError")
                        SINS_is_accurateMounted_by_scaleError = tmpstr[1];

                    if (tmpstr[0].Trim() == "StartHeightCorrection_flag")
                        StartHeightCorrection_flag = tmpstr[1];
                    if (tmpstr[0].Trim() == "StartHeightCorrection_value")
                        StartHeightCorrection_value = tmpstr[1];

                    if (tmpstr[0].Trim() == "alpha_kappa_1")
                        alpha_kappa_1 = tmpstr[1];

                    if (tmpstr[0].Trim() == "alpha_kappa_3")
                        alpha_kappa_3 = tmpstr[1];

                    if (tmpstr[0].Trim() == "ScaleError")
                        ScaleError = tmpstr[1];

                    if (tmpstr[0].Trim() == "initError_kappa_1")
                        initError_kappa_1 = tmpstr[1];

                    if (tmpstr[0].Trim() == "initError_kappa_3")
                        initError_kappa_3 = tmpstr[1];

                    if (tmpstr[0].Trim() == "initError_ScaleError")
                        initError_ScaleError = tmpstr[1];

                    //if (tmpstr[0].Trim() == "CalibrationStartMode")
                    //    CalibrationStartMode = tmpstr[1];
                }
            }


            SINSstate.timeStep = SINSstate.Freq = Convert.ToDouble(frequency);

            // --- Лишь каждое OdoLimitMeasuresNum обновление показаний одометра будут использоваться для коррекции --- //
            SINSstate.OdoLimitMeasuresNum = 5;

            // --- Количество тактов БИНС для начальной выставки от начала  --- //
            ProcHelp.AlignmentCounts = Convert.ToInt32(Math.Round(Convert.ToDouble(AlignmentDuration) / SINSstate.Freq));

            SINSstate.AlignmentEngineIsOff = Convert.ToInt32(AlignmentEngineIsOff);

            SINSstate.NoiseParamDetermin_mode = Convert.ToInt32(NoiseParamDetermin_mode);
            SINSstate.NoiseParamDetermin_startTime = Convert.ToInt32(Math.Round(Convert.ToDouble(NoiseParamDetermin_startTime) / SINSstate.Freq));
            SINSstate.NoiseParamDetermin_endTime = Math.Min(Convert.ToInt32(Math.Round(Convert.ToDouble(NoiseParamDetermin_endTime) / SINSstate.Freq)), ProcHelp.AlignmentCounts);

            SINSstate.NoiseParamDetermin_SigmaValueF = Convert.ToDouble(NoiseParamDetermin_SigmaValueF);
            SINSstate.NoiseParamDetermin_SigmaValueNu = Convert.ToDouble(NoiseParamDetermin_SigmaValueNu);

            // --- Заданные значения начальных углов ориентации: флаги и значения --- //
            if (Convert.ToInt32(Alignment_HeadingDetermined) == 1)
                SINSstate.Alignment_HeadingDetermined = true;
            else
                SINSstate.Alignment_HeadingDetermined = false;
            SINSstate.Alignment_HeadingValue = Convert.ToDouble(Alignment_HeadingValue) * SimpleData.ToRadian;

            if (Convert.ToInt32(Alignment_RollDetermined) == 1)
                SINSstate.Alignment_RollDetermined = true;
            else
                SINSstate.Alignment_RollDetermined = false;
            SINSstate.Alignment_RollValue = Convert.ToDouble(Alignment_RollValue) * SimpleData.ToRadian;

            if (Convert.ToInt32(Alignment_PitchDetermined) == 1)
                SINSstate.Alignment_PitchDetermined = true;
            else
                SINSstate.Alignment_PitchDetermined = false;
            SINSstate.Alignment_PitchValue = Convert.ToDouble(Alignment_PitchValue) * SimpleData.ToRadian;


            if (Convert.ToInt32(StartHeightCorrection_flag) == 1)
                SINSstate.first_N_meters_StartHeightCorrection_flag = true;
            else
                SINSstate.first_N_meters_StartHeightCorrection_flag = false;
            SINSstate.first_N_meters_StartHeightCorrection_value = Convert.ToDouble(StartHeightCorrection_value);

            //if (Convert.ToInt32(CalibrationStartMode) == 1)
            //    SINSstate.CalibrationStartMode = true;

            // --- Углы рассогласования осей БИНС и динамических осей объекта, если это нужно --- //
            SINSstate.alpha_kappa_3 = Convert.ToDouble(alpha_kappa_3) * SimpleData.ToRadian; // -- Угол рассогласования по курсу
            SINSstate.alpha_kappa_1 = Convert.ToDouble(alpha_kappa_1) * SimpleData.ToRadian; // -- Угол рассогласования по тангажу
            SINSstate.alpha_scaleError = Convert.ToDouble(ScaleError) / 100.0;


            SINSstate.initError_kappa_3 = Convert.ToDouble(initError_kappa_3) * SimpleData.ToRadian; // -- Специальная введенная ошибка по курсу
            SINSstate.initError_kappa_1 = Convert.ToDouble(initError_kappa_1) * SimpleData.ToRadian; // -- Специальная введенная ошибка по тангажу
            SINSstate.initError_scaleError = Convert.ToDouble(initError_ScaleError) / 100.0;


            // --- Шум по горизонтальным ошибкам координат --- //
            KalmanVars.Noise_Pos = 1.0;
            // --- Шум по вертикальным ошибкам координат --- //
            KalmanVars.Noise_Pos_Vertical = 0.01;
            // -------------------------------------------//

            // --- определяем мультипликатор для шума вертикального измерения по одометру
            SINSstate.OdoVerticalNoiseMultiplicator = 2;

            // --- Начальные ковариации --- //
            SINSstate.stdR = 0.01; // по ошибке координат БИНС, м
            SINSstate.stdOdoR = 0.01; // по ошибке координат одометрического счисления, м
            SINSstate.stdV = 0.01;

            // --- По флагу Точно ли установлен БИНС на корпусе объекта, определяем соответствующие начальные значения ковариаций
            if (Convert.ToInt32(SINS_is_accurateMounted_by_kappa_1) == 1)
                SINSstate.stdKappa1 = 1.0;
            else
                SINSstate.stdKappa1 = 10.0;

            if (Convert.ToInt32(SINS_is_accurateMounted_by_kappa_3) == 1)
                SINSstate.stdKappa3 = 1.0;
            else
                SINSstate.stdKappa3 = 10.0;


            SINSstate.stdScale = 0.01;

            SINSstate.Noise_Marker_PositionError = 0.1; // в метрах
            SINSstate.Noise_GPS_PositionError = 2.0; // в метрах
            KalmanVars.OdoNoise_STOP = 0.1;

            // --- Начальные координаты --- //
            ProcHelp.LongSNS = SINSstate_OdoMod.Longitude = SINSstate.Longitude_Start = SINSstate.LongSNS = SINSstate.Longitude = Convert.ToDouble(StartLongitude) * SimpleData.ToRadian;
            ProcHelp.LatSNS = SINSstate_OdoMod.Latitude = SINSstate.Latitude_Start = SINSstate.LatSNS = SINSstate.Latitude = Convert.ToDouble(StartLatitude) * SimpleData.ToRadian;
            ProcHelp.AltSNS = SINSstate_OdoMod.Height = SINSstate.Altitude_Start = SINSstate.AltSNS = SINSstate.Height = SINSstate.Altitude_prev = Convert.ToDouble(StartHeight);


            ////--- В случае выставления значения поправки на угол kappa_3 именшаем нач.ковариацию ---//
            //if (Math.Abs(SINSstate.alpha_kappa_3) > 0.00001 * SimpleData.ToRadian)
            //    SINSstate.stdKappa3 = 1.0; //минут

            ////--- В случае выставления значения поправки на угол kappa_1 именшаем нач.ковариацию ---//
            //if (Math.Abs(SINSstate.alpha_kappa_1) > 0.00001 * SimpleData.ToRadian)
            //    SINSstate.stdKappa1 = 1.0; //минут

            //--- В случае выставления значения поправки на погрешность масштаба именшаем нач.ковариацию ---//
            if (Math.Abs(SINSstate.alpha_scaleError) > 0.00001)
                SINSstate.stdScale = 0.001;


            ProcHelp.LongSNS = ProcHelp.LongSNS * 180 / Math.PI;
            ProcHelp.LatSNS = ProcHelp.LatSNS * 180 / Math.PI;

            ApplyMatrixStartCondition(SINSstate);
            ApplyMatrixStartCondition(SINSstate_OdoMod);


            //--- Если запуск производится в режиме калибровочки, то меняем начальные ковариации
            if (SINSstate.CalibrationStartMode)
            {
                //KalmanVars.Noise_Pos = 0.1;
                if (SINSstate.alpha_kappa_1 == 0)
                    SINSstate.stdKappa1 = 60.0;

                if (SINSstate.alpha_kappa_3 == 0)
                    SINSstate.stdKappa3 = 60.0;

                if (SINSstate.alpha_scaleError == 0)
                    SINSstate.stdScale = 0.02;
            }

        }
    }
}
