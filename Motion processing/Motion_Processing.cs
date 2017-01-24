using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using Common_Namespace;
using Alignment;
using SINSProcessingModes;
using Motion_Processing;
using System.Text.RegularExpressions;



namespace Motion_Processing
{
    public partial class SINS_Processing : Form
    {
        int iMx = SimpleData.iMx = 25;
        int iMq = SimpleData.iMq = SimpleData.iMx;
        int iMz = SimpleData.iMz = 15;
        int iMxSmthd = SimpleData.iMxSmthd = 9;

        int iMx_Vertical = SimpleData.iMx_Vertical = 25;
        int iMq_Vertical = SimpleData.iMq_Vertical = SimpleData.iMx_Vertical;

        StreamReader myFile;

        ParamToStart ParamStart = new ParamToStart();
        SINS_State SINSstate, SINSstate_OdoMod;
        Kalman_Vars KalmanVars;
        Proc_Help ProcHelp;

        int value_iMx_dV_12, value_iMx_alphaBeta, value_iMx_Nu0, value_iMx_f0_12, value_iMx_f0_3;
        int value_iMx_r_odo_12 = 0, value_iMx_kappa_3_ds = 0, Vertical_kappa1, Vertical_f0_3, Vertical_rOdo3;


        public static StreamWriter GRTV_output = new StreamWriter(SimpleData.PathOutputString + "\\TXT for GRTV\\S_GRTV_output.txt");


        public SINS_Processing()
        {
            InitializeComponent();
        }


        public void Main_Block_Click_new_Click(object sender, EventArgs e)
        {
            this.Single_Navigation_Processing();

            GRTV_output.Close();
            this.Close();
        }



        public void Single_Navigation_Processing()
        {
            int l = 0;

            this.DefineDimentionOfErrorVector();                                                            //---формирование размерности вектора ошибок---//
            this.DefineClassElementAndFlags();

            //---Инициализация начальных условий при отсутствии выставки---//
            StartParameters.StartSINS_Parameters(SINSstate, SINSstate_OdoMod, KalmanVars, ParamStart, ProcHelp);

            //--- ВЫБОР ВХОДНОГО ФАЛА С ДАННЫМИ ---//
            myFile = new StreamReader(SimpleData.PathInputString + "\\" + SINSstate.DataInFileName);

            // --- Грубая начальная выставка --- //
            l = Alignment_Rought.SINS_Alignment_Rought(ProcHelp, SINSstate, SINSstate_OdoMod, myFile, KalmanVars, GRTV_output);

            //--- stdF и stdNu значения, определяющие классы точности датчиков. На основе них формируется стартовые ковариации инструментальных погрешностей инерц.датчиков, а также угловых ошибок ---
            if (SINSstate.flag_AccuracyClass_0_02grph)
                for (int j = 0; j < 3; j++)
                {
                    SINSstate.stdF[j] = 1E-5 * 9.81; //далее умножается G
                    SINSstate.stdNu = 0.01; //град/час
                }
            if (SINSstate.flag_AccuracyClass_0_2_grph)
                for (int j = 0; j < 3; j++)
                {
                    SINSstate.stdF[j] = 1E-4 * 9.81; //далее умножается G
                    SINSstate.stdNu = 0.1; //град/час
                }


            // --- Формируются значения начальных ковариаций для ошибок углов ориентации
            SINSstate.stdAlpha1 = SINSstate.stdF[1] / 9.81; //радиан
            SINSstate.stdAlpha2 = SINSstate.stdF[0] / 9.81; //радиан
            SINSstate.stdBeta3 = SINSstate.stdNu * SimpleData.ToRadian / 3600.0 / (SimpleData.U * Math.Cos(SINSstate.Latitude)); //радиан


            // --- Инициализация матрицы начальной ковариации
            SINSprocessing.InitOfCovarianceMatrixes(SINSstate, KalmanVars);

            //---Переопределяем размерности векторов и матриц после выставки---
            this.DefineDimentionOfErrorVector();

            ////------ Основная функция навигации, БИНС + ОДОМЕТР ------
            SINS_Corrected.SINS_Corrected_Processing(l, false, myFile, SINSstate, KalmanVars, ProcHelp, SINSstate_OdoMod, GRTV_output);

            myFile.Close();
        }

        //---------------------------------------------------------------------------------------
        //---------------------------------------------------------------------------------------
        //---------------------------------------------------------------------------------------








        public void DefineDimentionOfErrorVector()
        {
            iMx = SimpleData.iMx = 0;

            // ---------- dR ----------//
            iMx = SimpleData.iMx += 2;

            // ---------- dV_12 ----------//
            value_iMx_dV_12 = SimpleData.iMx;
            iMx = SimpleData.iMx += 2;

            // ---------- alphaBeta ----------//
            value_iMx_alphaBeta = SimpleData.iMx;
            iMx = SimpleData.iMx += 3;

            // ---------- Nu0 ----------//
            value_iMx_Nu0 = SimpleData.iMx;
            iMx = SimpleData.iMx += 3;

            // ---------- f0_12 ----------//
            value_iMx_f0_12 = SimpleData.iMx;
            iMx = SimpleData.iMx += 2;

            // ---------- f0_3 ----------//
            value_iMx_f0_3 = SimpleData.iMx;
            iMx = SimpleData.iMx += 1;

            // ---------- dR_ODO_12 ----------//
            value_iMx_r_odo_12 = SimpleData.iMx;
            iMx = SimpleData.iMx += 2;

            // ---------- kappa_3 and Scale Error ----------//
            if (this.noOdoModelEstimate.Checked == false)
            {
                value_iMx_kappa_3_ds = SimpleData.iMx;
                iMx = SimpleData.iMx += 2;
            }


            //------------Размерность Вектора шумов----------//
            iMq = SimpleData.iMq = SimpleData.iMx;






            // ------------------------------------------//
            // ----------ВЕРТИКАЛЬНЫЙ КАНАЛ---------------//
            // ------------------------------------------//
            SimpleData.iMx_Vertical = 2;

            // --- Вертикальный одометрический канал
            // --- Если не включаем ошибки одометра в вектор, то из вертикального канала убираем Vertical_rOdo3 вовсе
            Vertical_rOdo3 = SimpleData.iMx_Vertical;
            SimpleData.iMx_Vertical++;

            // --- Добавляем вертикальный 0 ньютонометра ---
            Vertical_f0_3 = SimpleData.iMx_Vertical;
            SimpleData.iMx_Vertical++;

            // --- Если включаем ошибки одометра в вектор
            if (this.noOdoModelEstimate.Checked == false)
            {
                Vertical_kappa1 = SimpleData.iMx_Vertical;
                SimpleData.iMx_Vertical += 1;
            }

            // ---------------------------//
            SimpleData.iMq_Vertical = SimpleData.iMx_Vertical;
            // ---------------------------//

        }

        public void DefineClassElementAndFlags()
        {
            SINSstate = new SINS_State(); SINSstate_OdoMod = new SINS_State();
            KalmanVars = new Kalman_Vars();
            ProcHelp = new Proc_Help();

            SINSstate.FreqOutput = Convert.ToInt32(this.Output_Freq.Text);

            SINSstate.value_iMx_dV_12 = value_iMx_dV_12;
            SINSstate.value_iMx_alphaBeta = value_iMx_alphaBeta;
            SINSstate.value_iMx_Nu0 = value_iMx_Nu0;
            SINSstate.value_iMx_f0_12 = value_iMx_f0_12;
            SINSstate.value_iMx_f0_3 = value_iMx_f0_3;
            SINSstate.value_iMx_r_odo_12 = value_iMx_r_odo_12;
            SINSstate.value_iMx_kappa_3_ds = value_iMx_kappa_3_ds;

            SINSstate.flag_using_Checkpotints = this.flag_using_Checkpotints.Checked;

            SINSstate.flag_AutonomouseSolution = this.AutonomouseSolution.Checked;
            SINSstate.flag_noOdoModelEstimate = this.noOdoModelEstimate.Checked;
            SINSstate.flag_AccuracyClass_0_02grph = this.AccuracyClass_0_02grph.Checked;
            SINSstate.flag_AccuracyClass_0_2_grph = this.AccuracyClass_0_2_grph.Checked;

            SINSstate.flag_GRTV_output = this.flag_GRTV_output.Checked;


            // ------------------------------------------//
            SINSstate.Vertical_kappa1 = Vertical_kappa1;
            SINSstate.Vertical_f0_3 = Vertical_f0_3;
            SINSstate.Vertical_rOdo3 = Vertical_rOdo3;
        }


        private void SINS_Processing_Load(object sender, EventArgs e)
        {
        }

        private void AccuracyClass_0_02grph_CheckedChanged(object sender, EventArgs e)
        {
            if (this.AccuracyClass_0_02grph.Checked)
            {
                if (ConfigFileName.Text != "")
                    this.Main_Block_Click_new.Enabled = true;
                this.AccuracyClass_0_2_grph.Enabled = false;
            }
            else
            {
                this.Main_Block_Click_new.Enabled = false;
                this.AccuracyClass_0_2_grph.Enabled = true;
            }
        }

        private void AccuracyClass_0_2_grph_CheckedChanged(object sender, EventArgs e)
        {
            if (this.AccuracyClass_0_2_grph.Checked)
            {
                if (ConfigFileName.Text != "")
                    this.Main_Block_Click_new.Enabled = true;
                this.AccuracyClass_0_02grph.Enabled = false;
            }
            else
            {
                this.Main_Block_Click_new.Enabled = false;
                this.AccuracyClass_0_02grph.Enabled = true;
            }
        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            openFileDialog1.InitialDirectory = SimpleData.PathInputConfigurations;
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                SimpleData.ConfigurationFileIn = openFileDialog1.FileName;
                string[] tmpstr = SimpleData.ConfigurationFileIn.Split('\\');
                ConfigFileName.Text = tmpstr[tmpstr.Length-1];

                if (ConfigFileName.Text != "" && (this.AccuracyClass_0_2_grph.Checked || this.AccuracyClass_0_02grph.Checked))
                    this.Main_Block_Click_new.Enabled = true;
                else
                    this.Main_Block_Click_new.Enabled = false;
            }
        }









    }
}
