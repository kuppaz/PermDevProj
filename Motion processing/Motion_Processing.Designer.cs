namespace Motion_Processing
{
    partial class SINS_Processing
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.Main_Block_Click_new = new System.Windows.Forms.Button();
            this.Output_Freq = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.flag_using_Checkpotints = new System.Windows.Forms.CheckBox();
            this.groupBox10 = new System.Windows.Forms.GroupBox();
            this.AccuracyClass_0_02grph = new System.Windows.Forms.CheckBox();
            this.AccuracyClass_0_2_grph = new System.Windows.Forms.CheckBox();
            this.groupBox14 = new System.Windows.Forms.GroupBox();
            this.flag_GRTV_output = new System.Windows.Forms.CheckBox();
            this.label2 = new System.Windows.Forms.Label();
            this.button1 = new System.Windows.Forms.Button();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.ConfigFileName = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.AutonomouseSolution = new System.Windows.Forms.CheckBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.noOdoModelEstimate = new System.Windows.Forms.CheckBox();
            this.groupBox10.SuspendLayout();
            this.groupBox14.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // Main_Block_Click_new
            // 
            this.Main_Block_Click_new.Enabled = false;
            this.Main_Block_Click_new.Location = new System.Drawing.Point(494, 59);
            this.Main_Block_Click_new.Name = "Main_Block_Click_new";
            this.Main_Block_Click_new.Size = new System.Drawing.Size(83, 92);
            this.Main_Block_Click_new.TabIndex = 0;
            this.Main_Block_Click_new.Text = "Start";
            this.Main_Block_Click_new.UseVisualStyleBackColor = true;
            this.Main_Block_Click_new.Click += new System.EventHandler(this.Main_Block_Click_new_Click);
            // 
            // Output_Freq
            // 
            this.Output_Freq.Location = new System.Drawing.Point(97, 22);
            this.Output_Freq.Name = "Output_Freq";
            this.Output_Freq.Size = new System.Drawing.Size(28, 20);
            this.Output_Freq.TabIndex = 28;
            this.Output_Freq.Text = "100";
            this.Output_Freq.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(28, 25);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(66, 13);
            this.label1.TabIndex = 29;
            this.label1.Text = "Output Freq:";
            // 
            // flag_using_Checkpotints
            // 
            this.flag_using_Checkpotints.AutoSize = true;
            this.flag_using_Checkpotints.Location = new System.Drawing.Point(15, 26);
            this.flag_using_Checkpotints.Name = "flag_using_Checkpotints";
            this.flag_using_Checkpotints.Size = new System.Drawing.Size(110, 17);
            this.flag_using_Checkpotints.TabIndex = 52;
            this.flag_using_Checkpotints.Text = "Use ControlPoints";
            this.flag_using_Checkpotints.UseVisualStyleBackColor = true;
            // 
            // groupBox10
            // 
            this.groupBox10.Controls.Add(this.flag_using_Checkpotints);
            this.groupBox10.Location = new System.Drawing.Point(340, 37);
            this.groupBox10.Name = "groupBox10";
            this.groupBox10.Size = new System.Drawing.Size(140, 55);
            this.groupBox10.TabIndex = 39;
            this.groupBox10.TabStop = false;
            this.groupBox10.Text = "Correction Modes";
            // 
            // AccuracyClass_0_02grph
            // 
            this.AccuracyClass_0_02grph.AutoSize = true;
            this.AccuracyClass_0_02grph.Enabled = false;
            this.AccuracyClass_0_02grph.Location = new System.Drawing.Point(17, 33);
            this.AccuracyClass_0_02grph.Name = "AccuracyClass_0_02grph";
            this.AccuracyClass_0_02grph.Size = new System.Drawing.Size(158, 17);
            this.AccuracyClass_0_02grph.TabIndex = 65;
            this.AccuracyClass_0_02grph.Text = "AccuracyClass- 0.01 grad/h";
            this.AccuracyClass_0_02grph.UseVisualStyleBackColor = true;
            this.AccuracyClass_0_02grph.CheckedChanged += new System.EventHandler(this.AccuracyClass_0_02grph_CheckedChanged);
            // 
            // AccuracyClass_0_2_grph
            // 
            this.AccuracyClass_0_2_grph.AutoSize = true;
            this.AccuracyClass_0_2_grph.Checked = true;
            this.AccuracyClass_0_2_grph.CheckState = System.Windows.Forms.CheckState.Checked;
            this.AccuracyClass_0_2_grph.Location = new System.Drawing.Point(17, 52);
            this.AccuracyClass_0_2_grph.Name = "AccuracyClass_0_2_grph";
            this.AccuracyClass_0_2_grph.Size = new System.Drawing.Size(152, 17);
            this.AccuracyClass_0_2_grph.TabIndex = 66;
            this.AccuracyClass_0_2_grph.Text = "AccuracyClass- 0.1 grad/h";
            this.AccuracyClass_0_2_grph.UseVisualStyleBackColor = true;
            this.AccuracyClass_0_2_grph.CheckedChanged += new System.EventHandler(this.AccuracyClass_0_2_grph_CheckedChanged);
            // 
            // groupBox14
            // 
            this.groupBox14.Controls.Add(this.AccuracyClass_0_02grph);
            this.groupBox14.Controls.Add(this.AccuracyClass_0_2_grph);
            this.groupBox14.Location = new System.Drawing.Point(147, 96);
            this.groupBox14.Name = "groupBox14";
            this.groupBox14.Size = new System.Drawing.Size(187, 90);
            this.groupBox14.TabIndex = 80;
            this.groupBox14.TabStop = false;
            this.groupBox14.Text = "Aprior SINS Accuracy Class";
            // 
            // flag_GRTV_output
            // 
            this.flag_GRTV_output.AutoSize = true;
            this.flag_GRTV_output.Location = new System.Drawing.Point(355, 146);
            this.flag_GRTV_output.Name = "flag_GRTV_output";
            this.flag_GRTV_output.Size = new System.Drawing.Size(106, 17);
            this.flag_GRTV_output.TabIndex = 83;
            this.flag_GRTV_output.Text = "Output for GRTV";
            this.flag_GRTV_output.UseVisualStyleBackColor = true;
            // 
            // label2
            // 
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.25F);
            this.label2.Location = new System.Drawing.Point(346, 116);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(132, 27);
            this.label2.TabIndex = 84;
            this.label2.Text = "Флаг для вывода данных для    их преобразования в GRTV";
            this.label2.Click += new System.EventHandler(this.label2_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(36, 119);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 85;
            this.button1.Text = "ChooseFile";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // ConfigFileName
            // 
            this.ConfigFileName.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.AllSystemSources;
            this.ConfigFileName.Enabled = false;
            this.ConfigFileName.Location = new System.Drawing.Point(12, 95);
            this.ConfigFileName.Name = "ConfigFileName";
            this.ConfigFileName.Size = new System.Drawing.Size(129, 20);
            this.ConfigFileName.TabIndex = 86;
            // 
            // label3
            // 
            this.label3.Location = new System.Drawing.Point(9, 65);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(137, 27);
            this.label3.TabIndex = 87;
            this.label3.Text = "Выбирете конфигурационный файл";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // AutonomouseSolution
            // 
            this.AutonomouseSolution.AutoSize = true;
            this.AutonomouseSolution.Location = new System.Drawing.Point(7, 19);
            this.AutonomouseSolution.Name = "AutonomouseSolution";
            this.AutonomouseSolution.Size = new System.Drawing.Size(168, 17);
            this.AutonomouseSolution.TabIndex = 88;
            this.AutonomouseSolution.Text = "Автономное решение БИНС";
            this.AutonomouseSolution.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.noOdoModelEstimate);
            this.groupBox1.Controls.Add(this.AutonomouseSolution);
            this.groupBox1.Location = new System.Drawing.Point(147, 23);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(187, 67);
            this.groupBox1.TabIndex = 89;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Режимы запуска";
            // 
            // noOdoModelEstimate
            // 
            this.noOdoModelEstimate.AutoSize = true;
            this.noOdoModelEstimate.Location = new System.Drawing.Point(7, 40);
            this.noOdoModelEstimate.Name = "noOdoModelEstimate";
            this.noOdoModelEstimate.Size = new System.Drawing.Size(179, 17);
            this.noOdoModelEstimate.TabIndex = 89;
            this.noOdoModelEstimate.Text = "Без модели ошибок одометра";
            this.noOdoModelEstimate.UseVisualStyleBackColor = true;
            // 
            // SINS_Processing
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(591, 207);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.ConfigFileName);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.Output_Freq);
            this.Controls.Add(this.flag_GRTV_output);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.groupBox14);
            this.Controls.Add(this.Main_Block_Click_new);
            this.Controls.Add(this.groupBox10);
            this.Name = "SINS_Processing";
            this.Text = "SINS Solution";
            this.Load += new System.EventHandler(this.SINS_Processing_Load);
            this.groupBox10.ResumeLayout(false);
            this.groupBox10.PerformLayout();
            this.groupBox14.ResumeLayout(false);
            this.groupBox14.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button Main_Block_Click_new;
        private System.Windows.Forms.TextBox Output_Freq;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.GroupBox groupBox10;
        private System.Windows.Forms.CheckBox flag_using_Checkpotints;
        private System.Windows.Forms.CheckBox AccuracyClass_0_02grph;
        private System.Windows.Forms.CheckBox AccuracyClass_0_2_grph;
        private System.Windows.Forms.GroupBox groupBox14;
        private System.Windows.Forms.CheckBox flag_GRTV_output;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.TextBox ConfigFileName;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.CheckBox AutonomouseSolution;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.CheckBox noOdoModelEstimate;
    }
}

