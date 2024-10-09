namespace PathFinding
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
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
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            button1 = new Button();
            textBox1 = new TextBox();
            manualTb = new TextBox();
            modeTb = new TextBox();
            SuspendLayout();
            // 
            // button1
            // 
            button1.BackColor = Color.White;
            button1.Location = new Point(12, 179);
            button1.Name = "button1";
            button1.Size = new Size(161, 46);
            button1.TabIndex = 1;
            button1.TabStop = false;
            button1.Text = "Move Off";
            button1.UseVisualStyleBackColor = false;
            button1.Click += button1_Click;
            // 
            // textBox1
            // 
            textBox1.Location = new Point(12, 102);
            textBox1.Multiline = true;
            textBox1.Name = "textBox1";
            textBox1.Size = new Size(161, 57);
            textBox1.TabIndex = 2;
            // 
            // manualTb
            // 
            manualTb.Enabled = false;
            manualTb.Font = new Font("맑은 고딕", 5F);
            manualTb.Location = new Point(12, 319);
            manualTb.Multiline = true;
            manualTb.Name = "manualTb";
            manualTb.Size = new Size(161, 329);
            manualTb.TabIndex = 3;
            manualTb.TabStop = false;
            // 
            // modeTb
            // 
            modeTb.Enabled = false;
            modeTb.Location = new Point(12, 231);
            modeTb.Multiline = true;
            modeTb.Name = "modeTb";
            modeTb.Size = new Size(161, 82);
            modeTb.TabIndex = 4;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(14F, 32F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(2298, 1221);
            Controls.Add(modeTb);
            Controls.Add(manualTb);
            Controls.Add(textBox1);
            Controls.Add(button1);
            KeyPreview = true;
            Name = "Form1";
            StartPosition = FormStartPosition.CenterScreen;
            Text = "Form1";
            WindowState = FormWindowState.Maximized;
            Load += Form1_Load;
            SizeChanged += Form1_SizeChanged;
            KeyDown += Form1_KeyDown;
            MouseDown += Form1_MouseDown;
            MouseMove += Form1_MouseMove;
            MouseUp += Form1_MouseUp;
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion
        private Button button1;
        private TextBox textBox1;
        private TextBox manualTb;
        private TextBox modeTb;
    }
}
