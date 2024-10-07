using Microsoft.VisualBasic.Devices;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Collections.Generic;
using System.Drawing;
namespace PathFinding
{

    public partial class Form1 : Form
    {
        int cellSize;

        EnumColor[,] gridColor;
        EnumMode mode = EnumMode.SETTING;

        bool bDrag = false;
        bool bErase = false;

        PathFinder pathFinder = new PathFinder();

        Queue<ValueTuple<int, int, EnumColor>> naviQ = new (3000); // y, x, color

        System.Windows.Forms.Timer naviTimer = new System.Windows.Forms.Timer();

        public Form1()
        {
            InitializeComponent();

            this.SetStyle(ControlStyles.OptimizedDoubleBuffer, true);
            this.SetStyle(ControlStyles.AllPaintingInWmPaint, true);
            this.SetStyle(ControlStyles.UserPaint, true);

            gridColor = new EnumColor[Grid.height, Grid.width];

            for (int i = 0; i < Grid.height; i++)
            {
                for (int j = 0; j < Grid.width; j++)
                {
                    gridColor[i, j] = EnumColor.NO_USE;
                }
            }

            int big = Width > Height ? Width : Height;
            int small = Width < Height ? Width : Height;

            double changedMul = (double)big / small;
            cellSize = (int)(Grid.cellSize / changedMul);


            naviTimer.Tick += new EventHandler(Navigate);
            naviTimer.Interval = 50;

            MouseWheel += new MouseEventHandler(MouseWheelEvent);
        }

        private void MouseWheelEvent(object? sender, MouseEventArgs e)
        {
            var scrollValue = e.Delta * SystemInformation.MouseWheelScrollLines / 120;

            // TODO
        }

        private void Navigate(object? sender, EventArgs e)
        {
            naviQ.Clear();
            bool res = pathFinder.Navigate(naviQ);

            while(naviQ.Count > 0)
            {
                var vTuple = naviQ.Dequeue();

                // == gridColor[yPos, xPos] = color;
                gridColor[vTuple.Item1, vTuple.Item2] = vTuple.Item3;
            }

            if(res) // if successed return true
            {
                naviTimer.Stop();
            }

            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;

            // 그리드의 각 셀을 그리기
            for (int y = 0; y < Grid.height; y++)
            {
                for (int x = 0; x < Grid.width; x++)
                {
                    // 셀의 상태에 따라 색상 결정
                    Color color = Color.White;
                    switch (gridColor[y, x])
                    {
                        case EnumColor.NO_USE: break;
                        case EnumColor.OBSTACLE: color = Color.Gray; break;
                        case EnumColor.START_POINT: color = Color.Red; break;
                        case EnumColor.END_POINT: color = Color.Magenta; break;
                        case EnumColor.INTENDED: color = Color.Orange; break;
                        case EnumColor.END_SEARCH: color = Color.LightGray; break;
                        case EnumColor.PATH: color = Color.Blue; break;
                        case EnumColor.PATH_NODE: color = Color.ForestGreen; break;
                        default: break;
                    }
                    g.FillRectangle(new SolidBrush(color), Grid.buttonXPaddingSize + x * cellSize, y * cellSize, cellSize - 1, cellSize - 1);
                    g.DrawRectangle(Pens.Black, Grid.buttonXPaddingSize + x * cellSize, y * cellSize, cellSize - 1, cellSize - 1);
                }
            }
        }
        private void Form1_Load(object sender, EventArgs e)
        {
        }

        private void Form1_SizeChanged(object sender, EventArgs e)
        {

            int big = Width > Height ? Width : Height;
            int small = Width < Height ? Width : Height;

            double changedMul = (double)big / small;
            cellSize = (int)(Grid.cellSize / changedMul);

            Invalidate();

        }


        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Middle)
            {
                if (mode == EnumMode.SETTING)
                    mode = EnumMode.BLOCKING;
                else
                    mode = EnumMode.SETTING;

                return;
            }

            int xPos = (e.Location.X - 200) / cellSize;
            int yPos = (e.Location.Y / cellSize); ;

            if (!Grid.IsRightPos(xPos, yPos))
                return;

            if (mode == EnumMode.SETTING)
            {
                if (e.Button == MouseButtons.Left)
                {
                    if (!pathFinder.StartNode.isUsable && gridColor[yPos, xPos] == EnumColor.NO_USE)
                    {
                        pathFinder.StartNode.xPos = xPos;
                        pathFinder.StartNode.yPos = yPos;
                        pathFinder.StartNode.isUsable = true;

                        gridColor[yPos, xPos] = EnumColor.START_POINT;
                    }
                    else if (pathFinder.StartNode.isUsable && gridColor[yPos, xPos] == EnumColor.START_POINT)
                    {
                        pathFinder.StartNode.isUsable = false;
                        gridColor[yPos, xPos] = EnumColor.NO_USE;
                    }
                }

                else if (e.Button == MouseButtons.Right)
                {
                    if (!pathFinder.EndNode.isUsable && gridColor[yPos, xPos] == EnumColor.NO_USE)
                    {
                        pathFinder.EndNode.xPos = xPos;
                        pathFinder.EndNode.yPos = yPos;
                        pathFinder.EndNode.isUsable = true;
                        gridColor[yPos, xPos] = EnumColor.END_POINT;
                    }
                    else if (pathFinder.EndNode.isUsable && gridColor[yPos, xPos] == EnumColor.END_POINT)
                    {
                        pathFinder.EndNode.isUsable = false;
                        gridColor[yPos, xPos] = EnumColor.NO_USE;
                    }
                }
            }
            else // Blocking
            {
                if (e.Button == MouseButtons.Left)
                {
                    bDrag = true;
                    bErase = gridColor[yPos, xPos] != EnumColor.NO_USE;

                }
            }
            Invalidate();

        }

        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            if (mode != EnumMode.BLOCKING || !bDrag)
                return;

            int xPos = (e.Location.X - 200) / cellSize;
            int yPos = (e.Location.Y / cellSize);

            if (!Grid.IsRightPos(xPos, yPos))
                return;

            if (gridColor[yPos, xPos] != EnumColor.OBSTACLE && gridColor[yPos, xPos] != EnumColor.NO_USE)
                return;

            if (!bErase)
                gridColor[yPos, xPos] = EnumColor.OBSTACLE;
            else
                gridColor[yPos, xPos] = EnumColor.NO_USE;

            Invalidate();
        }

        //private bool IsRightPos(int xPos, int yPos)
        //{
        //    if (xPos < 0 || xPos >= Grid.width)
        //        return false;

        //    if (yPos < 0 || yPos >= Grid.height)
        //        return false;

        //    return true;
        //}
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            bDrag = false;
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
             {
                if (!naviTimer.Enabled && pathFinder.CanNavi())
                {
                    naviTimer.Start();
                }
            }
            // ESC, Delete
            else if(e.KeyCode == Keys.Escape)
            {
                if (naviTimer.Enabled)
                    naviTimer.Stop();

                if(pathFinder.StartNode.isUsable)
                gridColor[pathFinder.StartNode.yPos, pathFinder.StartNode.xPos] = EnumColor.NO_USE; 
                
                if(pathFinder.EndNode.isUsable)
                gridColor[pathFinder.EndNode.yPos, pathFinder.EndNode.xPos] = EnumColor.NO_USE; 
                pathFinder.Initialize();

                Invalidate();
            }
        }
    }

}
