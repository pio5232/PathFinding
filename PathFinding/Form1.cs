using Microsoft.VisualBasic.Devices;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
namespace PathFinding
{

    public partial class Form1 : Form
    {
        private int mouseOffsetX = 0;
        private int mouseOffsetY = 0;

        private Point prevMousePos;

        int cellSize;

        Grid[,] grid;
        EnumMode mode = EnumMode.SETTING;

        bool bDrag = false;
        bool bErase = false;
        bool bEnd = false;
        PathFinder pathFinder = new PathFinder();

        Queue<(PathFindNode, EnumColor)> naviQ = new(3000); // y, x, color

        System.Windows.Forms.Timer naviTimer = new System.Windows.Forms.Timer();

        const int defaultFontSize = 12;
        Font font = new Font("Calibri", defaultFontSize);
        SolidBrush textBrush = new SolidBrush(Color.Black);

        bool isBtn1Clicked = false;
        bool isMoving = false;

        public Form1()
        {
            InitializeComponent();

            this.SetStyle(ControlStyles.OptimizedDoubleBuffer, true);
            this.SetStyle(ControlStyles.AllPaintingInWmPaint, true);
            this.SetStyle(ControlStyles.UserPaint, true);

            grid = new Grid[GridStandard.height, GridStandard.width];

            for (int i = 0; i < GridStandard.height; i++)
            {
                for (int j = 0; j < GridStandard.width; j++)
                {
                    grid[i, j] = new Grid();
                }
            }

            int big = Width > Height ? Width : Height;
            int small = Width < Height ? Width : Height;

            double changedMul = (double)big / small;
            cellSize = (int)(GridStandard.cellSize / changedMul);


            naviTimer.Tick += new EventHandler(Navigate);
            naviTimer.Interval = 10;

            MouseWheel += new MouseEventHandler(MouseWheelEvent);
        }
        private void MouseWheelEvent(object? sender, MouseEventArgs e)
        {
            var scrollValue = e.Delta * SystemInformation.MouseWheelScrollLines / 120; // 3 or -3

            cellSize += scrollValue;

            if (cellSize < 10)
                cellSize = 10;
            else if (cellSize > 100)
                cellSize = 100;

            Invalidate();

            // TODO
        }

        private void Navigate(object? sender, EventArgs e)
        {
            naviQ.Clear();
            bool res = pathFinder.Navigate(naviQ);

            textBox1.Text = naviQ.Count.ToString();

            while (naviQ.Count > 0)
            {
                var vTuple = naviQ.Dequeue();

                // == gridColor[yPos, xPos] = color;
                grid[vTuple.Item1.yPos, vTuple.Item1.xPos].Color = vTuple.Item2;
                grid[vTuple.Item1.yPos, vTuple.Item1.xPos].pathFindNode = vTuple.Item1;
            }

            if (res) // if successed return true
            {
                bEnd = true;
                naviTimer.Stop();
            }

            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;

            // 그리드의 각 셀을 그리기

            PathFindNode? pathFindNode;
            for (int y = 0; y < GridStandard.height; y++)
            {
                for (int x = 0; x < GridStandard.width; x++)
                {
                    pathFindNode = grid[y, x].PathFindNode;

                    // 셀의 상태에 따라 색상 결정
                    Color color = Color.White;
                    switch (grid[y, x].Color)
                    {
                        case EnumColor.NO_USE: break;
                        case EnumColor.OBSTACLE: color = Color.Gray; break;
                        case EnumColor.START_POINT: color = Color.Red; break;
                        case EnumColor.END_POINT: color = Color.Magenta; break;
                        case EnumColor.INTENDED: color = Color.Orange; break;
                        case EnumColor.END_SEARCH: color = Color.LightGray; break;
                        case EnumColor.PATH: color = Color.Yellow; break;
                        case EnumColor.PATH_NODE: color = Color.ForestGreen; break;
                        default: break;
                    }

                    int rectX = GridStandard.buttonXPaddingSize + x * cellSize - mouseOffsetX;
                    int rectY = y * cellSize - mouseOffsetY;
                    int rectWidth = cellSize - 1;
                    int rectHeight = cellSize - 1;

                    g.FillRectangle(new SolidBrush(color), rectX, rectY, rectWidth, rectHeight);
                    g.DrawRectangle(Pens.Black, rectX, rectY, rectWidth, rectHeight);
                    //g.FillRectangle(new SolidBrush(color), GridStandard.buttonXPaddingSize + x * cellSize, y * cellSize, cellSize - 1, cellSize - 1);
                    //g.DrawRectangle(Pens.Black, GridStandard.buttonXPaddingSize + x * cellSize, y * cellSize, cellSize - 1, cellSize - 1);

                    if (pathFindNode != null)
                    {
                        StringBuilder strBuilder = new StringBuilder();
                        strBuilder.AppendFormat("pos : {0},{1}\n", pathFindNode.yPos, pathFindNode.xPos);
                        strBuilder.AppendFormat("g : {0}\n", pathFindNode.heuristic_g);
                        strBuilder.AppendFormat("h : {0}\n", pathFindNode.heuristic_h);
                        strBuilder.AppendFormat("f : {0}\n", pathFindNode.heuristic_f);

                        string text = strBuilder.ToString();

                        //SizeF textSize = g.MeasureString(text, font);

                        e.Graphics.DrawString(text, font, textBrush, rectX, rectY);
                    }
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
            cellSize = (int)(GridStandard.cellSize / changedMul);

            float fontSize = defaultFontSize / (float)changedMul;

            font = new Font("Calibri", fontSize);

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

            if (isBtn1Clicked)
            {
                isMoving = true;
                prevMousePos = e.Location;
            }
            else
            {
                int xPos = ((e.Location.X - 200) + mouseOffsetX )/ cellSize;
                int yPos = ((e.Location.Y + mouseOffsetY)/ cellSize); ;

                if (!GridStandard.IsRightPos(xPos, yPos))
                    return;

                if (mode == EnumMode.SETTING)
                {
                    if (e.Button == MouseButtons.Left)
                    {
                        if (!pathFinder.StartNode.isUsable && grid[yPos, xPos].Color == EnumColor.NO_USE)
                        {
                            pathFinder.StartNode.xPos = xPos;
                            pathFinder.StartNode.yPos = yPos;
                            pathFinder.StartNode.isUsable = true;

                            grid[yPos, xPos].Color = EnumColor.START_POINT;
                        }
                        else if (pathFinder.StartNode.isUsable && grid[yPos, xPos].Color == EnumColor.START_POINT)
                        {
                            pathFinder.StartNode.isUsable = false;
                            grid[yPos, xPos].Color = EnumColor.NO_USE;
                        }
                    }

                    else if (e.Button == MouseButtons.Right)
                    {
                        if (!pathFinder.EndNode.isUsable && grid[yPos, xPos].Color == EnumColor.NO_USE)
                        {
                            pathFinder.EndNode.xPos = xPos;
                            pathFinder.EndNode.yPos = yPos;
                            pathFinder.EndNode.isUsable = true;
                            grid[yPos, xPos].Color = EnumColor.END_POINT;
                        }
                        else if (pathFinder.EndNode.isUsable && grid[yPos, xPos].Color == EnumColor.END_POINT)
                        {
                            pathFinder.EndNode.isUsable = false;
                            grid[yPos, xPos].Color = EnumColor.NO_USE;
                        }
                    }
                }
                else // Blocking
                {
                    if (e.Button == MouseButtons.Left)
                    {
                        bDrag = true;
                        bErase = grid[yPos, xPos].Color != EnumColor.NO_USE;

                        if (!GridStandard.IsRightPos(xPos, yPos))
                            return;

                        if (grid[yPos, xPos].Color != EnumColor.OBSTACLE && grid[yPos, xPos].Color != EnumColor.NO_USE)
                            return;

                        if (!bErase)
                        {
                            grid[yPos, xPos].Color = EnumColor.OBSTACLE;
                            pathFinder.InsertSetMember(xPos, yPos);
                        }
                        else
                        {
                            grid[yPos, xPos].Color = EnumColor.NO_USE;
                            pathFinder.DeleteSetMember(xPos, yPos);
                        }
                    }
                }
            }
            Invalidate();

        }

        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isBtn1Clicked)
            {
                if (mode != EnumMode.BLOCKING || !bDrag)
                    return;

                int xPos = ((e.Location.X - 200) + mouseOffsetX)/ cellSize;
                int yPos = ((e.Location.Y +mouseOffsetY)/ cellSize);

                if (!GridStandard.IsRightPos(xPos, yPos))
                    return;

                bool isObstacle = grid[yPos, xPos].Color == EnumColor.OBSTACLE;
                bool isNoUse = grid[yPos, xPos].Color == EnumColor.NO_USE;

                if (!isObstacle && !isNoUse)
                    return;

                if (!bErase)
                {
                    if (isObstacle)
                        return;

                    grid[yPos, xPos].Color = EnumColor.OBSTACLE;
                    pathFinder.InsertSetMember(xPos, yPos);
                }
                else
                {
                    if (isNoUse)
                        return;

                    grid[yPos, xPos].Color = EnumColor.NO_USE;
                    pathFinder.DeleteSetMember(xPos, yPos);
                }
            }
            else // Move
            {
                if (!isMoving)
                    return;

                mouseOffsetX += prevMousePos.X - e.X;
                mouseOffsetY += prevMousePos.Y - e.Y;

                prevMousePos = e.Location;
                
            }
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
            isMoving = false;
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
            {
                if (!bEnd && !naviTimer.Enabled && pathFinder.CanNavi())
                {
                    naviTimer.Start();
                }
            }
            // ESC, Delete
            else if (e.KeyCode == Keys.Escape)
            {
                if (naviTimer.Enabled)
                    naviTimer.Stop();

                pathFinder.Initialize();

                for (int i = 0; i < grid.GetLength(0); i++)
                {
                    for (int j = 0; j < grid.GetLength(1); j++)
                    {
                        grid[i, j].Color = EnumColor.NO_USE;
                        grid[i, j].PathFindNode = null;
                    }
                }

                mouseOffsetX = 0;
                mouseOffsetY = 0;

                bEnd = false;
                Invalidate();
            }
            else if(e.KeyCode == Keys.B)
            {
                button1_Click(sender, e);
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (isBtn1Clicked)
            {
                isBtn1Clicked = false;

                button1.Text = "Move Off";
            }
            else
            {
                isBtn1Clicked = true;

                button1.Text = "Move On";
            }
        }
    }

}
