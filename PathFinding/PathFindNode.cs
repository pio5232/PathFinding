using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Numerics;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Xml.Linq;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TextBox;
using System.CodeDom.Compiler;
using System.Windows.Forms.Design;
namespace PathFinding
{

    abstract class PathFinder
    {
        protected PathFindNode _startNode = new PathFindNode();
        protected PathFindNode _endNode = new PathFindNode();
        public PathFindNode StartNode => _startNode;
        public PathFindNode EndNode => _endNode;

        protected PriorityQueue<PathFindNode, int> _openList = new PriorityQueue<PathFindNode, int>(300, new HeuristicComparer());
        //protected Dictionary<(int, int), PathFindNode> _openListDataSet = new(3000); // 검색 속도를 빠르게 하기 위해서 O(1) 테이블 사용.

        protected HashSet<(int, int)> _closedSet = new(3050); // (xPos, yPos) << 내가 (y,x)의 형태로 사용하는데, 여기는 반대로 저장함.

        protected Grid[,] _grid;

        public PathFinder(Grid[,] grid ) { _grid = grid; }
        public void InsertClosedSetMember(int xPos, int yPos)
        {
            Debug.Assert(_closedSet.Contains((xPos, yPos)) == false);
            _closedSet.Add((xPos, yPos));
        }

        public void DeleteClosedSetMember(int xPos, int yPos)
        {
            Debug.Assert(_closedSet.Contains((xPos, yPos)));
            _closedSet.Remove((xPos, yPos));
        }

        public bool IsEmpty()
        {
            return _openList.Count == 0;
        }

        public void Initialize()
        {
            //_openListDataSet.Clear();

            _openList.Clear();
            _closedSet.Clear();

            _startNode.Initialize();
            _endNode.Initialize();

            InitializeEach();
        }

        protected abstract void InitializeEach();

        public abstract bool CanNavi(); // Check Before Start
        public abstract bool Navigate();

        Queue<(int, int, EnumColor)> _logQueue = new(2000); // [y,x]의 형태로 저장
        protected void SetGrid(int x, int y, PathFindNode? node, EnumColor color)
        {
            if(color != EnumColor.JPS_SEARCH_PATH)
            _logQueue.Enqueue((y, x, color));
            
            _grid[y, x].enumColor = color;

            _grid[y, x].pathFindNode = node; 
        }

        protected bool IsGridBlocked(int x,int y)
        {
            return _grid[y, x].enumColor == EnumColor.OBSTACLE;
            // return _closedSet.Contains((x, y));
            //return _grid[y, x].enumColor != EnumColor.NO_USE;
        }
    }
    class AStarPathFinder : PathFinder
    {
        protected override void InitializeEach() { }
        public AStarPathFinder(Grid[,] grid) : base(grid) { }
        public override bool CanNavi() // Check Before Start
        {
            if (!_startNode.isUsable || !_endNode.isUsable)
                return false;

            if (_openList.Count == 0)
            {
                int absY = Math.Abs(_endNode.yPos - _startNode.yPos);
                int absX = Math.Abs(_endNode.xPos - _startNode.xPos);

                int large = absX > absY ? absX : absY;
                int small = absX < absY ? absX : absY;

                // large - small => 대각 횟수
                // large - small => Cross 횟수
                _startNode.heuristic_h = small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS;

                _startNode.heuristic_h *= (int)EnumDirWeight.H_WEIGHT;
                _startNode.heuristic_f = _startNode.heuristic_h;
                _openList.Enqueue(_startNode, _startNode.heuristic_f);

                return true;
            }

            return false;
        }
        public override bool Navigate()
        {
             Debug.Assert(_openList.Count > 0);

            PathFindNode node;
            do
            {
                node = _openList.Dequeue();
            }
            while (_closedSet.Contains((node.xPos, node.yPos)));
            
            _closedSet.Add((node.xPos, node.yPos));

            SetGrid(node.xPos, node.yPos, node, EnumColor.END_SEARCH);
            // 목적지 확인
            // 도착한 상태면 부모노드를 타고 가서 색을 바꿔준다.
            if (node.xPos == _endNode.xPos && node.yPos == _endNode.yPos)
            {
                // 출발지,목적지 제외 색바꿔야함
                while (node.parent_node != null)
                {
                    SetGrid(node.xPos,node.yPos, node, EnumColor.PATH);
                    node = node.parent_node;
                }

                SetGrid(_startNode.xPos, _startNode.yPos, _startNode, EnumColor.START_POINT);
                SetGrid(_endNode.xPos, _endNode.yPos, _endNode, EnumColor.END_POINT);

                return true;
            }

            for (EnumDir dir = EnumDir.UU; dir < EnumDir.DIR_MAX; dir++)
            {
                RegistNode(node, dir);
            }
            return false;
        }

        void RegistNode(PathFindNode parentNode, EnumDir dir)
        {
            int intendedPosX = parentNode.xPos;
            int intendedPosY = parentNode.yPos;

            switch (dir)
            {
                case EnumDir.UU: intendedPosY--; break;
                case EnumDir.RU: intendedPosY--; intendedPosX++; break;
                case EnumDir.RR: intendedPosX++; break;
                case EnumDir.RD: intendedPosY++; intendedPosX++; break;
                case EnumDir.DD: intendedPosY++; break;
                case EnumDir.LD: intendedPosY++; intendedPosX--; break;
                case EnumDir.LL: intendedPosX--; break;
                case EnumDir.LU: intendedPosY--; intendedPosX--; break;
                default: break;
            }

            // pos Check
            if (!GridStandard.IsRightPos(intendedPosX, intendedPosY))
                return;

            // 갔던 곳 또는 벽
            if (_closedSet.Contains((intendedPosX, intendedPosY)))
                return;

            // 중복 체크. openList에 등록이 되었지만 지금 경로가 더 좋다면 openList에 바꿔치기한다.
            int tempG = parentNode.heuristic_g + ((int)dir % 2 == 1 ? (int)EnumDirWeight.DIAGONAL : (int)EnumDirWeight.CROSS);

            int absX = Math.Abs(_endNode.xPos - intendedPosX);
            int absY = Math.Abs(_endNode.yPos - intendedPosY);
            int large = absX > absY ? absX : absY;
            int small = absX < absY ? absX : absY;

            int tempH = small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS;

            tempG *= (int)EnumDirWeight.G_WEIGHT;
            tempH *= (int)EnumDirWeight.H_WEIGHT;

            // 그것도 아니라면. 새로운 노드 생성 후 openList에 등록한다.
            PathFindNode newNode = new PathFindNode();
            newNode.parent_node = parentNode;
            newNode.xPos = intendedPosX;
            newNode.yPos = intendedPosY;

            newNode.heuristic_g = tempG;
            newNode.heuristic_h = tempH;
            newNode.heuristic_f = tempG + tempH;

            _openList.Enqueue(newNode, newNode.heuristic_f);

            SetGrid(newNode.xPos, newNode.yPos, newNode, EnumColor.INTENDED);
        }
    }

    class JPSPathFinder : PathFinder
    {
        public JPSPathFinder(Grid[,] grid)
            : base(grid) { }

        protected override void InitializeEach()
        {
            return;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool CanNavi() // Check Before Start
        {
            if (!_startNode.isUsable || !_endNode.isUsable)
                return false;

            if (_openList.Count == 0)
            {
                int absY = Math.Abs(_endNode.yPos - _startNode.yPos);
                int absX = Math.Abs(_endNode.xPos - _startNode.xPos);

                int large = absX > absY ? absX : absY;
                int small = absX < absY ? absX : absY;

                _startNode.heuristic_h = small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS;

                _startNode.heuristic_h *= (int)EnumDirWeight.H_WEIGHT;
                _startNode.heuristic_f = _startNode.heuristic_h;

                _startNode.jpsDir = EnumDir.JPS_ALL_DIR;

                _openList.Enqueue(_startNode, _startNode.heuristic_f);

                // 처음 시작할 때 8방향에 대해서 8개의 노드를 생성하도록 한다.
                
                // JPS에서는 맵 테두리에에 장애물을 설치한다.

                return true;
            }
            return false;
        }
        public override bool Navigate()
        {
             Debug.Assert(_openList.Count > 0);

            PathFindNode node;

            do
            {
                node = _openList.Dequeue();
            } while (_closedSet.Contains((node.xPos, node.yPos)));

            _closedSet.Add((node.xPos, node.yPos));
            
            SetGrid(node.xPos, node.yPos, node, EnumColor.END_SEARCH);
            
            return Search(node);

        }

        bool Search(PathFindNode node)
        {
            if(node.xPos == _endNode.xPos && node.yPos == _endNode.yPos)
            {
                while (node.parent_node != null)
                {
                    SetGrid(node.xPos, node.yPos, node, EnumColor.PATH);
                    node = node.parent_node;
                }

                SetGrid(_startNode.xPos, _startNode.yPos, _startNode, EnumColor.START_POINT);
                SetGrid(_endNode.xPos, _endNode.yPos, _endNode, EnumColor.END_POINT);

                return true;
            }
            // Search란? 자신의 방향으로 탐색 (노드 생성이 가능하거나 벽에 막혀있으면 그 방향은 탐색 종료.)
            int jpsDir = (int)node.jpsDir;

            for (int i = 0; i < (int)EnumDir.JPS_DIR_MAX; i++) // 8방향 처리.
            {
                // UU (0b_0000_0001) -> LU (0b_1000_0000)
                int iDir = (((int)EnumDir.JPS_UU) << i);

                if ((jpsDir & iDir) != 0)
                { 
                    // Search + 방향에 대해서는 EnumDir.JPS~~~ 방향을 리턴한다. 대각 방향에서 추갇
                    switch ((EnumDir)iDir)
                    {   
                            // 수평 수직
                        case EnumDir.JPS_UU: SearchUU(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_RR: SearchRR(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_DD: SearchDD(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_LL: SearchLL(node, node.xPos, node.yPos); break;
                            // 대각
                        case EnumDir.JPS_RU: SearchRU(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_RD: SearchRD(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_LD: SearchLD(node, node.xPos, node.yPos); break;
                        case EnumDir.JPS_LU: SearchLU(node, node.xPos, node.yPos); break;
                    }
                }
            }
            return false;
        }
        private void SearchLD(PathFindNode parent, int curXPos, int curYPos)
        {
            int direction = (int)EnumDir.JPS_LD | (int)EnumDir.JPS_LL | (int)EnumDir.JPS_DD;

            bool isCorner = false;

            SearchLL(parent, curXPos, curYPos);
            SearchDD(parent, curXPos, curYPos);

            while (true)
            {
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return;
                }

                curXPos -= 1;
                curYPos += 1;

                if (!GridStandard.IsRightPos(curXPos - 1, curYPos + 1) || IsGridBlocked(curXPos, curYPos))
                    break;
                if (SearchLL(null, curXPos, curYPos) || SearchDD(null, curXPos, curYPos))
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }


                // 오른쪽 코너 체크
                if (IsGridBlocked(curXPos +1, curYPos) && !IsGridBlocked(curXPos + 1, curYPos + 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_RD;
                }

                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos -1) && !IsGridBlocked(curXPos - 1, curYPos - 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_LU;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

                if (isCorner)
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }
            }
        }
        private void SearchRD(PathFindNode parent, int curXPos, int curYPos)
        {
            int direction = (int)EnumDir.JPS_RD | (int)EnumDir.JPS_RR | (int)EnumDir.JPS_DD;

            bool isCorner = false;

            SearchRR(parent, curXPos, curYPos);
            SearchDD(parent, curXPos, curYPos);

            while (true)
            {
                if(EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return;
                }

                curXPos += 1;
                curYPos += 1;

                if (!GridStandard.IsRightPos(curXPos + 1, curYPos + 1) || IsGridBlocked(curXPos, curYPos))
                    break;

                if (SearchRR(null, curXPos, curYPos) || SearchDD(null, curXPos, curYPos))
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }

                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos-1) && !IsGridBlocked(curXPos + 1, curYPos - 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_RU;
                }

                // 왼쪽 코너 체크
                if (IsGridBlocked(curXPos-1, curYPos) && !IsGridBlocked(curXPos - 1, curYPos + 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_LD;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

                if (isCorner)
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }
            }
        }
        private void SearchRU(PathFindNode parent, int curXPos, int curYPos)
        {
            int direction = (int)EnumDir.JPS_RU | (int)EnumDir.JPS_RR | (int)EnumDir.JPS_UU;
            
            bool isCorner = false;

            SearchRR(parent, curXPos, curYPos);
            SearchUU(parent, curXPos, curYPos);

            while (true)
            {
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent); 
                    return;
                }

                curXPos += 1;
                curYPos -= 1;

                if (!GridStandard.IsRightPos(curXPos + 1, curYPos - 1) || IsGridBlocked(curXPos, curYPos))
                    break;

                if (SearchRR(null, curXPos, curYPos) || SearchUU(null, curXPos, curYPos))
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }

                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos - 1, curYPos) && !IsGridBlocked(curXPos - 1, curYPos - 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_LU;
                }

                // 오른쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos + 1) && !IsGridBlocked(curXPos + 1, curYPos + 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_RD;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

                if (isCorner)
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }
            }
        }

        private void SearchLU(PathFindNode parent, int curXPos, int curYPos)
        {
            int direction = (int)EnumDir.JPS_LU | (int)EnumDir.JPS_LL | (int)EnumDir.JPS_UU;

            bool isCorner = false;

            SearchLL(parent, curXPos, curYPos);
            SearchUU(parent, curXPos, curYPos);

            while (true)
            {
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return;
                }

                curXPos -= 1;
                curYPos -= 1;
                if (!GridStandard.IsRightPos(curXPos - 1, curYPos - 1) || IsGridBlocked(curXPos, curYPos))
                    break;

                if (SearchLL(null, curXPos, curYPos) || SearchUU(null, curXPos, curYPos))
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }


                // 왼쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos + 1) && !IsGridBlocked(curXPos - 1, curYPos + 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_LD;
                }

                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos + 1, curYPos) && !IsGridBlocked(curXPos + 1, curYPos - 1))
                {
                    isCorner = true;
                    direction |= (int)EnumDir.JPS_RU;
                }

                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

                if (isCorner)
                {
                    RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
                    break;
                }
            }
        }
        private bool SearchLL(PathFindNode? parent, int curXPos, int curYPos)
        {
            // 수평, 수직에 대한 체크 로직은 현재 위치가 maxSize를 벗어나지 않거나, 막혀있지 않은 경우에 진행할 수 있도록 한다.
            int direction = (int)EnumDir.JPS_LL;

            bool isCorner = false;
            while(!isCorner)
            {
                curXPos -= 1; // 좌측 이동.

                // 코너를 탐색할 만큼의 공간이 없거나 막혀있다면 리턴.
                if (!GridStandard.IsRightPos(curXPos -1, curYPos) || IsGridBlocked(curXPos, curYPos))
                    break;

                // 목적지이면 그냥 대각에서 파생됐을 경우는 그냥 결과만 return, Cross 탐색중이라면 목적지를 오픈 리스트에 집어넣는다.
                if(EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return true;
                }
                
                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos - 1) && !IsGridBlocked(curXPos - 1, curYPos - 1))
                {
                    direction |= (int)EnumDir.JPS_LU;
                    isCorner = true;
                }

                // 아래쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos + 1) && !IsGridBlocked(curXPos - 1, curYPos + 1))
                {
                    direction |= (int)EnumDir.JPS_LD;
                    isCorner = true;
                }

                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);
            }

            if (isCorner && parent != null) // 대각 방향에서 파생된 것이 아니고. 방향이 Cross일 때.
                RegistNode(parent, curXPos, curYPos, (EnumDir)direction);

            return isCorner;
        }

        private bool SearchDD(PathFindNode? parent, int curXPos, int curYPos)
        {
            // 수평, 수직에 대한 체크 로직은 현재 위치가 maxSize를 벗어나지 않거나, 막혀있지 않은 경우에 진행할 수 있도록 한다.
            int direction = (int)EnumDir.JPS_DD;
            
            bool isCorner = false;
            while (!isCorner)
            {
                curYPos += 1; // 좌측 이동.

                // 코너를 탐색할 만큼의 공간이 없거나 막혀있다면 리턴.
                if (!GridStandard.IsRightPos(curXPos, curYPos+1) || IsGridBlocked(curXPos, curYPos))
                    break;
               
                // 목적지이면 그냥 대각에서 파생됐을 경우는 그냥 결과만 return, Cross 탐색중이라면 목적지를 오픈 리스트에 집어넣는다.
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return true;
                }

                // 왼쪽 코너 체크
                if (IsGridBlocked(curXPos-1, curYPos) && !IsGridBlocked(curXPos - 1, curYPos + 1))
                {
                    direction |= (int)EnumDir.JPS_LD;
                    isCorner = true;
                }

                // 오른쪽 코너 체크
                if (IsGridBlocked(curXPos+1, curYPos) && !IsGridBlocked(curXPos + 1, curYPos + 1))
                {
                    direction |= (int)EnumDir.JPS_RD;
                    isCorner = true;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

            }
            if (isCorner && parent != null) // 대각 방향에서 파생된 것이 아니고. 방향이 Cross일 때.
                RegistNode(parent, curXPos, curYPos, (EnumDir)direction);
            return isCorner;
        }

        private bool SearchRR(PathFindNode? parent, int curXPos, int curYPos)
        {
            int direction = (int)EnumDir.JPS_RR;
            bool isCorner = false;

            while (!isCorner)
            {
                curXPos += 1; // 좌측 이동.

                if (!GridStandard.IsRightPos(curXPos +1, curYPos) || IsGridBlocked(curXPos, curYPos))
                    break;
                
                // 목적지이면 그냥 대각에서 파생됐을 경우는 그냥 결과만 return, Cross 탐색중이라면 목적지를 오픈 리스트에 집어넣는다.
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return true;
                }

                // 위쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos - 1) && !IsGridBlocked(curXPos + 1, curYPos - 1))
                {
                    direction |= (int)EnumDir.JPS_RU;
                    isCorner = true;
                }

                // 아래쪽 코너 체크
                if (IsGridBlocked(curXPos, curYPos + 1) && !IsGridBlocked(curXPos + 1, curYPos + 1))
                {
                    direction |= (int)EnumDir.JPS_RD;
                    isCorner = true;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

            }
            if (isCorner && parent != null) // 대각 방향에서 파생된 것이 아니고. 방향이 Cross일 때.
                RegistNode(parent, curXPos, curYPos, (EnumDir)direction);

            return isCorner;
        }

        private bool SearchUU(PathFindNode? parent, int curXPos, int curYPos)
        {
            // 수평, 수직에 대한 체크 로직은 현재 위치가 maxSize를 벗어나지 않거나, 막혀있지 않은 경우에 진행할 수 있도록 한다.
            int direction = (int)EnumDir.JPS_UU;
            bool isCorner = false;

            while (!isCorner)
            {
                curYPos -= 1; // 좌측 이동.

                // 코너를 탐색할 만큼의 공간이 없거나 막혀있다면 리턴.
                if (!GridStandard.IsRightPos(curXPos, curYPos - 1) || IsGridBlocked(curXPos, curYPos))
                    break;

                // 목적지이면 그냥 대각에서 파생됐을 경우는 그냥 결과만 return, Cross 탐색중이라면 목적지를 오픈 리스트에 집어넣는다.
                if (EndNode.xPos == curXPos && EndNode.yPos == curYPos)
                {
                    RegistEndNode(parent);
                    return true;
                }

                // 왼쪽 코너 체크
                if (IsGridBlocked(curXPos - 1, curYPos) && !IsGridBlocked(curXPos - 1, curYPos - 1))
                {
                    direction |= (int)EnumDir.JPS_LU;
                    isCorner = true;
                }

                // 오른쪽 코너 체크
                if (IsGridBlocked(curXPos + 1, curYPos) && !IsGridBlocked(curXPos + 1, curYPos - 1))
                {
                    direction |= (int)EnumDir.JPS_RU;
                    isCorner = true;
                }
                SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

            }
            if (isCorner && parent != null) // 대각 방향에서 파생된 것이 아니고. 방향이 Cross일 때.
                RegistNode(parent, curXPos, curYPos, (EnumDir)direction);

            return isCorner;
        }

        // 1.증감치를 알아야한다.
        // RR -> X++ / LL -> X-- / UU -> Y-- / DD -> Y++

        // 2. 코너 판단을 하기 위한 좌표 증감값을 알아야한다.
        // RR -> +-Y, X +1 / LL -> +-Y, X-1 / UU -> +-X, Y-1 / DD -> +-X, Y+1

        //bool SearchCross(PathFindNode parentNode, SearchInfo searchInfo) // 부모 노드를 생성한 상태면 true / 부모 노드를 생성하지 않았으면 false
        //{
        //    if (isEnd)
        //        return true;

        //    bool isCorner = false;

        //    int searchDir = EnumToIdx(searchInfo.jpsDir);
        //    int searchIdx = searchDir * 2;

        //    int curXPos = searchInfo.startXPos; // start Position
        //    int curYPos = searchInfo.startYPos;

        //    bool isStartedFromParent = parentNode.xPos == curXPos && parentNode.yPos == curYPos;

        //    int extraClockWiseLeftDir = searchInfo.jpsDir >> 1; // EnumDir 참고. 시계 방향이기 때문에 구조상 Shift 1번이면 가능
        //    if ((extraClockWiseLeftDir & (int)EnumDir.JPS_ALL_DIR) == 0) // UU의 경우는 LU를 체크할 수가 없기 때문에 직접 할당.
        //        extraClockWiseLeftDir = (int)EnumDir.JPS_LU;

        //    SLog(searchInfo);
            
        //    while (true)
        //    {
        //        curXPos += jpsDirs[searchDir].X;
        //        curYPos += jpsDirs[searchDir].Y;
              
        //        if (!GridStandard.IsRightPos(curXPos, curYPos) || isGridBlocked(curXPos, curYPos))
        //            break;

        //        // TODO : LOG   

        //        if (IsEndPos(curXPos, curYPos))
        //        {
        //            if (isStartedFromParent)
        //                RegistNode(parentNode, curXPos, curYPos, EnumDir.JPS_NONE);
        //            else
        //            {
        //                int parentDir = (searchInfo.parentJpsDir == (int)EnumDir.JPS_LU) ?
        //                    (searchInfo.parentJpsDir >> 1 | (int)EnumDir.JPS_UU) : (searchInfo.parentJpsDir >> 1 | searchInfo.parentJpsDir << 1);

        //                PathFindNode innerNode = RegistNode(parentNode, searchInfo.startXPos, searchInfo.startYPos, (EnumDir)parentDir);

        //                RegistNode(innerNode, curXPos, curYPos, EnumDir.JPS_NONE);
        //            }
        //            return true;
        //        }

        //        // 시계 방향 기준 왼쪽
        //        if (isGridBlocked(curXPos + jpsSearchX[searchIdx].X, curYPos + jpsSearchY[searchIdx].X) &&
        //            !isGridBlocked(curXPos + jpsSearchX[searchIdx].Y, curYPos + jpsSearchY[searchIdx].Y))
        //        {
        //            isCorner = true;

        //            searchInfo.jpsDir |= extraClockWiseLeftDir;
        //        }
        //        // 시계 방향 기준 오른쪽
        //        if (isGridBlocked(curXPos + jpsSearchX[searchIdx + 1].X, curYPos + jpsSearchY[searchIdx + 1].X)
        //            && !isGridBlocked(curXPos + jpsSearchX[searchIdx + 1].Y, curYPos + jpsSearchY[searchIdx + 1].Y))
        //        {
        //            isCorner = true;

        //            searchInfo.jpsDir |= searchInfo.jpsDir << 1; // == extraClockWiseRightDir
        //        }

        //        if (isCorner == true)
        //            break;

        //        if(isStartedFromParent)
        //        SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);
        //    }

        //    if (isCorner)
        //    {
        //        // 부모 노드에서 뻗어나온 Cross
        //        if (isStartedFromParent)
        //            RegistNode(parentNode, curXPos, curYPos, (EnumDir)searchInfo.jpsDir);
        //        else // 대각선 방향으로 탐색하면서 뻗어나온 Cross
        //        {
        //            int parentDir = (searchInfo.parentJpsDir == (int)EnumDir.JPS_LU) ?
        //                (searchInfo.parentJpsDir >> 1 | (int)EnumDir.JPS_UU) : (searchInfo.parentJpsDir >> 1 | searchInfo.parentJpsDir << 1);

        //            RegistNode(parentNode, searchInfo.startXPos, searchInfo.startYPos, (EnumDir)parentDir);

        //            //RegistNode(innerNode, curXPos, curYPos, (EnumDir)searchInfo.jpsDir);
        //            return true;
        //        }
        //    }

        //    return false;
        //}
        // ex) Right -> ClosedSet에 좌표 확인 -> 없다가 있으면. 노드 생성한다.
        // 막혀있으면 true, 뚫려있으면 false를 리턴. 헷갈릴까봐 적어둔다.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        bool IsClosed(int xPos, int yPos) // Valid Pos Check를 위해. 뚫려있는지 막혀있는지 확인한다.
        {
            return _closedSet.Contains((xPos, yPos));
        }

        //void SearchDiagonal(PathFindNode parentNode, SearchInfo searchInfo)
        //{
        //    if (isEnd)
        //        return;

        //    int curXPos = parentNode.xPos;
        //    int curYPos = parentNode.yPos;

        //    int searchDir = searchInfo.jpsDir;

        //    int dirIdx = EnumToIdx(searchInfo.jpsDir); // inputDir을 사용해도 상관x
        //    int searchIdx = dirIdx * 2;

        //    // 대각 방향은 기본 3가지 (min, ... , max)라고 칭한다.
        //    // 시계 방향 기준 왼쪽에 존재하는 기준은 min, 오른쪽에 존재하는 기준은 max이다.
        //    // 왼쪽 코너일 때는 min >> 1, 오른쪽 코너일 때는 max << 1을 해준다.

        //    int clockWiseLeftDir = searchInfo.jpsDir >> 1;
        //    int extraClockWiseLeftDir;// 시계 방향 기준 Left Extra Direction

        //    if (clockWiseLeftDir == (int)EnumDir.JPS_UU) // inputDir이 JPS_RU인 경우
        //    {
        //        extraClockWiseLeftDir = (int)EnumDir.JPS_LU;
        //    }
        //    else
        //        extraClockWiseLeftDir = clockWiseLeftDir >> 1;

        //    int clockWiseRightDir;
        //    if (searchInfo.jpsDir == (int)EnumDir.JPS_LU) // inputDir이 JPS_LU인 경우
        //    {
        //        clockWiseRightDir = (int)EnumDir.JPS_UU;
        //    }
        //    else
        //    {
        //        clockWiseRightDir = searchInfo.jpsDir << 1;
        //    }
        //    int extraClockWiseRightDir = clockWiseRightDir << 1;
        //    searchDir |= clockWiseLeftDir | clockWiseRightDir;

        //    bool isCorner = false;

        //    SearchInfo childSearchInfo = new()
        //    {
        //        parentJpsDir = searchInfo.jpsDir,
        //        jpsDir = clockWiseLeftDir,
        //        startXPos = curXPos,
        //        startYPos = curYPos
        //    };

        //    SLog(searchInfo);
            
        //    while (true)
        //    {
        //        // 들어온 대각 기준 시계 방향 기준 좌측 / 우측 탐색

        //        childSearchInfo.jpsDir = clockWiseLeftDir;
        //        if (SearchCross(parentNode, childSearchInfo))
        //            break;

        //        childSearchInfo.jpsDir = clockWiseRightDir;
        //        if (SearchCross(parentNode, childSearchInfo))
        //            break;

        //        curXPos += jpsDirs[dirIdx].X;
        //        curYPos += jpsDirs[dirIdx].Y;

        //        childSearchInfo.startXPos = curXPos;
        //        childSearchInfo.startYPos = curYPos;
        //        if (!GridStandard.IsRightPos(curXPos , curYPos) || isGridBlocked(curXPos,curYPos))
        //            break;

        //        if (IsEndPos(curXPos, curYPos))
        //        {
        //            RegistNode(parentNode, curXPos, curYPos, EnumDir.JPS_NONE);
        //            return;
        //        }
        //        // RD Check
        //        if (isGridBlocked(curXPos + jpsSearchX[searchIdx].X, curYPos + jpsSearchY[searchIdx].X)
        //            && !isGridBlocked(curXPos + jpsSearchX[searchIdx].Y, curYPos + jpsSearchY[searchIdx].Y))
        //        {
        //            isCorner = true;

        //            searchDir |= extraClockWiseLeftDir;
        //        }
        //        // LU Check
        //        if (isGridBlocked(curXPos + jpsSearchX[searchIdx + 1].X, curYPos + jpsSearchY[searchIdx + 1].X)
        //            && !isGridBlocked(curXPos + jpsSearchX[searchIdx + 1].Y, curYPos + jpsSearchY[searchIdx + 1].Y))
        //        {
        //            isCorner = true;

        //            searchDir |= extraClockWiseRightDir;
        //        }

        //        if (isCorner == true)
        //            break;

        //        SetGrid(curXPos, curYPos, null, EnumColor.JPS_SEARCH_PATH);

        //    }

        //    if (isCorner)
        //    {
        //        RegistNode(parentNode, curXPos, curYPos, (EnumDir)searchDir);
        //    }
        //}

        void RegistEndNode(PathFindNode? parentNode)
        {
            if (parentNode == null)
                return;

            _endNode.parent_node = parentNode;
            
            _openList.Enqueue(_endNode, _endNode.heuristic_f);

            SetGrid(_endNode.xPos, _endNode.yPos, _endNode, EnumColor.INTENDED);
        }
        PathFindNode RegistNode(PathFindNode? parentNode, int xPos, int yPos, EnumDir dir)
        {
            int absX = Math.Abs(_startNode.xPos - xPos);
            int absY = Math.Abs(_startNode.yPos - yPos);
            int large = absX > absY ? absX : absY;
            int small = absX < absY ? absX : absY;
            int tempG = (small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS) * (int)EnumDirWeight.G_WEIGHT;

            absX = Math.Abs(_endNode.xPos - xPos);
            absY = Math.Abs(_endNode.yPos - yPos);
            large = absX > absY ? absX : absY;
            small = absX < absY ? absX : absY;
            int tempH = (small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS) * (int)EnumDirWeight.H_WEIGHT;

            PathFindNode newNode = new()
            {
                parent_node = parentNode,
                xPos = xPos,
                yPos = yPos,
                jpsDir = dir,

                heuristic_g = tempG,
                heuristic_h = tempH,
                heuristic_f = tempG + tempH
            };

            _openList.Enqueue(newNode, newNode.heuristic_f);

            SetGrid(newNode.xPos, newNode.yPos, newNode, EnumColor.INTENDED);

            return newNode;
        }

    }
    class PathFindNode
    {
        public PathFindNode? parent_node = null;
        public int xPos = -1;
        public int yPos = -1;

        public int heuristic_g = 0; // 출발점으로부터 이동 거리 ( 유클리드 방식 ) . 대각 존재 (루트 2)
        public int heuristic_h = 0; // 목적지까지의 거리 ( 맨하탄 ) . 가로 + 세로 ___|
        public int heuristic_f = 0;

        public bool isUsable = false; // start, end에만 해당

        public EnumDir jpsDir = EnumDir.JPS_NONE; //
        public void Initialize()
        {
            parent_node = null;
            xPos = -1;
            yPos = -1;
            heuristic_f = 0;
            heuristic_g = 0;
            heuristic_h = 0;

            isUsable = false;
        }
    }

    class HeuristicComparer : IComparer<int>
    {
        public int Compare(int x, int y)
        {
            
            return x - y;
        }
    }
}
