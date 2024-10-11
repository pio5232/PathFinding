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
        protected Dictionary<(int, int), PathFindNode> _openListDataSet = new(3000); // 검색 속도를 빠르게 하기 위해서 O(1) 테이블 사용.

        protected HashSet<(int, int)> _closedSet = new(3050); // (xPos, yPos) << 내가 (y,x)의 형태로 사용하는데, 여기는 반대로 저장함.

        protected Queue<(PathFindNode, EnumColor)> _colorQ;
        // CloseList 존재해야하지만, 현재 grid의 색상을 통해 CloseList로 검사하는 역할을 할 수 있음.

        public PathFinder(Queue<(PathFindNode, EnumColor)> colorQ) { _colorQ = colorQ; }
        public void InsertSetMember(int xPos, int yPos)
        {
            Debug.Assert(_closedSet.Contains((xPos, yPos)) == false);
            _closedSet.Add((xPos, yPos));
        }

        public void DeleteSetMember(int xPos, int yPos)
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
            _openListDataSet.Clear();

            _openList.Clear();
            _closedSet.Clear();

            _startNode.Initialize();
            _endNode.Initialize();

            InitializeEach();
        }

        protected abstract void InitializeEach();

        public abstract bool CanNavi(); // Check Before Start
        public abstract bool Navigate();


    }
    class AStarPathFinder : PathFinder
    {
        protected override void InitializeEach() { }
        public AStarPathFinder(Queue<(PathFindNode, EnumColor)> colorQ) : base(colorQ) { }
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
                //openList.Add(startNode);
                _openListDataSet.Add((_startNode.xPos, _startNode.yPos), _startNode);

                return true;
            }

            return false;
        }
        public override bool Navigate()
        {
             Debug.Assert(_openList.Count > 0);

            //openList.Sort((PathFindNode first, PathFindNode second) => {
            //    if(first.heuristic_f != second.heuristic_f)
            //    return first.heuristic_f - second.heuristic_f;

            //    return first.heuristic_h - second.heuristic_h;
            //});

            PathFindNode node = _openList.Dequeue();
            //openList[0];
            //openList.RemoveAt(0);

            // closeSet에 존재하지 않는데 openList에 있다 == 무조건 openListDataSet에 존재.
            _openListDataSet.Remove((node.xPos, node.yPos));
            _closedSet.Add((node.xPos, node.yPos));

            _colorQ.Enqueue((node, EnumColor.END_SEARCH));
            // 목적지 확인
            // 도착한 상태면 부모노드를 타고 가서 색을 바꿔준다.
            if (node.xPos == EndNode.xPos && node.yPos == EndNode.yPos)
            {
                // 출발지,목적지 제외 색바꿔야함
                while (node.parent_node != null)
                {
                    _colorQ.Enqueue((node, EnumColor.PATH));
                    node = node.parent_node;
                }

                _colorQ.Enqueue((_startNode, EnumColor.START_POINT));
                _colorQ.Enqueue((_endNode, EnumColor.END_POINT));

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


            if (_openListDataSet.ContainsKey((intendedPosX, intendedPosY)))
            {
                PathFindNode item = _openListDataSet[(intendedPosX, intendedPosY)];

                if (item.yPos == intendedPosY && item.xPos == intendedPosX)
                {
                    if (item.heuristic_g > tempG)
                    {
                        item.parent_node = parentNode;

                        item.heuristic_g = tempG;
                        item.heuristic_h = tempH;
                        item.heuristic_f = tempG + tempH;
                    }
                }
                return;
            }

            // 그것도 아니라면. 새로운 노드 생성 후 openList에 등록한다.
            PathFindNode newNode = new PathFindNode();
            newNode.parent_node = parentNode;
            newNode.xPos = intendedPosX;
            newNode.yPos = intendedPosY;

            newNode.heuristic_g = tempG;
            newNode.heuristic_h = tempH;
            newNode.heuristic_f = tempG + tempH;

            _openList.Enqueue(newNode, newNode.heuristic_f);
            _openListDataSet.Add((newNode.xPos, newNode.yPos), newNode);
            //openList.Add(newNode);

            _colorQ.Enqueue((newNode, EnumColor.INTENDED));
        }
    }

    class JPSPathFinder : PathFinder
    {
        private Queue<(int, int)> _pathQ;
        string s = new string("");


        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        static int EnumToIdx(int jpsEnum)
        {
            switch(jpsEnum)
            {
                case (int)EnumDir.JPS_UU: return (int)EnumDir.UU;
                case (int)EnumDir.JPS_RU: return (int)EnumDir.RU;
                case (int)EnumDir.JPS_RR: return (int)EnumDir.RR;
                case (int)EnumDir.JPS_RD: return (int)EnumDir.RD;
                case (int)EnumDir.JPS_DD: return (int)EnumDir.DD;
                case (int)EnumDir.JPS_LD: return (int)EnumDir.LD;
                case (int)EnumDir.JPS_LL: return (int)EnumDir.LL;
                case (int)EnumDir.JPS_LU: return (int)EnumDir.LU;
                default: return (int)EnumDir.JPS_NONE;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static EnumDir IdxToEnum(EnumDir jpsEnum)
        {
            switch (jpsEnum)
            {
                case EnumDir.UU: return EnumDir.JPS_UU;
                case EnumDir.RU: return EnumDir.JPS_RU;
                case EnumDir.RR: return EnumDir.JPS_RR;
                case EnumDir.RD: return EnumDir.JPS_RD;
                case EnumDir.DD: return EnumDir.JPS_DD;
                case EnumDir.LD: return EnumDir.JPS_LD;
                case EnumDir.LL: return EnumDir.JPS_LL;
                case EnumDir.LU: return EnumDir.JPS_LU;
                default: return EnumDir.JPS_NONE;
            }
        }
        static readonly Point[] jpsDirs = {
            new (0, -1), // ↑
            new (1, -1), // ↗
            new (1, 0 ), // →
            new (1,1 ), // ↘
            new (0, 1),// ↓
            new ( -1, 1),// ↙
            new ( -1, 0),// ←
            new ( -1,-1)// ↖
            }; // 그 방향에 대한 증감치


        // 코너를 발견하기 위해서 탐색해야하는 방향의 값을 저장
        // 방향당 2개씩 존재한다. 2*dir, 2*dir+1
        static readonly Point[] jpsSearchX =
        {
            // 시계 방향으로 계산
            // UU 기준 코너를 발견하기 위해서는
            // -1,0 -> -1,-1 이 왼쪽 코너, 1,0 -> 1,-1이 오른쪽 코너
            // 그렇기에 왼쪽 코너 / 오른쪽 코너 계산 식을 넣음.
            new (-1, -1), new(1, 1),  // ↑ 
            new (-1, -1), new(0, 1),// ↗
            new (0, 1), new(0,1),  // →
            new (0, 1), new(-1, -1),  // ↘
            new (1, 1), new(-1, -1),  // ↓
            new (1, 1), new(0, -1),// ↙
            new (0, -1), new(0, -1),// ←
            new (0,-1), new(1,1) // ↖
        };
        static readonly Point[] jpsSearchY =
        {
            // 시계 방향으로 계산/
            new (0, -1), new(0,-1),  // ↑
            new (0, -1), new(1, 1),  // ↗
            new (-1, -1),new(1, 1),  // →
            new (-1, -1), new(0, 1),  // ↘
            new (0, 1), new(0,1),  // ↓
            new (0, 1), new(-1, -1),// ↙
            new (1, 1), new(-1, -1),// ←
            new (1,1), new(0, -1) // ↖
        };
        private bool isEnd = false;
        class SearchInfo
        {
            public int parentJpsDir = (int)EnumDir.JPS_NONE;
            public int jpsDir = (int)EnumDir.JPS_NONE;
            public int startXPos = -1;
            public int startYPos = -1;
        }

        public JPSPathFinder(Queue<(PathFindNode, EnumColor)> colorQ, Queue<(int, int)> pathQ)
            : base(colorQ) { _pathQ = pathQ; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsEndPos(int x, int y)
        {
            bool ret = x == EndNode.xPos && y == EndNode.yPos;

            if (ret)
                isEnd = true;

            return ret;
        }
        protected override void InitializeEach()
        {
            isEnd = false;
        }
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

                //for(EnumDir idx = EnumDir.UU; idx < EnumDir.DIR_MAX; idx++)
                //{
                //    EnumDir jpsDir = IdxToEnum(idx);

                //    PathFindNode newNode = new ();
                //    newNode.parent_node = _startNode;

                //    newNode.xPos = _startNode.xPos + jpsDirs[(int)idx].X;
                //    newNode.yPos = _startNode.yPos + jpsDirs[(int)idx].Y;

                //    absY = Math.Abs(_endNode.yPos - newNode.yPos);
                //    absX = Math.Abs(_endNode.xPos - newNode.xPos);

                //    large = absX > absY ? absX : absY;
                //    small = absX < absY ? absX : absY;

                //    newNode.heuristic_h = (small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS)*(int)EnumDirWeight.H_WEIGHT;

                //    absY = Math.Abs(_startNode.yPos - newNode.yPos);
                //    absX = Math.Abs(_startNode.xPos - newNode.xPos);

                //    large = absX > absY ? absX : absY;
                //    small = absX < absY ? absX : absY;

                //    newNode.heuristic_g =  (small * (int)EnumDirWeight.DIAGONAL + (large - small) * (int)EnumDirWeight.CROSS)*(int)EnumDirWeight.G_WEIGHT;
                //    newNode.heuristic_f = newNode.heuristic_g + newNode.heuristic_h;

                //    newNode.jpsDir = jpsDir;
                //    _openList.Enqueue(newNode, newNode.heuristic_f);
                //    _openListDataSet.Add((newNode.xPos, newNode.yPos), newNode);

                //    _colorQ.Enqueue((newNode, EnumColor.INTENDED));
                //}
                _openList.Enqueue(_startNode, _startNode.heuristic_f);
                _openListDataSet.Add((_startNode.xPos, _startNode.yPos), _startNode);

                // 처음 시작할 때 8방향에 대해서 8개의 노드를 생성하도록 한다.

                //_closedSet.Add((_startNode.xPos, _startNode.yPos));

                //_colorQ.Enqueue((_startNode, EnumColor.END_SEARCH));

                return true;
            }
            return false;
        }
        public override bool Navigate()
        {
            if (isEnd)
            {
                PathFindNode pathNode = _endNode;
                while (pathNode.parent_node != null)
                {
                    _colorQ.Enqueue((pathNode, EnumColor.PATH));
                    pathNode = pathNode.parent_node;
                }

                _colorQ.Enqueue((_startNode, EnumColor.START_POINT));
                _colorQ.Enqueue((_endNode, EnumColor.END_POINT));

                return true;
            }

            Debug.Assert(_openList.Count > 0);

            PathFindNode node = _openList.Dequeue();

            _openListDataSet.Remove((node.xPos, node.yPos));

            _closedSet.Add((node.xPos, node.yPos));

            _colorQ.Enqueue((node, EnumColor.END_SEARCH));

            Search(node);

            return false;
        }

    void Search(PathFindNode node)
        {
            // Search란? 자신의 방향으로 탐색 (노드 생성이 가능하거나 벽에 막혀있으면 그 방향은 탐색 종료.)
            int jpsDir = (int)node.jpsDir;

            SearchInfo searchInfo = new ();
            for (int i = 0; i < (int)EnumDir.JPS_DIR_MAX; i++) // 8방향 처리.
            {
                // UU (0b_0000_0001) -> LU (0b_1000_0000)
                int iDir = (((int)EnumDir.JPS_UU) << i);                

                if ((jpsDir & iDir) != 0)
                {
                    searchInfo.parentJpsDir = (int)EnumDir.JPS_NONE;
                    searchInfo.jpsDir = iDir;
                    searchInfo.startXPos = node.xPos;
                    searchInfo.startYPos = node.yPos;

                    switch ((EnumDir)iDir)
                    {
                        case EnumDir.JPS_UU:
                        case EnumDir.JPS_RR:
                        case EnumDir.JPS_DD:
                        case EnumDir.JPS_LL:
                            SearchCross(node, searchInfo); break;
                        case EnumDir.JPS_RU:
                        case EnumDir.JPS_RD:
                        case EnumDir.JPS_LD:
                        case EnumDir.JPS_LU:
                            SearchDiagonal(node, searchInfo);break;
                    }
                }
            }
        }

        void SLog(SearchInfo searchInfo)
        {
            if (searchInfo.parentJpsDir != (int)EnumDir.JPS_NONE)
                s += " ";

            switch ((EnumDir)searchInfo.jpsDir)
            {
                case EnumDir.JPS_UU: s += string.Format("Up, 0b_0000_0001 [x : {0}, y : {1}]\n",searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_RR: s += string.Format("Right, 0b_0000_0100 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_DD: s += string.Format("Down, 0b_0001_0000 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_LL: s += string.Format("Left, 0b_0100_0000 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_RU: s += string.Format("Right Up, 0b_0000_0010 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_RD: s += string.Format("Right Down, 0b_0000_1000 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_LD: s += string.Format("Left Down, 0b_0010_0000 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                case EnumDir.JPS_LU: s += string.Format("Left Up, 0b_1000_0000 [x : {0}, y : {1}]\n", searchInfo.startXPos, searchInfo.startYPos); break;
                default: break;
            }
        }
        // 1.증감치를 알아야한다.
        // RR -> X++ / LL -> X-- / UU -> Y-- / DD -> Y++

        // 2. 코너 판단을 하기 위한 좌표 증감값을 알아야한다.
        // RR -> +-Y, X +1 / LL -> +-Y, X-1 / UU -> +-X, Y-1 / DD -> +-X, Y+1

        bool SearchCross(PathFindNode parentNode, SearchInfo searchInfo)
        {
            if (isEnd)
                return true;

            bool isCorner = false;

            int searchDir = EnumToIdx(searchInfo.jpsDir);
            int searchIdx = searchDir * 2;

            int curXPos = searchInfo.startXPos; // start Position
            int curYPos = searchInfo.startYPos;

            int extraClockWiseLeftDir = searchInfo.jpsDir >> 1; // EnumDir 참고. 시계 방향이기 때문에 구조상 Shift 1번이면 가능
            if ((extraClockWiseLeftDir & (int)EnumDir.JPS_ALL_DIR) == 0) // UU의 경우는 LU를 체크할 수가 없기 때문에 직접 할당.
                extraClockWiseLeftDir = (int)EnumDir.JPS_LU;

            while (true)
            {
                if (!GridStandard.IsRightPos(curXPos + jpsDirs[searchDir].X, curYPos + jpsDirs[searchDir].Y))
                    break;

                // TODO : LOG   
                SLog(searchInfo);

                if (IsEndPos(curXPos, curYPos))
                {
                    if (searchInfo.startXPos == parentNode.xPos && searchInfo.startYPos == parentNode.yPos)
                        RegistNode(parentNode, curXPos, curYPos, EnumDir.JPS_NONE);
                    else
                    {
                        int parentDir = (searchInfo.parentJpsDir == (int)EnumDir.JPS_LU) ?
                            (searchInfo.parentJpsDir >> 1 | (int)EnumDir.JPS_UU) : (searchInfo.parentJpsDir >> 1 | searchInfo.parentJpsDir << 1);

                        PathFindNode innerNode = RegistNode(parentNode, searchInfo.startXPos, searchInfo.startYPos, (EnumDir)parentDir);

                        RegistNode(innerNode, curXPos, curYPos, EnumDir.JPS_NONE);
                    }
                    return true;
                }

                // 시계 방향 기준 왼쪽
                if (IsClosed(curXPos + jpsSearchX[searchIdx].X, curYPos + jpsSearchY[searchIdx].X) && 
                    !IsClosed(curXPos + jpsSearchX[searchIdx].Y, curYPos + jpsSearchY[searchIdx].Y))
                {
                    isCorner = true;

                    searchInfo.jpsDir |= extraClockWiseLeftDir;
                }
                // 시계 방향 기준 오른쪽
                if (IsClosed(curXPos + jpsSearchX[searchIdx + 1].X, curYPos + jpsSearchY[searchIdx + 1].X)
                    && !IsClosed(curXPos + jpsSearchX[searchIdx + 1].Y, curYPos + jpsSearchY[searchIdx + 1].Y))
                {
                    isCorner = true;

                    searchInfo.jpsDir |= searchInfo.jpsDir << 1; // == extraClockWiseRightDir
                }

                if (isCorner == true)
                    break;

                _pathQ.Enqueue((curXPos, curYPos));

                curXPos += jpsDirs[searchDir].X;
                curYPos += jpsDirs[searchDir].Y;

                if (IsClosed(curXPos, curYPos))
                    break;
            }

            if (isCorner)
            {
                if (searchInfo.startXPos == parentNode.xPos && searchInfo.startYPos == parentNode.yPos)
                    RegistNode(parentNode, curXPos, curYPos, (EnumDir)searchInfo.jpsDir);
                else
                {
                    int parentDir = (searchInfo.parentJpsDir == (int)EnumDir.JPS_LU) ?
                        (searchInfo.parentJpsDir >> 1 | (int)EnumDir.JPS_UU) : (searchInfo.parentJpsDir >> 1 | searchInfo.parentJpsDir << 1);

                    PathFindNode innerNode = RegistNode(parentNode, searchInfo.startXPos, searchInfo.startYPos, (EnumDir)parentDir);

                    RegistNode(innerNode, curXPos, curYPos, (EnumDir)searchInfo.jpsDir);
                }
            }

            return false;
        }
        // ex) Right -> ClosedSet에 좌표 확인 -> 없다가 있으면. 노드 생성한다.
        // 막혀있으면 true, 뚫려있으면 false를 리턴. 헷갈릴까봐 적어둔다.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        bool IsClosed(int xPos, int yPos) // Valid Pos Check를 위해. 뚫려있는지 막혀있는지 확인한다.
        {
            return _closedSet.Contains((xPos, yPos));
        }

        void SearchDiagonal(PathFindNode parentNode, SearchInfo searchInfo)
        {
            if (isEnd)
                return;

            int curXPos = parentNode.xPos;
            int curYPos = parentNode.yPos;

            int searchDir = searchInfo.jpsDir;

            int dirIdx = EnumToIdx(searchInfo.jpsDir); // inputDir을 사용해도 상관x
            int searchIdx = dirIdx * 2;

            // 대각 방향은 기본 3가지 (min, ... , max)라고 칭한다.
            // 시계 방향 기준 왼쪽에 존재하는 기준은 min, 오른쪽에 존재하는 기준은 max이다.
            // 왼쪽 코너일 때는 min >> 1, 오른쪽 코너일 때는 max << 1을 해준다.

            int clockWiseLeftDir = searchInfo.jpsDir>> 1;
            int extraClockWiseLeftDir;// 시계 방향 기준 Left Extra Direction

            if (clockWiseLeftDir == (int)EnumDir.JPS_UU) // inputDir이 JPS_RU인 경우
            {
                extraClockWiseLeftDir = (int)EnumDir.JPS_LU;
            }
            else
                extraClockWiseLeftDir = clockWiseLeftDir >> 1;
            
            int clockWiseRightDir;
            if(searchInfo.jpsDir == (int)EnumDir.JPS_LU) // inputDir이 JPS_LU인 경우
            {
                clockWiseRightDir = (int)EnumDir.JPS_UU;
            }
            else 
            {
                clockWiseRightDir = searchInfo.jpsDir << 1;
            }
            int extraClockWiseRightDir = clockWiseRightDir << 1;
            searchDir |= clockWiseLeftDir | clockWiseRightDir;

            bool isCorner = false;

            SearchInfo childSearchInfo = new()
            {
                parentJpsDir = searchInfo.jpsDir,
                jpsDir = clockWiseLeftDir,
                startXPos = curXPos,
                startYPos = curYPos
            };

            while (true)
            {
                if (!GridStandard.IsRightPos(curXPos + jpsDirs[dirIdx].X, curYPos + jpsDirs[dirIdx].Y))
                    break;

                SLog(searchInfo);
                if (IsEndPos(curXPos, curYPos))
                {
                    RegistNode(parentNode, curXPos, curYPos, EnumDir.JPS_NONE);
                    return;
                }
                // 들어온 대각 기준 시계 방향 기준 좌측 / 우측 탐색

                childSearchInfo.jpsDir = clockWiseLeftDir;
                SearchCross(parentNode, childSearchInfo);

                childSearchInfo.jpsDir = clockWiseRightDir;
                SearchCross(parentNode, childSearchInfo);

                // RD Check
                if (IsClosed(curXPos + jpsSearchX[searchIdx].X, curYPos + jpsSearchY[searchIdx].X)
                    && !IsClosed(curXPos + jpsSearchX[searchIdx].Y, curYPos + jpsSearchY[searchIdx].Y))
                {
                    isCorner = true;

                    searchDir |= extraClockWiseLeftDir;
                }
                // LU Check
                if (IsClosed(curXPos + jpsSearchX[searchIdx + 1].X, curYPos + jpsSearchY[searchIdx + 1].X)
                    && !IsClosed(curXPos + jpsSearchX[searchIdx + 1].Y, curYPos + jpsSearchY[searchIdx +1].Y))
                {
                    isCorner = true;

                    searchDir |= extraClockWiseRightDir;
                }

                if (isCorner == true)
                    break;
                _pathQ.Enqueue((curXPos, curYPos));

                curXPos += jpsDirs[dirIdx].X;
                curYPos += jpsDirs[dirIdx].Y;

                childSearchInfo.startXPos = curXPos;
                childSearchInfo.startYPos = curYPos;

                if (IsClosed(curXPos, curYPos))
                    break;
            }

            if (isCorner)
            {
                RegistNode(parentNode, curXPos, curYPos, (EnumDir)searchDir);
            }
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

            if (_openListDataSet.ContainsKey((xPos, yPos)))
            {
                PathFindNode item = _openListDataSet[(xPos, yPos)];

                if (item.yPos == yPos && item.xPos == xPos)
                {
                    if (item.heuristic_g > tempG)
                    {
                        item.parent_node = parentNode;

                        item.heuristic_g = tempG;
                        item.heuristic_h = tempH;
                        item.heuristic_f = tempG + tempH;
                    }
                }
                return item;
            }

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
            _openListDataSet.Add((xPos, yPos), newNode);
            //openList.Add(newNode);

            _colorQ.Enqueue((newNode, EnumColor.INTENDED));

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
