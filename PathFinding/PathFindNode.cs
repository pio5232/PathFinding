using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Numerics;
using System.Diagnostics;
namespace PathFinding
{
    class PathFinder
    {
        PathFindNode startNode = new PathFindNode();
        PathFindNode endNode = new PathFindNode();
        public PathFindNode StartNode => startNode;
        public PathFindNode EndNode => endNode;

        //PriorityQueue<PathFindNode, int> openList = new PriorityQueue<PathFindNode, int>(300, new HeuristicComparer());
        //HashSet<(int, int)> openListDataSet = new(200); // 검색 속도를 빠르게 하기 위해서 O(1) 테이블 사용.
        Dictionary<(int, int), PathFindNode> openListDataSet = new(3000); // 검색 속도를 빠르게 하기 위해서 O(1) 테이블 사용.
        List<PathFindNode> openList = new List<PathFindNode>(3050);

        HashSet<(int, int)> closedSet = new (3050); // (xPos, yPos) << 내가 (y,x)의 형태로 사용하는데, 여기는 반대로 저장함.

        // CloseList 존재해야하지만, 현재 grid의 색상을 통해 CloseList로 검사하는 역할을 할 수 있음.

        public void InsertSetMember(int xPos, int yPos)
        {
            Debug.Assert(closedSet.Contains((xPos,yPos)) == false);
            closedSet.Add((xPos,yPos));
        }

        public void DeleteSetMember(int xPos, int yPos)
        {
            Debug.Assert(closedSet.Contains((xPos, yPos)));
            closedSet.Remove((xPos, yPos));
        }

        public bool IsEmpty()
        {
            return openList.Count == 0;
        }

        public void Initialize()
        {
            openListDataSet.Clear();

            openList.Clear();
            closedSet.Clear();

            startNode.Initialize();
            endNode.Initialize();
        }

        public bool CanNavi() // Check Before Start
        {
            if (!startNode.isUsable || !endNode.isUsable)
                return false;

            if (openList.Count == 0)
            {
                int absY = Math.Abs(endNode.yPos - startNode.yPos);
                int absX = Math.Abs(endNode.xPos - startNode.xPos);

                int large = absX > absY ? absX : absY;
                int small = absX < absY ? absX : absY;

                // large - small => 대각 횟수
                // large - small => Cross 횟수
                startNode.heuristic_h = small * (int)DirWeight.DIAGONAL + (large - small) * (int)DirWeight.CROSS;
                startNode.heuristic_f = startNode.heuristic_h;
                //openList.Enqueue(startNode, startNode.heuristic_f);
                openList.Add(startNode);
                openListDataSet.Add((startNode.xPos, startNode.yPos), startNode);
            }

            return true;
        }
        public bool Navigate(Queue<(PathFindNode, EnumColor)> colorQ)
        {
            Debug.Assert(openList.Count > 0);

            openList.Sort((PathFindNode first, PathFindNode second) => {
                if(first.heuristic_f != second.heuristic_f)
                return first.heuristic_f - second.heuristic_f;

                return first.heuristic_h - second.heuristic_h;
            });

            PathFindNode node = openList[0];
            openList.RemoveAt(0);

            // closeSet에 존재하지 않는데 openList에 있다 == 무조건 openListDataSet에 존재.
            openListDataSet.Remove((node.xPos, node.yPos));
            closedSet.Add((node.xPos, node.yPos));

            colorQ.Enqueue((node, EnumColor.END_SEARCH));
            // 목적지 확인
            // 도착한 상태면 부모노드를 타고 가서 색을 바꿔준다.
            if (node.xPos == endNode.xPos && node.yPos == endNode.yPos)
            {
                // 출발지,목적지 제외 색바꿔야함
                while (node.parent_node != null)
                {
                    colorQ.Enqueue((node, EnumColor.PATH));
                    node = node.parent_node;
                }
                
                colorQ.Enqueue((startNode,EnumColor.START_POINT));
                colorQ.Enqueue((endNode,EnumColor.END_POINT));

                return true;
            }

            for (EnumDir dir = EnumDir.RU; dir < EnumDir.DIR_MAX; dir++)
            {
                RegistNode(node, dir, colorQ);
            }
            return false;
        }

        void RegistNode(PathFindNode parentNode, EnumDir dir, Queue<(PathFindNode, EnumColor)> colorQ)
        {
            int intendedPosX = parentNode.xPos;
            int intendedPosY = parentNode.yPos;

            switch(dir)
            {
                case EnumDir.RU: intendedPosY--; intendedPosX++; break;
                case EnumDir.RR: intendedPosX++; break;
                case EnumDir.RD: intendedPosY++; intendedPosX++; break;
                case EnumDir.DD: intendedPosY++;  break;
                case EnumDir.LD: intendedPosY++; intendedPosX--; break;
                case EnumDir.LL: intendedPosX--;  break;
                case EnumDir.LU: intendedPosY--; intendedPosX--; break;
                case EnumDir.UU: intendedPosY--;  break;
                default:break;
            }

            // pos Check
            if (!GridStandard.IsRightPos(intendedPosX, intendedPosY))
                return;

            // 갔던 곳 또는 벽
            if (closedSet.Contains((intendedPosX, intendedPosY)))
                return;

            // 중복 체크. openList에 등록이 되었지만 지금 경로가 더 좋다면 openList에 바꿔치기한다.
            int tempG = parentNode.heuristic_g + ((int)dir % 2 == 0 ? (int)DirWeight.DIAGONAL : (int)DirWeight.CROSS);

            int absX = Math.Abs(endNode.xPos - intendedPosX);
            int absY = Math.Abs(endNode.yPos - intendedPosY);
            int large = absX > absY ? absX : absY;
            int small = absX < absY ? absX : absY;

            int tempH = small * (int)DirWeight.DIAGONAL + (large-small) * (int)DirWeight.CROSS ;
            int tempF = tempG + tempH;


            if (openListDataSet.ContainsKey((intendedPosX, intendedPosY)))
            {
                PathFindNode item = openListDataSet[(intendedPosX, intendedPosY)];
                
                if (item.yPos == intendedPosY && item.xPos == intendedPosX)
                {
                    if (item.heuristic_g > tempG)
                    {
                        item.parent_node = parentNode;

                        item.heuristic_g = tempG;
                        item.heuristic_h = tempH;
                        item.heuristic_f = tempF;
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
            newNode.heuristic_f = tempF;

            //openList.Enqueue(newNode, newNode.heuristic_f);
            openListDataSet.Add((newNode.xPos, newNode.yPos), newNode);
            openList.Add(newNode);

            colorQ.Enqueue((newNode, EnumColor.INTENDED));
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
