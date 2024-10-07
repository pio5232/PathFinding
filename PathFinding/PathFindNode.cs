using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Numerics;
namespace PathFinding
{
    class PathFinder
    {
        
        PathFindNode startNode = new PathFindNode();
        PathFindNode endNode = new PathFindNode();

        public PathFindNode StartNode => startNode;
        public PathFindNode EndNode => endNode;
        PriorityQueue<PathFindNode, int> openList = new PriorityQueue<PathFindNode, int>(300, new HeuristicComparer());
        HashSet<(int, int)> closedSet = new (3050);
        // CloseList 존재해야하지만, 현재 grid의 색상을 통해 CloseList로 검사하는 역할을 할 수 있음.
        public void Insert(in PathFindNode node)
        {
            openList.Enqueue(node, node.heuristic_f);
        }

        public PathFindNode Pop()
        {
            return openList.Dequeue();
        }

        public bool IsEmpty()
        {
            return openList.Count == 0;
        }

        public void Initialize()
        {
            openList.Clear();

            startNode.Initialize();
            endNode.Initialize();
        }

        public bool CanNavi() // Check Before Start
        {
            if (!startNode.isUsable || !endNode.isUsable)
                return false;

            if (openList.Count == 0)
            {
                startNode.heuristic_h = Math.Abs(endNode.yPos - startNode.yPos) + Math.Abs(endNode.xPos - startNode.xPos) *(int)DirWeight.DEFAULT_WEIGHT;
                startNode.heuristic_f = startNode.heuristic_h;
                openList.Enqueue(startNode, startNode.heuristic_f);
            }

            return true;
        }
        public bool Navigate(Queue<ValueTuple<int, int, EnumColor>> colorQ)
        {
            PathFindNode node = openList.Dequeue();

            // List를 사용할 때 Regist에서 갈 곳보다 좋다면 덮어썼지만, PQ라서 이미 갔던 곳에 대한 정보가 뽑혀서 나올 수 있음
            // 그래서 그것에 대한 체크.
            if (closedSet.Contains((node.xPos, node.yPos)))
                return false;

            closedSet.Add((node.xPos, node.yPos));

            ValueTuple<int, int, EnumColor> vtPos = (node.yPos, node.xPos, EnumColor.END_SEARCH);
            colorQ.Enqueue(vtPos);
            // 목적지 확인
            // 도착한 상태면 부모노드를 타고 가서 색을 바꿔준다.
            if (node.xPos == endNode.xPos && node.yPos == endNode.yPos)
            {
                // 출발지,목적지 제외 색바꿔야함
                while (node.parent_node != null)
                {
                    vtPos.Item1 = node.yPos;
                    vtPos.Item2 = node.xPos;
                    vtPos.Item3 = EnumColor.PATH;

                    colorQ.Enqueue(vtPos);
                    node = node.parent_node;
                }
                ValueTuple<int, int, EnumColor> vtStart = (startNode.yPos, startNode.xPos, EnumColor.START_POINT);
                ValueTuple<int, int, EnumColor> vtEnd = (endNode.yPos, endNode.xPos, EnumColor.END_POINT);

                colorQ.Enqueue(vtStart);
                colorQ.Enqueue(vtEnd);

                return true;
            }

            for (EnumDir dir = EnumDir.RU; dir < EnumDir.DIR_MAX; dir++)
            {
                RegistNode(node, dir, colorQ);
            }
            return false;
        }

        void RegistNode(PathFindNode parentNode, EnumDir dir, Queue<ValueTuple<int, int, EnumColor>> colorQ)
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
            if (!Grid.IsRightPos(intendedPosX, intendedPosY))
                return;

            // 갔던 곳.
            if (closedSet.Contains((intendedPosX, intendedPosY)))
                return;

            PathFindNode newNode = new PathFindNode();
            newNode.parent_node = parentNode;
            newNode.xPos = intendedPosX;
            newNode.yPos = intendedPosY;

            // manhatan
            newNode.heuristic_g = parentNode.heuristic_g + ((int)dir % 2 == 0 ? (int)DirWeight.DIAGONAL : (int)DirWeight.CROSS);
            newNode.heuristic_h = Math.Abs(endNode.yPos - newNode.yPos) + Math.Abs(endNode.xPos - newNode.xPos) * (int)DirWeight.DEFAULT_WEIGHT;
            newNode.heuristic_f = newNode.heuristic_g + newNode.heuristic_h;

            openList.Enqueue(newNode, newNode.heuristic_f);
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
