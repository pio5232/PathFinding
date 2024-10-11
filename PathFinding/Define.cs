using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace PathFinding
{
    static class GridStandard
    {
        public const int height = 50;
        public const int width = 60;
        public const int cellSize = 150;   // 각 셀의 크기
        public const int buttonXPaddingSize = 200;

        public static bool IsRightPos(int xPos, int yPos)
        {
            if (xPos < 0 || xPos >= GridStandard.width)
                return false;

            if (yPos < 0 || yPos >= GridStandard.height)
                return false;

            return true;
        }
    }

    enum PathFindType
    {
        ASTAR = 0,
        JPS = 1,
    }

    enum EnumDirWeight
    {
        CROSS = 10, // 상하좌우, manhattan
        DIAGONAL = 14, // 대각

        H_WEIGHT = 2, // 기본 이동은 10의 값을 가진다 라고 설정.
        G_WEIGHT = 1,
    }
    enum EnumMode
    {
        SETTING = 0,// 기본 모드
        BLOCKING, // 벽을 설치하는 모드.
        MODE_MAX = 2,
    };
    enum EnumColor
    {
        NO_USE = 0, // 0, 사용중이지 않음
        OBSTACLE,        // 1, 장애물 있음
        START_POINT,           // 2, 시작점
        END_POINT,             // 3, 종료점
        INTENDED,              // 4, 향할 곳, 예정된 곳 
        END_SEARCH,            // 5, 갔던 곳
        PATH,                  // 6, 경로
        PATH_NODE,                  // 7. 중간 경로 노드

        JPS_SEARCH_PATH, // 8. JPS에서 노드로 등록되진 않았지만, 이미 탐색한 길.
        COLOR_MAX,
    };

    // 1시부터 시계방향
    enum EnumDir
    {
        // ----- ASTAR & JPS_ARRAY-----
        UU = 0,      // ↑
        RU,      // ↗
        RR,      // →
        RD,      // ↘
        DD,      // ↓
        LD,      // ↙
        LL,      // ←
        LU,      // ↖
        DIR_MAX,     // 

        // ----- JPS -----

        JPS_UU = 0b_0000_0001, // ↑
        JPS_RU = 0b_0000_0010, // ↗
        JPS_RR = 0b_0000_0100, // →
        JPS_RD = 0b_0000_1000, // ↘
        JPS_DD = 0b_0001_0000, // ↓
        JPS_LD = 0b_0010_0000, // ↙
        JPS_LL = 0b_0100_0000, // ←
        JPS_LU = 0b_1000_0000, // ↖

        JPS_DIR_MAX = 8,
        JPS_ALL_DIR = 0b_1111_1111,
        JPS_NONE = 0b_0000_0000,
    };
    

    class Grid
    {
        public EnumColor enumColor = EnumColor.NO_USE;

        public PathFindNode? pathFindNode = null;

        public EnumColor Color
        {
            get => enumColor;
            set => enumColor = value; 
        }

        public PathFindNode? PathFindNode
        {
            get => pathFindNode;
            set => pathFindNode = value;
        }

            
    }

}
