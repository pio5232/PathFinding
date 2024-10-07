﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinding
{
    static class Grid
    {
        public const int height = 50;
        public const int width = 60;
        public const int cellSize = 70;   // 각 셀의 크기
        public const int buttonXPaddingSize = 200;

        public static bool IsRightPos(int xPos, int yPos)
        {
            if (xPos < 0 || xPos >= Grid.width)
                return false;

            if (yPos < 0 || yPos >= Grid.height)
                return false;

            return true;
        }
    }
    enum DirWeight
    {
        CROSS = 10, // 상하좌우
        DIAGONAL = 14, // 대각
        
        DEFAULT_WEIGHT = 10, // 기본 이동은 10의 값을 가진다 라고 설정.
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
        COLOR_MAX,
    };

    // 1시부터 시계방향
    enum EnumDir
    {
        RU = 0, // ↗
        RR,      // →
        RD,      // ↘
        DD,      // ↓
        LD,      // ↙
        LL,      // ←
        LU,      // ↖
        UU,      // ↑
        DIR_MAX,     // 
    };

    enum eMode
    {
        SETTING = 0,// 기본 모드
        BLOCKING, // 벽을 설치하는 모드.
        MODE_MAX = 2,
    };
}
