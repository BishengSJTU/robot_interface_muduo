#include "mapping_table.h"

void MappingTable(const int &cab_all_position_num, const int &position,
                  int &root, int &benchmark_position)
{
    if(cab_all_position_num == 26) {
        if (position >= 1 && position <= 7) {
            root = 1;
            benchmark_position = 4;
        } else if (position >= 8 && position <= 14) {
            root = 2;
            benchmark_position = 11;
        } else if (position >= 15 && position <= 21) {
            root = 3;
            benchmark_position = 18;
        } else if (position >= 22 && position <= 26) {
            root = 4;
            benchmark_position = 25;
        }
    }
    else if(cab_all_position_num == 32) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 10) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 11 && position <= 16) {
            root = 3;
            benchmark_position = 13;
        } else if (position >= 17 && position <= 22) {
            root = 4;
            benchmark_position = 19;
        } else if (position >= 23 && position <= 27) {
            root = 5;
            benchmark_position = 25;
        } else if (position >= 28 && position <= 32) {
            root = 6;
            benchmark_position = 30;
        }
    }
    else if(cab_all_position_num == 35) {
        if (position >= 1 && position <= 6) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 7 && position <= 12) {
            root = 2;
            benchmark_position = 9;
        } else if (position >= 13 && position <= 18) {
            root = 3;
            benchmark_position = 15;
        } else if (position >= 19 && position <= 24) {
            root = 4;
            benchmark_position = 21;
        } else if(position >= 25 && position <= 30) {
            root = 5;
            benchmark_position = 27;
        } else if(position >= 31 && position <= 35) {
            root = 6;
            benchmark_position = 33;
        }
    }
    else if(cab_all_position_num == 33) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 11) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 12 && position <= 17) {
            root = 3;
            benchmark_position = 14;
        } else if (position >= 18 && position <= 23) {
            root = 4;
            benchmark_position = 20;
        } else if(position >= 24 && position <= 28) {
            root = 5;
            benchmark_position = 26;
        } else if(position >= 29 && position <= 33) {
            root = 6;
            benchmark_position = 31;
        }
    }
}