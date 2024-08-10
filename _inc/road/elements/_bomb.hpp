//
// Created by 一心向丞 on 2024/5/5.
//

#ifndef BOMB_HPP
#define BOMB_HPP
enum flag_bomb_e {
    BOMB_NONE = 0,
    BOMB_DETECTION,
    BOMB_FOUND_0,
    BOMB_FOUND,         //找到炸弹
    BOMB_FIRST_CONE,    //第一个锥桶
    BOMB_SECOND_CONE,    //第二个锥桶
    BOMB_BLOCK,         //黑块
};
extern flag_bomb_e flag_bomb;
extern int bomb_x;           //横坐标
extern int bomb_y;           //纵坐标
extern int bomb_block_x;           //黑块横坐标
extern int bomb_block_y;           //黑块纵坐标
void check_bomb();
void run_bomb();

#endif //BOMB_HPP
