//
// Created by 一心向丞 on 2024/5/5.
//

#ifndef BOMB_HPP
#define BOMB_HPP
enum flag_bomb_e {
    BOMB_NONE = 0,
    BOMB_DETECTION,
    BOMB_FOUND,

};
extern flag_bomb_e flag_bomb;

void check_bomb();
void run_bomb();

#endif //BOMB_HPP
