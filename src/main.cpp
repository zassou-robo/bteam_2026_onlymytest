#include <mbed.h>
#include "controler.hpp"
#include "C620.hpp"
#include "PID_new.hpp"

dji::C620 c620(PA_11, PA_12);
CAN arduino{PB_12, PB_13, (int)1e6};
PidGain gain_c610{1.0, 0.1, 0.0};
Pid pid_robomas_C610({gain_c610, -6000, 6000});


Ps5 ps5;
int16_t goal_drive[4] = {0};
int16_t goal_turn[4] = {0};
int16_t rpm_goal[8] = {0};
int16_t rpm_actual[8] = {0};
CANMessage msg;

int main(){
        for(int i = 0; i < 8; i++){
            rpm_goal[i] = 0;
        }
    while(1){
        auto now = HighResClock::now();
        static auto pre = now;
        c620.read_data();
        if(ps5.read(arduino)){
            if(ps5.lstick_y > 20 || ps5.lstick_y < -20){
                goal_drive[0] = -ps5.lstick_y * 20;
                goal_drive[1] = ps5.lstick_y * 20;
            }else{
                goal_drive[0] = 0;
                goal_drive[1] = 0;
            }
            if(ps5.rstick_x > 20 || ps5.rstick_x < -20){
                goal_turn[0] = -ps5.rstick_x * 10;
                goal_turn[1] = -ps5.rstick_x * 10;
            }else{
                goal_turn[0] = 0;
                goal_turn[1] = 0;
            }
            for(int i = 0; i < 2; i++){
                rpm_goal[i] = goal_drive[i] + goal_turn[i];
            }
            if(ps5.r1){//楕円タイヤ
                rpm_goal[2] = 4000;
            }else if(ps5.l1){
                rpm_goal[2] = -4000;
            }else{
                rpm_goal[2] = 0;
            }
            if(ps5.circle){
                rpm_goal[3] = 6000;
            }else{
                rpm_goal[3] = 0;
            }
            if(now - pre > 10ms){
                for(int i = 0; i < 8; i++){
                    rpm_actual[i] = c620.get_rpm(i + 1);
                }
                for(int i = 0; i < 4; i++){
                    c620.set_output(pid_robomas_C610.calc(rpm_goal[i], rpm_actual[i], 0.01f), i + 1);
                }
                c620.write();
                pre = now;
            }
        }
    }
}