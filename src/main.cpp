#include <mbed.h>
#include "controler.hpp"
#include "C620.hpp"
#include "PID_new.hpp"
#include "omuni.hpp"
#include "diff_angle_calc.hpp"
#include "pos_pid.hpp"
// dji::C620 c620(PA_11, PA_12);
// CAN arduino(PB_12, PB_13, (int)1e6);
dji::C620 c620(PB_12,PB_13);
CAN arduino(PA_11,PA_12, (int)1e6);
DigitalIn minipino_limit(D8,PullUp);

constexpr float goal_angle = 35;
constexpr int bl_max_angle = 8192;
constexpr int bl_gear_ratio = 36;
int goal = goal_angle / 360 * bl_max_angle * bl_gear_ratio;

PidGain gain_c610{1.0, 0.1, 0.0};
PidGain gain_c620{2.0, 0.5, 0.0};
constexpr int NUM_MOTORS = 8;
std::array<Pid, NUM_MOTORS> pids = {{
    Pid({gain_c610, -10000, 10000}),
    Pid({gain_c610, -10000, 10000}),
    Pid({gain_c610, -10000, 10000}),
    Pid({gain_c610, -10000, 10000}),
    Pid({gain_c620, -10000, 10000}),
    Pid({gain_c620, -10000, 10000}),
    Pid({gain_c620, -10000, 10000}),
    Pid({gain_c620, -10000, 10000})
}};

BufferedSerial pc(USBTX,USBRX,115200);
constexpr int canid_1 = 2;
constexpr int canid_2 = 4;
constexpr int sensor_id = 10;
constexpr int servo_id = 140;
constexpr int roller_canid = 200;
CANMessage msg;

Ps5 ps5;
int16_t meca_1[4]         = {0};
int16_t meca_2[4]         = {0};
int16_t omuni_rpm_goal[4] = {0};
int16_t meca_rpm_goal[4]  = {0};
int16_t rpm_actual[8]     = {0};
int16_t pre_angle[8]      = {0};
int16_t angle_goal[8]     = {0};
int16_t angle_raw[8]      = {0};
int16_t angle_actual[8]   = {0};
int16_t dc_angle[4]       = {0};
uint8_t servo[8]          = {0}; 
uint8_t signal[8]         = {0};

constexpr int drive_const = 5000;//オムニドライブパワー
constexpr int turn_const = 3000;//オムニ旋回パワー
constexpr int mi_power = 10000;//巡る命の回収パワー
constexpr int mi_ud_power = 5000;//巡る命の昇降パワー
constexpr int minipino_power_1 = 6000;//ミニピーノ
constexpr int minipino_power_2 = 7000;//
constexpr int minipino_power_3 = 8000;//
constexpr int bc_power = 15000;//バッドカンパニーの押し出しパワー
constexpr int bc_ud_power = 5000;//バッドカンパニーの上下パワー
constexpr int arc_power = 5000;//有澤還れのパワー
constexpr int pochi_power = 16000;//ぽち(パチンコ)
constexpr int mi_servo_angle = 128;

int mi_out = 0;//      555   1個
uint8_t mi_servo_out = 0;//M2006 1
int mi_ud_out = 0;//   555   1
int minipino_out = 0;//M3508 1
int bc_out = 0;//      385   1
int rc_out = 0;//      385   1
int bc_ud_out = 0;//   385   1
int arc_out = 0;//     735   2
int pochi_out = 0;//   555   1
omuni_param param{drive_const,turn_const};
Omuni omuni({param});
bool robot_mode = 0;// 0:固定バケツ関連　1:その他
int  bc_mode = 0;
bool reload = 1;
int reload_time = 0;
bool mi_servo_mode = 0;
int main(){
    c620.read_data();
        for(int i = 0; i < 4; i++){
            omuni_rpm_goal[i] = 0;
        }
        // const int defo_angle = c620.get_angle(6);
    while(1){
        auto now = HighResClock::now();
        static auto pre = now;
        c620.read_data();
        if(arduino.read(msg) && msg.id == sensor_id){
            // for(int i = 0;i < 4;i ++){
            //     dc_angle[i] = msg.data[2*i + 1] << 8 | msg.data[2*i];
            //     printf("%d  ",dc_angle[i]);
            // }
            // for(int i = 0; i < 4; i++){
            //     uint16_t raw = ((uint16_t)msg.data[2*i + 1] << 8)| msg.data[2*i];
            //     dc_angle[i] = (int16_t)raw;
            //     printf("[%d] raw=%u angle=%d  ",i, raw, dc_angle[i]);
            // }
            // for(int i = 0;i < 8;i++){
            //     printf("%d   ",msg.data[i]);
            // }
            // printf("\n");
            // printf("\n");
        }
        if(ps5.read(arduino)){
            static bool pre_option = ps5.option;
            if (ps5.option && !pre_option)
            {
                robot_mode = !robot_mode;
            }
            pre_option = ps5.option;
            
            omuni.omuni_calc(ps5.lstick_x,ps5.lstick_y,ps5.rstick_x,omuni_rpm_goal);
            if(robot_mode){
                if(ps5.r1){
                    arc_out = -arc_power;
                }else if(ps5.r2 > 40){
                    arc_out = arc_power / 2;
                }else{
                    arc_out = 0;
                }
                if(ps5.cross){
                    signal[0] = 1;
                }else{
                    signal[0] = 0;
                }
                if(ps5.square){
                    pochi_out = pochi_power;
                }else{
                    pochi_out = 0;
                }
            }else{
                static bool pre_cross = ps5.cross;
                if (ps5.cross && !pre_cross)
                {
                    bc_mode ++;
                    if(bc_mode > 3){
                        bc_mode = 0;
                    }
                }
                pre_cross = ps5.cross;
                static bool pre_down = ps5.down;
                if (ps5.down && !pre_down)
                {
                    mi_servo_mode = !mi_servo_mode;
                }
                pre_down = ps5.down;
                if(ps5.r1){
                    bc_ud_out = -bc_ud_power;
                }else if(ps5.r2 > 40){
                    bc_ud_out = bc_ud_power;
                }else{
                    bc_ud_out = 0;
                }
                if(bc_mode == 1){
                    bc_out = 0;
                    rc_out = bc_power;
                }else if(bc_mode == 2){
                    bc_out = bc_power;
                    rc_out = bc_power;
                }else if(bc_mode == 3){
                    bc_out = -bc_power;
                    rc_out = 0;
                }else{
                    bc_out = 0;
                    rc_out = 0;
                }
                if(ps5.circle && !reload){
                    minipino_out = -minipino_power_1;
                    if(minipino_limit == 0){
                        minipino_out += minipino_power_1;
                        reload = 1;
                    }
                }else if(ps5.triangle && !reload){
                    minipino_out = -minipino_power_2;
                    if(minipino_limit == 0){
                        minipino_out += minipino_power_2;
                        reload = 1;
                    }
                }else if(ps5.square && !reload){
                    minipino_out = -minipino_power_3;
                    if(minipino_limit == 0){
                        minipino_out += minipino_power_3;
                        reload = 1;
                    }
                }else{
                    if(reload){
                        minipino_out = 2500;
                        reload_time ++ ;
                        if(reload_time > 50){
                            reload = 0;
                            reload_time = 0;
                            minipino_out = 0;
                        }
                    }else{
                        minipino_out = 0;
                    }
                }
                if(ps5.l1){
                    mi_ud_out = mi_ud_power;
                }else if(ps5.l2 > 40){
                    mi_ud_out = -mi_ud_power;
                }else{
                    mi_ud_out = 0;
                }
                if(mi_servo_mode){
                    mi_servo_out = mi_servo_angle;
                }else{
                    mi_servo_out = 0;
                }
                if(ps5.left){
                    mi_out = mi_power;
                }else if(ps5.right){
                    mi_out = -mi_power;
                }else{
                    mi_out = 0;
                }
            }
            for(int i = 0; i < 8; i++){
                rpm_actual[i] = c620.get_rpm(i + 1);
                angle_raw[i] = c620.get_angle(i + 1);
                angle_actual[i] += diff_angle_calc(angle_raw[i],pre_angle[i]);
                pre_angle[i] = angle_raw[i];
            }
            meca_1[0] = mi_out;
            meca_1[1] = bc_out;
            meca_1[2] = bc_ud_out;
            meca_1[3] = rc_out;
            meca_2[0] = mi_ud_out;
            meca_2[1] = pochi_out;
            meca_2[2] = -arc_out;
            meca_2[3] = arc_out;
            meca_rpm_goal[0] = minipino_out;
            for(int i = 0;i < 8;i++){
                servo[i] = mi_servo_out;
            }
            // meca_rpm_goal[1] = mi_servo_out;
            if(now - pre > 10ms){
                // printf(">neo_angle:\n",angle_actual[4]);
                // printf(">neo_rpm:\n",rpm_actual[4]);
                printf("%d%d%d%d\n",servo[0],servo[1],servo[2],servo[3]);
                for(int i = 0; i < 4; i++){
                    c620.set_output(pids[i].calc(omuni_rpm_goal[i], rpm_actual[i], 0.01f), i + 1);
                }
                c620.set_output(pids[4].calc(meca_rpm_goal[0],rpm_actual[4],0.01f),5);
                // if(meca_rpm_goal[1] == 0){
                //     c620.set_output(static_cast<int16_t>(pos_pid_calc(0.04,0.69,meca_rpm_goal[1] - angle_actual[5],rpm_actual[5])),6);
                // }else if(meca_rpm_goal[1] != 0){
                //     c620.set_output(static_cast<int16_t>(pos_pid_calc(0.04,0.69,meca_rpm_goal[1] - angle_actual[5],rpm_actual[5])),6);
                // }
                CANMessage msg1(canid_1,(const uint8_t *)meca_1,8);
                CANMessage msg2(canid_2,(const uint8_t *)meca_2,8);
                CANMessage sub_signal(roller_canid,(const uint8_t *)signal,8);
                CANMessage servo_msg(servo_id,(const uint8_t *)servo,8);
                arduino.write(msg1);
                arduino.write(msg2);
                arduino.write(sub_signal);
                arduino.write(servo_msg);
                c620.write();
                pre = now;
            }
        }
    }
}