#include <mbed.h>
#include "controler.hpp"
#include "C620.hpp"
#include "PID_new.hpp"
#include "omuni.hpp"
#include "diff_angle_calc.hpp"
#include "pos_pid.hpp"
// dji::C620 c620(PA_11, PA_12);
// CAN arduino{PB_12, PB_13, (int)1e6};
dji::C620 c620(PB_12,PB_13);
CAN arduino{PA_11,PA_12, (int)1e6};
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
uint8_t signal[8]         = {0};

constexpr int drive_const = 5000;//オムニドライブパワー
constexpr int turn_const = 3000;//オムニ旋回パワー
constexpr int mi_power = 10000;//巡る命の回収パワー
constexpr int mi_ud_power = 5000;//巡る命の昇降パワー
constexpr int minipino_power_1 = 11000;//ミニピーノ
constexpr int minipino_power_2 = 12000;//
constexpr int minipino_power_3 = 13000;//
constexpr int bc_power = 5000;//バッドカンパニーの押し出しパワー
constexpr int bc_ud_power = 5000;//バッドカンパニーの上下パワー
constexpr int arc_power = 15000;//有澤還れのパワー
constexpr int pochi_power = 16000;//ぽち(パチンコ)

int mi_out = 0;//      555   1個
int mi_servo_out = 0;//M2006 1
int mi_ud_out = 0;//   555   1
int minipino_out = 0;//M3508 1
int bc_out = 0;//      385   2
int bc_ud_out = 0;//   385   1
int arc_out = 0;//     735   2
int pochi_out = 0;//   555   1
omuni_param param{drive_const,turn_const};
Omuni omuni({param});
bool robot_mode = 0;// 0:固定バケツ関連　1:その他
int main(){
        for(int i = 0; i < 4; i++){
            omuni_rpm_goal[i] = 0;
        }
    while(1){
        auto now = HighResClock::now();
        static auto pre = now;
        c620.read_data();
        if(arduino.read(msg) && msg.id == sensor_id){
            for(int i = 0;i < 4;i ++){
                dc_angle[i] = msg.data[2*i + 1] << 8 | msg.data[2*i];
            }
        }
        if(ps5.read(arduino)){
            omuni.omuni_calc(ps5.lstick_x,ps5.lstick_y,ps5.rstick_x,omuni_rpm_goal);
            if(robot_mode){
                if(ps5.r1){
                    arc_out = arc_power;
                }else if(ps5.r2 > 30){
                    arc_out = -arc_power;
                }else{
                    arc_out = 0;
                }
                if(ps5.circle){
                    signal[0] = 1;
                }else{
                    signal[0] = 0;
                }
            }
            for(int i = 0; i < 8; i++){
                rpm_actual[i] = c620.get_rpm(i + 1);
                angle_raw[i] = c620.get_angle(i + 1);
                angle_actual[i] += diff_angle_calc(angle_raw[i],pre_angle[i]);
                pre_angle[i] = angle_actual[i];
            }
            if(now - pre > 10ms){
                for(int i = 0; i < 4; i++){
                    c620.set_output(pids[i].calc(omuni_rpm_goal[i], rpm_actual[i], 0.01f), i + 1);
                    c620.set_output(pids[i + 4].calc(meca_rpm_goal[i],rpm_actual[i],0.01f),i + 5);
                }
                CANMessage msg1(canid_1,(const uint8_t *)meca_1,8);
                CANMessage msg2(canid_2,(const uint8_t *)meca_2,8);
                CANMessage sub_signal(roller_canid,(const uint8_t *)signal,8);
                arduino.write(msg1);
                arduino.write(msg2);
                arduino.write(sub_signal);
                c620.write();
                pre = now;
            }
        }
    }
}