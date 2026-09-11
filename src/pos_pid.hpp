#ifndef POS_PID_HPP
#define POS_PID_HPP
//きむうぉ
//きむ魚

float  pos_pid_calc(float kp,float kd,int diff_pos,int rpm){
    float output = kp * diff_pos - kd * rpm;
    float limit = 10000;
    output = std::clamp(output, -limit, limit);
    return output;
}

#endif 