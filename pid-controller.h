#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#define uSec 1.0e6
#define DBL_MAX 999999999.0
#define DBL_MIN -DBL_MAX

#include <Arduino.h>

class PID {
    protected:
        double kp, ki, kd,
            position_error = 0, velocity_error = 0,
            max_intergral = 1.0, min_intergral = -1.0,
            period = 0.2, total_error = 0, prev_error = 0,
            position_tolerance = 0.05, velocity_tolerance = DBL_MAX,
            min_limit = -255.0, max_limit = 255.0,
            setpoint, input = 0, output = 0;
            long prev_time = 0;
        double clamp(double value, double low, double high);    
    public:
        PID(double kp, double ki, double kd);
        PID(double kp, double ki, double kd, double period);
        double calculate(double input, bool *run);
        double calculate(double input, double setpoint, bool *run);
        void set_tunings(double kp, double ki, double kd);
        void set_kp(double kp);
        void set_ki(double ki);
        void set_kd(double kd);
        void set_period(double period);
        void set_setpoint(double setpoint);
        void set_intergrator_range(double min_intergral, double max_intergral);
        void set_tolerance(double position_tolerance);
        void set_tolerance(double position_tolerance, double velocity_tolerance);
        void set_output_limits(double min_limit, double max_limit);
        bool at_setpoint();
        double get_kp();
        double get_ki();
        double get_kd();
        double get_period();
        double get_output();
        double get_setpoint();
        double get_position_error();
        double get_velocity_error();
        void reset();
};

#endif
