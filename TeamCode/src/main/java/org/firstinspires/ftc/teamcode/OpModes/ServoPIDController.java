package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ServoPIDController {
    private double kp, kd, ki;

    private double lasterror = 0;
    private double integralsum = 0;
    ElapsedTime timer = null;

    public ServoPIDController(double kp, double kd, double ki){
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;

        timer = new ElapsedTime();
    }

    public void setPIDConstants(double kp, double kd, double ki){
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
    }

    public double calculate(double reference, double current){
        double error;
        error = reference - current;

        if(error > 180 ){
            error = error-360; // or error -= 360
        }
        else if (error < -180){
            error = error+360;
        }
        else{
            //do nothing!
        }

        double derivitave = (error - lasterror) / timer.seconds();
        integralsum += (error * timer.seconds());
        timer.reset();
        return (kp * error) + (ki * integralsum) + (kd * derivitave);
    }

}
