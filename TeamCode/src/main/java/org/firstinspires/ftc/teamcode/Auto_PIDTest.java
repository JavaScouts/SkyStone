package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Auto_PIDTest extends BaseAutonomous {

    ArrayList<DcMotor> motors, leftMotors, rightMotors;

    @Override
    public void before_start() {
        before_start_blue();
        motors.add(l);
        motors.add(r);
        motors.add(bl);
        motors.add(br);

    }

    @Override
    public void after_start() {

    }

    void pid_drive(double power, double angle) {

        for (DcMotor motor : motors) {
            motor.setPower(power);
        }




    }


    double map(double x, double min_a, double max_a, double min_b, double max_b) {

        return (x-min_a)/(max_a-min_a) * (max_b-min_b) + min_b;

    }


}
