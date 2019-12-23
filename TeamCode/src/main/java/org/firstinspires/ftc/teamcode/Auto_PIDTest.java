package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name = "PID test")
public class Auto_PIDTest extends BaseAutonomous {

    ArrayList<DcMotor> motors = new ArrayList<>(4);
    ArrayList<DcMotor> leftMotors = new ArrayList<>(2);
    ArrayList<DcMotor> rightMotors = new ArrayList<>(2);
    ElapsedTime t = new ElapsedTime();
    private double KP = 0.2;
    private double KI = 0;
    private double KD = 0.005;

    @Override
    public void before_start() {
        before_start_blue();
        motors.add(l);
        motors.add(br);
        motors.add(r);
        motors.add(bl);
        leftMotors.add(l);
        leftMotors.add(bl);
        rightMotors.add(r);
        rightMotors.add(br);
    }

    @Override
    public void after_start() {

        pid_drive(0.4,0,4,0.001);
        pid_strafe(0.4,0,4,0.001);
        pid_strafe(-0.4,0,4,0.001);
        pid_drive(-0.4,0,4,0.001);

    }

    void pid_drive(double power, double angle, double sec, double dt) {

        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
        t.reset();
        double err = 0;
        double prev_err = 0;
        double integral = 0;
        double derivative = 0;
        double output = 0;
        while(t.seconds() < sec && opModeIsActive()) {
            err = getError(angle);
            err = map(err, -180, 180, -1, 1);
            integral = integral + (err * dt);
            derivative = (err - prev_err) / dt;
            output = (KP * err) + (KI * integral) + (KD * derivative);
            telemetry.addData("output",output);
            //output = map(output, -max_error, max_error, -adj, adj);
            //telemetry.addData("adjust_output",output);
            prev_err = err;

            telemetry.addData("p", err);
            telemetry.addData("i", integral);
            telemetry.addData("d", derivative);
            telemetry.update();

            for (DcMotor lm : leftMotors) {
                lm.setPower(lm.getPower() - output);
            }

            for (DcMotor rm : rightMotors) {
                rm.setPower(rm.getPower() + output);
            }

            sleep((long)(dt * 1000));

        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

    }

    void pid_strafe(double power, double angle, double sec, double dt) {

        l.setPower(-power);
        bl.setPower(power);
        r.setPower(power);
        br.setPower(-power);
        t.reset();
        double err = 0;
        double prev_err = 0;
        double integral = 0;
        double derivative = 0;
        double output = 0;
        while(t.seconds() < sec && opModeIsActive()) {
            err = getError(angle);
            err = map(err, -180, 180, -1, 1);
            integral = integral + (err * dt);
            derivative = (err - prev_err) / dt;
            output = (KP * err) + (KI * integral) + (KD * derivative);
            telemetry.addData("output",output);
            //output = map(output, -max_error, max_error, -adj, adj);
            //telemetry.addData("adjust_output",output);
            prev_err = err;

            telemetry.addData("p", err);
            telemetry.addData("i", integral);
            telemetry.addData("d", derivative);
            telemetry.update();

            for (DcMotor lm : leftMotors) {
                lm.setPower(lm.getPower() - output);
            }

            for (DcMotor rm : rightMotors) {
                rm.setPower(rm.getPower() + output);
            }

            sleep((long)(dt * 1000));

        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

    }


    double map(double x, double min_a, double max_a, double min_b, double max_b) {

        return (x-min_a)/(max_a-min_a) * (max_b-min_b) + min_b;

    }


}
