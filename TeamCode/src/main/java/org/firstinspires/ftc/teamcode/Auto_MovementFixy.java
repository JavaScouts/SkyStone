package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name = "Move Better W/ Vel")
public class Auto_MovementFixy extends BaseAutonomous {

    DcMotorEx l0;
    DcMotorEx r0;
    DcMotorEx bl0;
    DcMotorEx br0;

    @Override
    public void before_start() {
        before_start_blue();
    }

    @Override
    public void after_start() {

        l0 = (DcMotorEx) l;
        r0 = (DcMotorEx) r;
        bl0 = (DcMotorEx) bl;
        br0 = (DcMotorEx) br;
        okWeReallyGonnaDriveToPointNow(5,18,0,0,5);

    }

    double okWeReallyGonnaDriveToPointNow(double finishIn, double x, double y, double rot, double bufferS) {

        if (opModeIsActive()) {

            // convert in to m, deg to rad
            x /= 39.3701;
            y /= 39.3701;
            rot *= 0.01745;

            // vars for following eqs
            double max_achievable_velocity = 36; //TODO put in result from max_speed test of vel
            double CPR = 85.56;
            double OneOverR = 1 / 0.075/2;
            double LxPlusLy = 0.3429;
            double Vx = x * finishIn;
            double Vy = y * finishIn;
            double W = rot * finishIn;

            // calculate motor angular velocity ( rad / sec )
            double Wl = OneOverR * (Vx - Vy - (W * (LxPlusLy)));
            double Wr = OneOverR * (Vx + Vy + (W * (LxPlusLy)));
            double Wbl = OneOverR * (Vx + Vy - (W * (LxPlusLy)));
            double Wbr = OneOverR * (Vx - Vy + (W * (LxPlusLy)));

            // calculate position per motor
            double l_count = finishIn * Wl * CPR;
            double r_count = finishIn * Wr * CPR;
            double bl_count = finishIn * Wbl * CPR;
            double br_count = finishIn * Wbr * CPR;

            l_count += l0.getCurrentPosition();
            r_count += r0.getCurrentPosition();
            bl_count += bl0.getCurrentPosition();
            br_count += br0.getCurrentPosition();

            // calculate power per motor
            double maxf = Math.max(Math.abs(Wl), Math.abs(Wr));
            double maxb = Math.max(Math.abs(Wbl), Math.abs(Wbr));
            double max = Math.max(maxf, maxb);

            // normalize if unachievable
            if (max > max_achievable_velocity) {
                Wl = map(Wl, -max, max, -max_achievable_velocity, max_achievable_velocity);
                Wr = map(Wr, -max, max, -max_achievable_velocity, max_achievable_velocity);
                Wbl = map(Wbl, -max, max, -max_achievable_velocity, max_achievable_velocity);
                Wbr = map(Wbr, -max, max, -max_achievable_velocity, max_achievable_velocity);
                telemetry.addLine("This move is not feasible.");
                telemetry.addLine("We can't move fast enough.");
                telemetry.addLine("But we try anyway.");
                telemetry.update();
            }

            l0.setTargetPosition((int) l_count);
            r0.setTargetPosition((int) r_count);
            bl0.setTargetPosition((int) bl_count);
            br0.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l0.setVelocity(Wl, AngleUnit.RADIANS);
            r0.setVelocity(Wr, AngleUnit.RADIANS);
            br0.setVelocity(Wbr, AngleUnit.RADIANS);
            bl0.setVelocity(Wbl, AngleUnit.RADIANS);

            while   (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < finishIn + bufferS)) {


                telemetry.addLine("We goin");
                telemetry.addData("Powers", "Vel are %7f : %7f : %7f : %7f", Wl, Wr, Wbl, Wbr);
                telemetry.addData("Target","Targets are %7d : %7d : %7d : %7d",     (int) l_count, (int)r_count, (int)bl_count, (int)br_count);
                telemetry.addData("Timing", "Current: %7f / Target: %7f / ETA: %7f", runtime.seconds(), finishIn, finishIn - runtime.seconds());
                telemetry.update();

                if (closeEnough((int) l_count, (int) r_count, (int) bl_count, (int) br_count)) {
                    break;
                }

            }

            cancelMovement();

        }
        telemetry.addLine("FINISHED MOVE");
        telemetry.update();
        return 0;

    }


}
