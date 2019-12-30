package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.annotation.Documented;

@Autonomous(name = "Move Better")
public class Auto_MovementFix extends BaseAutonomous {

    @Override
    public void before_start() {
        before_start_blue();
    }

    @Override
    public void after_start() {

        okWeReallyGonnaDriveToPointNow(5,18,0,0,5);

    }

    double okWeReallyGonnaDriveToPointNow(double finishIn, double x, double y, double rot, double bufferS) {

        if (opModeIsActive()) {

            // convert in to m, deg to rad
            x /= 39.3701;
            y /= 39.3701;
            rot *= 0.01745;

            // vars for following eqs
            double max_RPM = 260;
            double max_achievable_velocity = (max_RPM / 60) * 2 * PI;
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

            l_count += l.getCurrentPosition();
            r_count += r.getCurrentPosition();
            bl_count += bl.getCurrentPosition();
            br_count += br.getCurrentPosition();

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

            double l_power = map(Wl, -max_achievable_velocity, max_achievable_velocity, -1, 1);
            double r_power = map(Wr, -max_achievable_velocity, max_achievable_velocity, -1, 1);
            double bl_power = map(Wbl, -max_achievable_velocity, max_achievable_velocity, -1, 1);
            double br_power = map(Wbr, -max_achievable_velocity, max_achievable_velocity, -1, 1);

            l.setTargetPosition((int) l_count);
            r.setTargetPosition((int) r_count);
            bl.setTargetPosition((int) bl_count);
            br.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(Math.abs(l_power));
            r.setPower(Math.abs(r_power));
            bl.setPower(Math.abs(bl_power));
            br.setPower(Math.abs(br_power));

            while   (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < finishIn + bufferS)) {

                telemetry.addLine("We goin");
                telemetry.addData("Powers", "Powers are %7f : %7f : %7f : %7f", l_power, r_power, bl_power, br_power);
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
