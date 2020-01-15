package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Move Better W/ Vel")
public class WhereWeAt extends LinearOpMode {

    DcMotorEx l;
    DcMotorEx r;
    DcMotorEx bl;
    DcMotorEx br;
    ElapsedTime e;
    static final double PI = Math.PI;

    @Override
    public void runOpMode() {

        Hardware h = new Hardware();
        e = new ElapsedTime();
        h.init(hardwareMap, this);

        l = (DcMotorEx) h.leftDrive;
        r = (DcMotorEx) h.rightDrive;
        bl = (DcMotorEx) h.backLDrive;
        br = (DcMotorEx) h.backRDrive;

        waitForStart();

        double[] af = new double[]{0, 0, 0, 0, 0};
        e.reset();
        while(opModeIsActive()) {
            where(af, af[4]);
        }

    }

    public double[] where(double[] prev, double prevt) {

        double Vx, Vy, Vt;
        double ctr = (1 / 537.6) * (2 * PI);
        double[] counts = new double[]{l.getCurrentPosition() * ctr, r.getCurrentPosition() * ctr, bl.getCurrentPosition() * ctr, br.getCurrentPosition() * ctr};
        double[] change = new double[]{0, 0, 0, 0};
        double[] out = new double[]{0, 0, 0, 0, 0};

        for(int i = 0; i < counts.length; i++) {
            change[i] = counts[i] - prev[i];
        }
        for(int i = 0; i < counts.length; i++) {
            out[i] = counts[i];
        }

        // calc and store dt
        double cur = e.seconds();
        out[4] = cur;
        double dt = cur - prevt;

        Vx = (change[0] + change[1] + change[2] + change[3]) * (0.075 / 4);
        Vy = (-change[0] + change[1] + change[2] - change[3]) * (0.075 / 4);
        Vt = (-change[0] + change[1] - change[2] + change[3]) * (0.075 / (4 * 0.3429));


        return out;

    }

}
