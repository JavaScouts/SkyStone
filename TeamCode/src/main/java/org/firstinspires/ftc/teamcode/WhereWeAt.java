package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.text.DecimalFormat;

@TeleOp(name = "Where are we, man?")
public class WhereWeAt extends LinearOpMode {

    DcMotorEx l;
    DcMotorEx r;
    DcMotorEx bl;
    DcMotorEx br;
    ElapsedTime e;
    static final double PI = Math.PI;
    private static DecimalFormat df = new DecimalFormat("0.0000");

    @Override
    public void runOpMode() {

        Hardware h = new Hardware();
        e = new ElapsedTime();
        h.init(hardwareMap, this);

        l = (DcMotorEx) h.leftDrive;
        r = (DcMotorEx) h.rightDrive;
        bl = (DcMotorEx) h.backLDrive;
        br = (DcMotorEx) h.backRDrive;

        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        double[] af = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
        String Px, Py, Pt = " ";
        e.reset();
        while(opModeIsActive()) {

            h.manualDrive(1.0);
            h.moveRobot();
            af = where(af);
            Px = df.format(af[5] * 39.37);
            Py = df.format(af[6] * 39.37);
            Pt = df.format(af[7] * 39.37);
            telemetry.addData("Positions", "X:[+"+Px+"]  Y:[+"+Py+"]  T:[+"+Pt+"]");
            telemetry.update();
        }
    }

    public double[] where(double[] prev) {

        // output format : [l, r, bl, br, time, x, y, t]
        double Vx, Vy, Vt;
        double PI = 3.1415;
        double ctr = PI / 537.6;
        double[] counts = new double[]{l.getCurrentPosition() * ctr, r.getCurrentPosition() * ctr, bl.getCurrentPosition() * ctr, br.getCurrentPosition() * ctr};
        double[] change = new double[]{0, 0, 0, 0};
        double[] out = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

        // add values to change
        for(int i = 0; i < counts.length; i++) {
            change[i] = counts[i] - prev[i];
        }

        // store current in output (return output)
        System.arraycopy(counts, 0, out, 0, counts.length);
        System.arraycopy(prev, 5, out, 5, 3);

        // calc and store dt
        double cur = e.nanoseconds();
        double prevt = prev[4];
        out[4] = cur;
        double dt = (cur - prevt) / 1000000000;
        telemetry.addData("DT",dt);

        // calculate wheel velocities and store in change
        for(int i = 0; i < change.length; i++) {
            change[i] /= dt;
        }

        Vx = (change[0] + change[1] + change[2] + change[3]) * (0.0375 / 4);
        Vy = (-change[0] + change[1] + change[2] - change[3]) * (0.0375 / 4);
        Vt = (-change[0] + change[1] - change[2] + change[3]) * (0.0375 / (4 * 0.3429));

        telemetry.addData("Velocities","X:[+"+df.format(Vx * 39.37)+"]  Y:[+"+df.format(Vy * 39.37)+"]  T:[+"+df.format(Vt * 39.37)+"]");
        //System.out.println("V: "+Vx+" | "+Vy+" | "+Vt+"");

        out[7] += dt * Vt;
        out[5] += dt * ((Vx * Math.cos(out[7])) - (Vy * Math.sin(out[7])));
        out[6] += dt * ((Vx * Math.sin(out[7])) + (Vy * Math.cos(out[7])));

        //System.out.println("P: "+out[5]+" | "+out[6]+" | "+out[7]+"");
        //System.out.println(out[0]+" "+out[1]+" "+out[2]+" "+out[3]+" "+out[4]+" "+out[5]+" "+out[6]+" "+out[7]+"");

        return out;

    }

}
