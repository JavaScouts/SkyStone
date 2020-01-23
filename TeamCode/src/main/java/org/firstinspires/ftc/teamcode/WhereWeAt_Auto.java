package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.BaseAutonomous.map;

@Autonomous(name = "Where are we, man? Let's move!")
public class WhereWeAt_Auto extends LinearOpMode {

    DcMotorEx l;
    DcMotorEx r;
    DcMotorEx bl;
    DcMotorEx br;
    ElapsedTime e;
    static final double PI = Math.PI;
    double dt;
    private static DecimalFormat df = new DecimalFormat("0.0000");
    private static final double KP = 0.008;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private Hardware h;

    @Override
    public void runOpMode() {

        h = new Hardware();
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

            go(24, 0, 0, 4,3);
            go(-24, 0, 0, 4,3);
            go(24, 0, 0, 4,3);

        }
    }

    public void go(double x, double y, double t, double tolerable, double finishIn) {

        double[] a = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

        double Px, Py, Pt, err, prev_err1, prev_err2, prev_err3, integral1, integral2, integral3, derivative, output1, output2, output3;
        String SPx, SPy, SPt;
        boolean keep_going = true;
        boolean use_x_pid, use_y_pid, use_t_pid;
        use_x_pid = false;
        use_y_pid = false;
        use_t_pid = false;
        if(x != 0) {
            use_x_pid = true;
        }
        if(y != 0) {
            use_y_pid = true;
        }
        if(t != 0) {
            use_t_pid = true;
        }
        integral1 = 0;
        integral2 = 0;
        integral3 = 0;
        prev_err1 = 0;
        prev_err2 = 0;
        prev_err3 = 0;
        output1 = 0;
        output2 = 0;
        output3 = 0;

        while(keep_going) {

            keep_going = true;

            a = where(a);
            Px = a[5] * 39.37;
            Py = a[6] * 39.37;
            Pt = a[7] * 39.37;

            if(use_x_pid) {
                err = Px - x;
                integral1 = integral1 + (err * dt);
                derivative = (err - prev_err1) / dt;
                output1 = (KP * err) + (KI * integral1) + (KD * derivative);
                output1 = Range.clip(output1, -1, 1);
                telemetry.addData("output - x", output1);
                //output = map(output, -max_error, max_error, -adj, adj);
                //telemetry.addData("adjust_output",output);
                prev_err1 = err;
                if (close_enough(err, tolerable)) {
                    keep_going = false;
                }
            }

            if(use_y_pid) {
                err = Py -y;
                integral2 = integral2 + (err * dt);
                derivative = (err - prev_err2) / dt;
                output2 = (KP * err) + (KI * integral2) + (KD * derivative);
                output2 = Range.clip(output2, -1, 1);
                telemetry.addData("output - y", output2);
                //output = map(output, -max_error, max_error, -adj, adj);
                //telemetry.addData("adjust_output",output);
                prev_err2 = err;
                if (close_enough(err, tolerable)) {
                    keep_going = false;
                }
            }

            if(use_t_pid) {
                err = Pt - t;
                integral3 = integral3 + (err * dt);
                derivative = (err - prev_err3) / dt;
                output3 = (KP * err) + (KI * integral3) + (KD * derivative);
                output3 = Range.clip(output3, -1, 1);
                telemetry.addData("output - t", output3);
                //output = map(output, -max_error, max_error, -adj, adj);
                //telemetry.addData("adjust_output",output);
                prev_err3 = err;
                if (close_enough(err, tolerable)) {
                    keep_going = false;
                }
            }

            l.setPower(l.getPower()+(output1 - output2 - output3));
            r.setPower(r.getPower()+(output1 + output2 + output3));
            bl.setPower(bl.getPower()+(output1 + output2 - output3));
            br.setPower(br.getPower()+(output1 - output2 + output3));

            SPx = df.format(Px);
            SPy = df.format(Py);
            SPt = df.format(Pt);
            telemetry.addData("Positions", "X:[+"+SPx+"]  Y:[+"+SPy+"]  T:[+"+SPt+"]");
            telemetry.update();

        }

    }

    public boolean close_enough(double in, double tolerance_in) {

        return (in > -tolerance_in) && (in < tolerance_in);

    }

    public double[] where(double[] prev) {

        // output format : [l, r, bl, br, time, x, y, t]
        double Vx, Vy, Vt;
        double PI = 3.1415;
        double ctr = PI / 145.6;
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
        dt = (cur - prevt) / 1000000000;
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
