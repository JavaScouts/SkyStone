package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Place Block (high iq remix)")
public class Auto_AssistPlace extends BaseAutonomous {

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void before_start() {
        before_start_blue();
    }

    @Override
    public void after_start() {

        h.push_block_further_in_to_placer.setPosition(0);
        sleep(500);
        h.push_block_further_in_to_placer.setPosition(0.4);
        sleep(700);
        h.small.setPosition(0.7);
        sleep(700);
        h.push_block_further_in_to_placer.setPosition(0);
        sleep(700);
        place_block();

    }

    @Override
    void place_block() {
        int lmao = up_some();
        h.big.setPosition(0.6);
        sleep(700);
        h.small.setPosition(0.1);
        sleep(300);
        h.small.setPosition(0.7);
        h.big.setPosition(0.05);
        sleep(700);
        down_some(lmao);
    }

    int up_some() {

        time.reset();
        int start = Rev.getCurrentPosition();
        while(time.seconds() < 10 && opModeIsActive() && rn.getDistance(DistanceUnit.INCH) < 7) {
            Rev.setPower(-0.15);
        }
        Rev.setPower(-0.001);
        return start;

    }

    void down_some(int count) {

        Rev.setTargetPosition(count);
        Rev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rev.setPower(0.24);
        while(opModeIsActive() && Rev.isBusy()) {
            idle();
        }
        Rev.setPower(-0.001);

    }

}
