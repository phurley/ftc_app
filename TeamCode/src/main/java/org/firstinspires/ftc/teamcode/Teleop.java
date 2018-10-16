package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple Teleop", group="Teleop")
public class Teleop extends OpMode {
    private DcMotor left1;
    private DcMotor right1;

    @Override
    public void init() {
        left1 = hardwareMap.dcMotor.get("Left1");
        right1 = hardwareMap.dcMotor.get("Right1");
        right1.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        left1.setPower(gamepad1.left_stick_y);
        right1.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void stop() {
        left1.setPower(0);
        right1.setPower(0);
    }
}

