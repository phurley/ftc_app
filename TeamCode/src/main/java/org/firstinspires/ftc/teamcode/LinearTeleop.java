package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Linear Teleop", group="Teleop")
public class LinearTeleop extends LinearOpMode {
    private DcMotor left1;
    private DcMotor right1;

    @Override
    public void runOpMode() throws InterruptedException {
        left1 = hardwareMap.dcMotor.get("Left1");
        right1 = hardwareMap.dcMotor.get("Right1");
        right1.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            left1.setPower(gamepad1.left_stick_y);
            right1.setPower(gamepad1.right_stick_y);
        }

        left1.setPower(0);
        right1.setPower(0);
    }
}
