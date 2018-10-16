package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="BasicAuton", group="Auton")
public class BasicAuton extends LinearOpMode {
    protected DcMotor left1;
    protected DcMotor right1;

    protected void setPower(double left, double right) {
        left1.setPower(left);
        right1.setPower(right);
    }

    @Override
    public void runOpMode() {
        left1 = hardwareMap.dcMotor.get("Left1");
        right1 = hardwareMap.dcMotor.get("Right1");
        right1.setDirection(DcMotor.Direction.REVERSE);
        configure();

        waitForStart();
        runAuton();
    }

    protected void configure() { }

    protected void runAuton() {
        while (opModeIsActive()) {
            setPower(1,1);
            sleep(10);
        }
        setPower(0,0);
    }

}
