package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OpenLoopAuton", group="Auton")
public class OpenLoopAuton extends BasicAuton {
    static final double DRIVE_SPEED             = 0.6;
    static final double TURN_SPEED              = 0.5;

    @Override
    protected void runAuton() {
        openLoopDrive(6, DRIVE_SPEED, 1);
        openLoopRotate(3, TURN_SPEED, 0.7);
        openLoopDrive(6, DRIVE_SPEED, 1);
    }

    protected void openLoopDrive(double duration, double power, double rampTime) {
        ElapsedTime ramp = new ElapsedTime();

        while (ramp.seconds() < rampTime) {
            double rampPower = power * (ramp.seconds() / rampTime);
            setPower(rampPower, rampPower);
            sleep(100);
        }

        duration -= rampTime * 2;
        ElapsedTime elapsed = new ElapsedTime();
        while (elapsed.seconds() < duration) {
            setPower(power, power);
            sleep(100);
        }

        ramp.reset();
        while (ramp.seconds() < rampTime) {
            double rampPower = power * ((rampTime - ramp.seconds()) / rampTime);
            setPower(rampPower, rampPower);
            sleep(100);
        }
    }

    protected void openLoopRotate(double duration, double power, double rampTime) {
        ElapsedTime ramp = new ElapsedTime();

        while (ramp.seconds() < rampTime) {
            double rampPower = power * (ramp.seconds() / rampTime);
            setPower(rampPower, -rampPower);
            sleep(100);
        }

        duration -= rampTime * 2;
        ElapsedTime elapsed = new ElapsedTime();
        while (elapsed.seconds() < duration) {
            setPower(power, -power);
            sleep(100);
        }

        ramp.reset();
        while (ramp.seconds() < rampTime) {
            double rampPower = power * ((rampTime - ramp.seconds()) / rampTime);
            setPower(rampPower, -rampPower);
            sleep(100);
        }
    }
}
