package org.firstinspires.ftc.teamcode.flame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FlameTeleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public boolean b1Pressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FlameHardware flame = new FlameHardware(hardwareMap);
        telemetry.addLine("Init Done :)");
        telemetry.update();

        while(opModeIsActive())
        {
            //robot oriented drive method call
            if (rDrive)
                flame.robotODrive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

            //field oriented drive method call
            if (!rDrive)
                flame.fieldODrive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.right_stick_button);

            //drive mode toggles
            if (gamepad1.right_bumper)
                rDrive = true;
            if (gamepad1.left_bumper)
                rDrive = false;

            //slowmode setup
            if (gamepad1.b && !b1Pressed)
            {
                slowMode = !slowMode;
                b1Pressed = true;
            }
            else if (!gamepad1.b)
                b1Pressed = false;

            if (slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;
        }
    }
}
