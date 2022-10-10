package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public boolean bPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        //teleop loop
        while (opModeIsActive()) {
            //robot oriented drive method call
            if (rDrive)
                robot.robotODrive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

            //field oriented drive method call
            if (!rDrive)
                robot.fieldODrive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.right_stick_button);

            //drive mode toggles
            if (gamepad1.left_bumper)
                rDrive = true;
            if (gamepad1.right_bumper)
                rDrive = false;

            //slowmode setup
            if (gamepad1.b && !bPressed) {
                slowMode = !slowMode;
                bPressed = true;
            } else if (!gamepad1.b)
                bPressed = false;

            if (slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;

            /*wheelIntake.setPower(gamepad1.right_trigger);
            wheelIntake.setPower(-gamepad1.left_trigger);

            if (!transferOverride)
            {
                transferSlide.setPower(-gamepad1.right_trigger);
                transferSlide.setPower(gamepad1.left_trigger);
            }

            transfer();*/

            telemetry.addData("Slow mode", slowMode);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.update();
        }
    }
}
