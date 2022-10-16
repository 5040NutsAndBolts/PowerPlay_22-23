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

    public boolean b1Pressed = false;
    public boolean y2Pressed = false;
    public boolean bumper2Pressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        //teleop loop
        while (opModeIsActive())
        {
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
            if (gamepad1.b && !b1Pressed)
            {
                slowMode = !slowMode;
                b1Pressed = true;
            } else if (!gamepad1.b)
                b1Pressed = false;

            if (slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;

            //wheel intake portion //uncomment when wheel intake added
            /*if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
            {
                robot.wheelIntake.setPower(gamepad2.right_trigger);
                robot.wheelIntake.setPower(-gamepad2.left_trigger);
            }
            else
            {
                robot.wheelIntake.setPower(gamepad1.right_trigger);
                robot.wheelIntake.setPower(-gamepad1.left_trigger);
            }*/

            //sets transfer override
            /*if(gamepad2.y) //uncomment when ready to set encoder positions
                robot.transferOverride = true;
            else
                robot.transferOverride = false;*/

            //transfer level setup
            if(gamepad2.right_bumper && !bumper2Pressed && robot.transferLevel < 2)
            {
                robot.transferLevel++;
                bumper2Pressed = true;
            }
            else if(gamepad2.left_bumper && !bumper2Pressed && robot.transferLevel > 0)
            {
                robot.transferLevel--;
                bumper2Pressed = true;
            }
            else if(!gamepad2.left_bumper && !gamepad2.right_bumper)
                bumper2Pressed = false;

            //transfer mech calls
            if (robot.transferOverride)
            {
                robot.transferSlide.setPower(-gamepad1.right_trigger);
                robot.transferSlide.setPower(gamepad1.left_trigger);
            }
            else
                robot.transfer();
            telemetry.addData("motorHeight", robot.transferSlide.getCurrentPosition());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.addLine();
            telemetry.addData("Transfer Level", robot.transferLevel);
            telemetry.update();

        }
    }
}
