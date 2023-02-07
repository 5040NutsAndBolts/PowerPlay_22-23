package org.firstinspires.ftc.teamcode.flame;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Flame Teleop", group = "Teleop")
public class FlameTeleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public boolean b1Pressed = false;

    public boolean podFailure = false;

    ElapsedTime matchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        FlameOdo flame = new FlameOdo(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        FlameHardware.currentOpMode=this;
        flame.resetOdometry(0,0,0);

        telemetry.addLine("Init Done :)");
        telemetry.update();

        waitForStart();
        matchTime.reset();
        matchTime.startTime();

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

            flame.updatePositionRoadRunner();

            //podFailure = flame.failSafe(matchTime);

            if(podFailure)
                requestOpModeStop();

            /*telemetry.addData("left pod", flame.leftEncoderPos);
            telemetry.addData("right pod", flame.rightOdom.getCurrentPosition());
            telemetry.addData("center pod", flame.centerOdom.getCurrentPosition());
            telemetry.addData("reading 0", flame.leftReadings[0]);
            telemetry.addData("reading 1", flame.leftReadings[1]);
            telemetry.addData("reading 2", flame.leftReadings[2]);
            telemetry.addData("pod failed", podFailure);
            telemetry.addData("time", matchTime.seconds());
            telemetry.update();

            dashboardTelemetry.addData("left pos", flame.leftOdom.getCurrentPosition());
            dashboardTelemetry.addData("right pos", flame.rightOdom.getCurrentPosition());
            dashboardTelemetry.addData("center pos", flame.centerOdom.getCurrentPosition());
            dashboardTelemetry.addData("reading 0", flame.leftReadings[0]);
            dashboardTelemetry.addData("reading 1", flame.leftReadings[1]);
            dashboardTelemetry.addData("reading 2", flame.leftReadings[2]);
            dashboardTelemetry.addData("pod failed", podFailure);
            dashboardTelemetry.addData("time", matchTime.seconds());
            dashboardTelemetry.update();*/

            telemetry.addData("left red", flame.leftLineSensor.red());
            telemetry.addData("left blue", flame.leftLineSensor.blue());
            telemetry.addData("left green", flame.leftLineSensor.green());
            telemetry.addData("left luminosity", flame.leftLineSensor.alpha());
            telemetry.addData("left all", flame.leftLineSensor.argb());
            telemetry.addLine();
            telemetry.addData("center red", flame.centerLineSensor.red());
            telemetry.addData("center blue", flame.centerLineSensor.blue());
            telemetry.addData("center green", flame.centerLineSensor.green());
            telemetry.addData("center luminosity", flame.centerLineSensor.alpha());
            telemetry.addData("center all", flame.centerLineSensor.argb());
            telemetry.addLine();
            telemetry.addData("right red", flame.rightLineSensor.red());
            telemetry.addData("right blue", flame.rightLineSensor.blue());
            telemetry.addData("right green", flame.rightLineSensor.green());
            telemetry.addData("right luminosity", flame.rightLineSensor.alpha());
            telemetry.addData("right all", flame.rightLineSensor.argb());
            telemetry.addLine();
            telemetry.update();
        }
    }
}
