package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Failsafe Test", group = "TeleOp")
public class FailSafeTesting extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        boolean podFailure = false;

        ElapsedTime matchTime = new ElapsedTime();

        RoadRunner robot = new RoadRunner(hardwareMap);

        robot.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        matchTime.reset();
        matchTime.startTime();

        while(opModeIsActive())
        {
            robot.robotODrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            podFailure = robot.failSafe(matchTime);

            if(podFailure)
                requestOpModeStop();

            telemetry.addData("left pod", robot.leftOdom.getCurrentPosition());
            telemetry.addData("right pod", robot.rightOdom.getCurrentPosition());
            telemetry.addData("center pod", robot.centerOdom.getCurrentPosition());
            telemetry.addData("reading 0", robot.leftReadings[0]);
            telemetry.addData("reading 1", robot.leftReadings[1]);
            telemetry.addData("reading 2", robot.leftReadings[2]);
            telemetry.addData("pod failed", podFailure);
            telemetry.addData("time", matchTime.seconds());
            telemetry.update();

            dashboardTelemetry.addData("left pod", robot.leftOdom.getCurrentPosition());
            dashboardTelemetry.addData("right pod", robot.rightOdom.getCurrentPosition());
            dashboardTelemetry.addData("center pod", robot.centerOdom.getCurrentPosition());
            dashboardTelemetry.addData("reading 0", robot.leftReadings[0]);
            dashboardTelemetry.addData("reading 1", robot.leftReadings[1]);
            dashboardTelemetry.addData("reading 2", robot.leftReadings[2]);
            dashboardTelemetry.addData("pod failed", podFailure);
            dashboardTelemetry.addData("time", matchTime.seconds());
            dashboardTelemetry.update();
        }
    }
}
