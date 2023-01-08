package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.helperclasses.RoadRunner;
import org.firstinspires.ftc.teamcode.signalfinding.GreenFinder;
import org.firstinspires.ftc.teamcode.signalfinding.OrangeFinder;
import org.firstinspires.ftc.teamcode.signalfinding.PurpleFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue Left Auto", group = "Autonomous")
public class BlueSideLeft extends LinearOpMode
{
    int autoNumber = 1;

    boolean intakeCone, depositCone, counterSpin;

    Trajectory preloadGoal0, backAway1, loopSetup2, park1X, park2X, park3X;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException
    {
        //camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        RoadRunner robot = new RoadRunner(hardwareMap);

        //runs camera and associated programs
        while(!isStopRequested() && !isStarted())
        {
            double orangeScore = OrangeFinder.orangeScore;
            double greenScore = GreenFinder.greenScore;
            double purpleScore = PurpleFinder.purpleScore;

            webcam.setPipeline(new OrangeFinder());
            webcam.setPipeline(new GreenFinder());
            webcam.setPipeline(new PurpleFinder());

            if(greenScore >= orangeScore && greenScore >= purpleScore)
                autoNumber = 2;
            else if(purpleScore > orangeScore && purpleScore > greenScore)
                autoNumber = 3;
            else
                autoNumber = 1;

            dashboardTelemetry.addData("auto", autoNumber);
            dashboardTelemetry.update();

            telemetry.addData("auto", autoNumber);
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(64, -30, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        waitForStart();

        preloadGoal0 = robot.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    counterSpin = true;
                })
                .splineTo(new Vector2d(50, -14), Math.toRadians(180),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(28, -5), Math.toRadians(135),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    depositCone = true;
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(backAway1);
                })
                .build();

        backAway1 = robot.trajectoryBuilder(preloadGoal0.end(), Math.toRadians(135))
                .strafeTo(new Vector2d(35, -12),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(loopSetup2);
                })
                .build();

        loopSetup2 = robot.trajectoryBuilder(backAway1.end())
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .splineTo(new Vector2d(16, -16), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(6, -28), Math.toRadians(135),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    if(autoNumber == 1)
                        robot.followTrajectoryAsync(park1X);
                    else if(autoNumber == 2)
                        robot.followTrajectoryAsync(park2X);
                    else
                        robot.followTrajectoryAsync(park3X);
                })
                .build();

        park1X = robot.trajectoryBuilder(loopSetup2.end())
                .strafeTo(new Vector2d(14,-36))
                .splineTo(new Vector2d(12,-60), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park2X = robot.trajectoryBuilder(loopSetup2.end())
                .strafeTo(new Vector2d(14,-36))
                .build();

        park3X = robot.trajectoryBuilder(loopSetup2.end())
                .strafeTo(new Vector2d(14,-36))
                .splineTo(new Vector2d(10,-10), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        robot.followTrajectoryAsync(preloadGoal0);

        while(opModeIsActive())
        {
            robot.update();

            robot.transfer();

            if(intakeCone)
            {
                robot.intakeCone();
                intakeCone = false;
            }

            if(depositCone)
            {
                robot.depositCone();
                depositCone = false;
            }

            robot.runCounterSpin(counterSpin);

            telemetry.addData("transfer level", robot.transferLevel);
            telemetry.update();
        }
    }
}