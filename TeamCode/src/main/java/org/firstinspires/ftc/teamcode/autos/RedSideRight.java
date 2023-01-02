package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.signalfinding.GreenFinder;
import org.firstinspires.ftc.teamcode.signalfinding.OrangeFinder;
import org.firstinspires.ftc.teamcode.signalfinding.PurpleFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red Right Auto", group = "Autonomous")
public class RedSideRight extends LinearOpMode
{
    int autoNumber = 1;

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

        Odometry robot = new Odometry(hardwareMap);

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

            telemetry.addData("auto", autoNumber);
            telemetry.update();
        }

        robot.resetOdometry(0,80,0);

        waitForStart();

        while(opModeIsActive())
        {


            robot.robotODrive(0,0,0);
        }
    }

}
