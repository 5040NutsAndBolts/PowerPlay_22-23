package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Odometry extends Hardware
{
    public Odometry (HardwareMap hardwareMap)
    {
        super(hardwareMap);
    }

    //constructs localizer object using one parameter of a list of three wheel positions
    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer
            (
                    new ArrayList<>(Arrays.asList(
                            new Pose2d(5.685874282, 0, Math.PI / 2), //center wheel
                            new Pose2d(0, trackwidth/2, 0), //right wheel
                            new Pose2d(0, -trackwidth/2, 0))) //left wheel
            )
    {
        //overrides getWheelPositions method
        @Override
        public List<Double> getWheelPositions()
        {
            ArrayList<Double> wheelPositions = new ArrayList<>(3);
            wheelPositions.add(centerOdomTraveled);
            wheelPositions.add(leftOdomTraveled);
            wheelPositions.add(rightOdomTraveled);
            return wheelPositions;
        }
    };

    private int getDeltaLeftTicks()
    {
        try
        {
            int total=bulkData.getMotorCurrentPosition(leftOdom);
            int oldPos = leftEncoderPos;
            leftEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    private int getDeltaRightTicks()
    {
        try
        {
            int total=bulkData.getMotorCurrentPosition(rightOdom);
            int oldPos = rightEncoderPos;
            rightEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    private int getDeltaCenterTicks()
    {
        try
        {
            int total=bulkData.getMotorCurrentPosition(centerOdom);
            int oldPos = centerEncoderPos;
            centerEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    public void updatePositionRoadRunner()
    {
        try
        {
            bulkData = expansionHub.getBulkInputData();
        }
        catch(Exception e)
        {
            return;
        }

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks()/ ODOM_TICKS_PER_IN );
        double deltaRightDist = -(getDeltaRightTicks()/ ODOM_TICKS_PER_IN );
        double deltaCenterDist = getDeltaCenterTicks()/ ODOM_TICKS_PER_IN;

        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = -odom.getPoseEstimate().component1();
        y = -odom.getPoseEstimate().component2();
    }

    public void resetOdometry(double x, double y, double theta)
    {
        odom.setPoseEstimate(new Pose2d(-x, -y, theta));

        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;

        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Resets encoder values then sets them back to run without encoders because wheels and odometry are same pointer
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}