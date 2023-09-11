/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.drivebases;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.opencv.core.Point;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcImu;
import teamcode.FtcAuto;
import teamcode.RobotParams;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class RobotDrive
{
    public static final int INDEX_LEFT_FRONT = 0;
    public static final int INDEX_RIGHT_FRONT = 1;
    public static final int INDEX_LEFT_BACK = 2;
    public static final int INDEX_RIGHT_BACK = 3;
    protected final String[] driveMotorNames = {
        RobotParams.HWNAME_LFDRIVE_MOTOR, RobotParams.HWNAME_RFDRIVE_MOTOR,
        RobotParams.HWNAME_LBDRIVE_MOTOR, RobotParams.HWNAME_RBDRIVE_MOTOR};
    protected final boolean[] driveMotorInverted = {
        RobotParams.LFDRIVE_INVERTED, RobotParams.RFDRIVE_INVERTED,
        RobotParams.LBDRIVE_INVERTED, RobotParams.RBDRIVE_INVERTED};
    //
    // Sensors.
    //
    public final FtcImu gyro;

    //
    // Subclass needs to initialize the following variables.
    //
    // Drive motors.
    public FtcDcMotor[] driveMotors;
    // Drive Base.
    public TrcDriveBase driveBase;
    // Drive Controllers.
    public TrcPidDrive pidDrive;
    public TrcPurePursuitDrive purePursuitDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotDrive()
    {
        gyro = new FtcImu(
            RobotParams.HWNAME_IMU, RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }   //RobotDrive

    /**
     * This method cancels any PIDDrive operation still in progress.
     *
     * @param owner specifies the owner that requested the cancel.
     */
    public void cancel(String owner)
    {
        if (pidDrive != null && pidDrive.isActive())
        {
            pidDrive.cancel(owner);
        }

        if (purePursuitDrive != null && purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel(owner);
        }

        driveBase.stop(owner);
    }   //cancel

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method creates and configures all drive motors.
     *
     * @param motorNames specifies an array of names for each drive motor.
     * @param inverted specifies an array of boolean indicating if the drive motor needs to be inverted.
     * @return an array of created drive motors.
     */
    protected FtcDcMotor[] createDriveMotors(String[] motorNames, boolean[] inverted)
    {
        FtcDcMotor[] motors = new FtcDcMotor[motorNames.length];

        for (int i = 0; i < motorNames.length; i++)
        {
            motors[i] = new FtcDcMotor(motorNames[i]);
            motors[i].setVelocityPidCoefficients(RobotParams.DRIVE_VELPID_COEFFS);
            motors[i].setPositionPidCoefficients(RobotParams.DRIVE_POSPID_COEFFS);
            motors[i].setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE_ON);
            motors[i].setMotorInverted(inverted[i]);
        }

        return motors;
    }   //createDriveMotors

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading, boolean tileUnit)
    {
        double unitScale = tileUnit? RobotParams.FULL_TILE_INCHES: 1.0;

        return new TrcPose2D(xTargetLocation*unitScale, yTargetLocation*unitScale, heading);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading)
    {
        return pathPoint(xTargetLocation, yTargetLocation, heading, true);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading, boolean tileUnit)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, tileUnit);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, true);
    }   //pathPoint

    /**
     * This method sets the robot's autonomous starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setAutoStartPosition(FtcAuto.AutoChoices autoChoices)
    {
    }   //setAutoStartPosition

}   //class RobotDrive
