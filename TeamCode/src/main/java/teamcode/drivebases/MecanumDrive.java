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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcOdometryWheel;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import teamcode.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    /**
     * Constructor: Create an instance of the object.
     */
    public MecanumDrive()
    {
        super();

        driveMotors = createDriveMotors(driveMotorNames, driveMotorInverted);

        driveBase = new TrcMecanumDriveBase(
            driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
            driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK], gyro);

        if (RobotParams.Preferences.useExternalOdometry)
        {
            //
            // Create the external odometry device that uses the right back encoder port as the X odometry and
            // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
            // odometry.
            //
            TrcOdometryWheel driveBaseOdometry = new TrcOdometryWheel(
                new TrcOdometryWheel.AxisSensor(
                    driveMotors[INDEX_RIGHT_BACK], RobotParams.X_ODWHEEL_Y_OFFSET, RobotParams.X_ODWHEEL_X_OFFSET),
                new TrcOdometryWheel.AxisSensor[] {
                    new TrcOdometryWheel.AxisSensor(
                        driveMotors[INDEX_LEFT_FRONT], RobotParams.YLEFT_ODWHEEL_X_OFFSET,
                        RobotParams.YLEFT_ODWHEEL_Y_OFFSET),
                    new TrcOdometryWheel.AxisSensor(
                        driveMotors[INDEX_RIGHT_FRONT], RobotParams.YRIGHT_ODWHEEL_X_OFFSET,
                        RobotParams.YRIGHT_ODWHEEL_Y_OFFSET)},
                gyro);
            //
            // Set the drive base to use the external odometry device overriding the built-in one.
            //
            driveBase.setDriveBaseOdometry(driveBaseOdometry);
            driveBase.setOdometryScales(RobotParams.ODWHEEL_INCHES_PER_COUNT, RobotParams.ODWHEEL_INCHES_PER_COUNT);
        }
        else
        {
            driveBase.setOdometryScales(RobotParams.XPOS_INCHES_PER_COUNT, RobotParams.YPOS_INCHES_PER_COUNT);
        }
        //
        // Create and initialize PID controllers.
        //
        TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
        TrcPidController.PidParameters xPosPidParams = new TrcPidController.PidParameters(
            RobotParams.xPosPidCoeff, RobotParams.XPOS_TOLERANCE, driveBase::getXPosition);
        TrcPidController.PidParameters yPosPidParams = new TrcPidController.PidParameters(
            RobotParams.yPosPidCoeff, RobotParams.YPOS_TOLERANCE, driveBase::getYPosition);
        TrcPidController.PidParameters turnPidParams = new TrcPidController.PidParameters(
            RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, driveBase::getHeading);
//            RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, RobotParams.TURN_SETTLING,
//            RobotParams.TURN_STEADY_STATE_ERR, RobotParams.TURN_STALL_ERRRATE_THRESHOLD, driveBase::getHeading);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, xPosPidParams, yPosPidParams, turnPidParams);

        pidDrive.getXPidCtrl().setRampRate(RobotParams.X_RAMP_RATE);
        pidDrive.getYPidCtrl().setRampRate(RobotParams.Y_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setRampRate(RobotParams.TURN_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setAbsoluteSetPoint(true);
        // FTC robots generally have USB performance issues where the sampling rate of the gyro is not high enough.
        // If the robot turns too fast, PID will cause oscillation. By limiting turn power, the robot turns slower.
        pidDrive.getTurnPidCtrl().setOutputLimit(RobotParams.TURN_POWER_LIMIT);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setMsgTracer(tracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff, RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
//        purePursuitDrive.getXPosPidCtrl().setStallErrRateThreshold(RobotParams.PPD_POS_ERR_RATE_THRESHOLD);
//        purePursuitDrive.getYPosPidCtrl().setStallErrRateThreshold(RobotParams.PPD_POS_ERR_RATE_THRESHOLD);
//        purePursuitDrive.getYPosPidCtrl().setErrorTolerances(RobotParams.PPD_POS_TOLERANCE, 2.0);
//        purePursuitDrive.getTurnPidCtrl().setStallErrRateThreshold(RobotParams.PPD_TURN_ERR_RATE_THRESHOLD);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(tracer, logPoseEvents, tracePidInfo);

//        purePursuitDrive.getXPosPidCtrl().setOnTargetDebugEnabled(false);
//        purePursuitDrive.getYPosPidCtrl().setOnTargetDebugEnabled(false);
//        purePursuitDrive.getTurnPidCtrl().setOnTargetDebugEnabled(false);
    }   //MecanumDrive

}   //class MecanumDrive
