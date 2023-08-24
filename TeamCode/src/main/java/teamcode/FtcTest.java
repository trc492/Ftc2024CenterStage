/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdPurePursuitDrive;
import TrcCommonLib.command.CmdTimedDrive;

import TrcCommonLib.trclib.TrcElapsedTimer;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcChoiceMenu;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcMenu;
import TrcFtcLib.ftclib.FtcPidCoeffCache;
import TrcFtcLib.ftclib.FtcValueMenu;
import teamcode.drivebases.SwerveDrive;

/**
 * This class contains the Test Mode program. It extends FtcTeleOp so that we can teleop control the robot for
 * testing purposes. It provides numerous tests for diagnosing problems with the robot. It also provides tools
 * for tuning and calibration.
 */
@TeleOp(name="FtcTest", group="Ftc3543")
public class FtcTest extends FtcTeleOp
{
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        VISION_TEST,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        PURE_PURSUIT_DRIVE,
        CALIBRATE_SWERVE_STEERING
    }   //enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SENSORS_TEST;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;
        TrcPidController.PidCoefficients tunePidCoeff = null;
        double tuneDistance = 0.0;
        double tuneHeading = 0.0;
        double tuneDrivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "test=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%1f " +
                "driveTime=%.1f " +
                "drivePower=%.1f " +
                "tunePidCoeff=%s " +
                "tuneDistance=%.1f " +
                "tuneHeading=%.1f " +
                "tuneDrivePower=%.1f",
                test, xTarget, yTarget, turnTarget, driveTime, drivePower, tunePidCoeff, tuneDistance, tuneHeading,
                tuneDrivePower);
        }   //toString

    }   //class TestChoices

    private final FtcPidCoeffCache pidCoeffCache = new FtcPidCoeffCache(RobotParams.TEAM_FOLDER_PATH);
    private final TestChoices testChoices = new TestChoices();
    private TrcElapsedTimer elapsedTimer = null;
    private FtcChoiceMenu<Test> testMenu = null;

    private TrcRobot.RobotCommand testCommand = null;
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    //
    // Swerve Steering calibration.
    //
    private static final double STEER_CALIBRATE_STEP = 0.01;
    private static final String[] posNames = {"Zero", "Minus90", "Plus90"};
    private static final int[] posDataIndices = {-1, 0, 1};
    private int posIndex = 0;
    private int wheelIndex = 0;

    //
    // Overrides FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        //
        // TeleOp initialization.
        //
        super.initRobot();
        if (RobotParams.Preferences.useLoopPerformanceMonitor)
        {
            elapsedTimer = new TrcElapsedTimer("TestLoopMonitor", 2.0);
        }
        //
        // Test menus.
        //
        doTestMenus();
        //
        // Create the robot command for the tests that need one.
        //
        switch (testChoices.test)
        {
            case DRIVE_MOTORS_TEST:
                if (!RobotParams.Preferences.noRobot)
                {
                    testCommand = new CmdDriveMotorsTest(
                        new FtcDcMotor[] {robot.robotDrive.lfDriveMotor, robot.robotDrive.rfDriveMotor,
                                          robot.robotDrive.lbDriveMotor, robot.robotDrive.rbDriveMotor},
                        5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, testChoices.drivePower, 0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, 0.0, testChoices.drivePower, 0.0);
                }
                break;

            case PID_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    // Distance targets are in feet, so convert them into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.drivePower, null,
                        new TrcPose2D(testChoices.xTarget*12.0, testChoices.yTarget*12.0, testChoices.turnTarget));
                }
                break;

            case TUNE_X_PID:
                if (!RobotParams.Preferences.noRobot)
                {
                    // Distance target is in feet, so convert it into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(testChoices.tuneDistance*12.0, 0.0, 0.0));
                }
                break;

            case TUNE_Y_PID:
                if (!RobotParams.Preferences.noRobot)
                {
                    // Distance target is in feet, so convert it into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, testChoices.tuneDistance*12.0, 0.0));
                }
                break;

            case TUNE_TURN_PID:
                if (!RobotParams.Preferences.noRobot)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, 0.0, testChoices.tuneHeading));
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    testCommand = new CmdPurePursuitDrive(
                        robot.robotDrive.driveBase, RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff,
                        RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
                }
                break;
        }
        //
        // Only VISION_TEST needs TensorFlow, shut it down for all other tests.
        //
        if (robot.vision != null && robot.vision.tensorFlowVision != null && testChoices.test != Test.VISION_TEST)
        {
            robot.globalTracer.traceInfo("TestInit", "Disabling TensorFlowVision.");
            robot.vision.setTensorFlowVisionEnabled(false);
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called before test mode is about to start so it can initialize appropriate subsystems for the
     * test.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        final String funcName = "startMode";

        super.startMode(prevMode, nextMode);
        switch (testChoices.test)
        {
            case VISION_TEST:
                if (robot.vision != null)
                {
                    //
                    // Vision generally will impact performance, so we only enable it if it's needed.
                    //
                    if (robot.vision.aprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(funcName, "Enabling AprilTagVision.");
                        robot.vision.setAprilTagVisionEnabled(true);
                    }

                    if (robot.vision.redConeVision != null)
                    {
                        robot.globalTracer.traceInfo(funcName, "Enabling RedConeVision.");
                        robot.vision.setRedConeVisionEnabled(true);
                    }

                    if (robot.vision.blueConeVision != null)
                    {
                        robot.globalTracer.traceInfo(funcName, "Enabling BlueConeVision.");
                        robot.vision.setBlueConeVisionEnabled(true);
                    }

                    if (robot.vision.tensorFlowVision != null)
                    {
                        robot.globalTracer.traceInfo(funcName, "Enabling TensorFlowVison.");
                        robot.vision.setTensorFlowVisionEnabled(true);
                    }
                }
                break;

            case PID_DRIVE:
            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);
                    //
                    // Doing a 48x48-inch square box with robot heading always pointing to the center of the box.
                    //
                    // Set the current position as the absolute field origin so the path can be an absolute path.
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));
                    ((CmdPurePursuitDrive)testCommand).start(
                        robot.robotDrive.driveBase.getFieldPosition(), false,
                        new TrcPose2D(-24.0, 0, 45.0),
                        new TrcPose2D(-24.0, 48.0, 135.0),
                        new TrcPose2D(24.0, 48.0, 225.0),
                        new TrcPose2D(24.0, 0.0, 315.0),
                        new TrcPose2D(0.0, 0.0, 0.0));
                }
                break;

            case CALIBRATE_SWERVE_STEERING:
                if (robot.robotDrive != null && (robot.robotDrive instanceof SwerveDrive))
                {
                    posIndex = 0;
                    wheelIndex = 0;
                    ((SwerveDrive) robot.robotDrive).setSteeringServoPosition(posDataIndices[posIndex]);
                }
                else
                {
                    throw new RuntimeException("Steering calibration can only be done on Swerve Drive.");
                }
                break;
        }
    }   //startMode

    /**
     * This method is called before test mode is about to exit so it can do appropriate cleanup.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (testCommand != null)
        {
            testCommand.cancel();
        }

        super.stopMode(prevMode, nextMode);
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        //
        // Run the testCommand if any.
        //
        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Display test status.
        //
        switch (testChoices.test)
        {
            case DRIVE_SPEED_TEST:
                if (!RobotParams.Preferences.noRobot)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;

                    if (prevTime != 0.0)
                    {
                        acceleration = (velocity - prevVelocity)/(currTime - prevTime);
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(8, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(9, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                }
                break;

            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    robot.dashboard.displayPrintf(8, "Timed Drive: %.0f sec", testChoices.driveTime);
                    robot.dashboard.displayPrintf(9, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    robot.dashboard.displayPrintf(
                        10, "rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                        robot.robotDrive.lfDriveMotor.getPosition(), robot.robotDrive.rfDriveMotor.getPosition(),
                        robot.robotDrive.lbDriveMotor.getPosition(), robot.robotDrive.rbDriveMotor.getPosition());
                }
                break;

            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (!RobotParams.Preferences.noRobot && testChoices.tunePidCoeff != null)
                {
                    robot.dashboard.displayPrintf(7, "TunePid=%s", testChoices.tunePidCoeff);
                }
                //
                // Intentionally falling through.
                //
            case PID_DRIVE:
            case PURE_PURSUIT_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl;
                    int lineNum = 9;

                    if (testChoices.test == Test.PURE_PURSUIT_DRIVE)
                    {
                        xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                        yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                        turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                    }
                    else
                    {
                        xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                        yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                        turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                    }

                    robot.dashboard.displayPrintf(
                        8, "RobotPose=%s,rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                        robot.robotDrive.driveBase.getFieldPosition(),
                        robot.robotDrive.lfDriveMotor.getPosition(), robot.robotDrive.rfDriveMotor.getPosition(),
                        robot.robotDrive.lbDriveMotor.getPosition(), robot.robotDrive.rbDriveMotor.getPosition());
                    if (xPidCtrl != null)
                    {
                        xPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    yPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                    turnPidCtrl.displayPidInfo(lineNum);
                }
                break;
        }

        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                15, "Period: %.3f(%.3f/%.3f)",
                elapsedTimer.getAverageElapsedTime(), elapsedTimer.getMinElapsedTime(),
                elapsedTimer.getMaxElapsedTime());
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            switch (testChoices.test)
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                    doSensorsTest();
                    break;

                case VISION_TEST:
                    doVisionTest();
                    break;

                case CALIBRATE_SWERVE_STEERING:
                    if (robot.robotDrive != null && (robot.robotDrive instanceof SwerveDrive))
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;

                        swerveDrive.setSteeringServoPosition(posDataIndices[posIndex]);
                        robot.dashboard.displayPrintf(
                            1, "State: pos=%s, wheel=%s", posNames[posIndex], SwerveDrive.servoNames[wheelIndex]);
                        robot.dashboard.displayPrintf(
                            2, "Front Steer: lfPos=%.2f, rfPos=%.2f",
                            swerveDrive.getSteeringServoPosition(0, posDataIndices[posIndex]),
                            swerveDrive.getSteeringServoPosition(1, posDataIndices[posIndex]));
                        robot.dashboard.displayPrintf(
                            3, "Back Steer: lbPos=%.2f, rbPos=%.2f",
                            swerveDrive.getSteeringServoPosition(2, posDataIndices[posIndex]),
                            swerveDrive.getSteeringServoPosition(3, posDataIndices[posIndex]));
                    }
                    break;
            }
        }
    }   //periodic

    //
    // Overrides TrcGameController.ButtonHandler in TeleOp.
    //

    /**
     * This method is called when a driver gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (allowButtonControl() || testChoices.test == Test.VISION_TEST)
        {
            boolean processed = false;
            //
            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.
            //
            robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                    {
                        if (pressed)
                        {
                            posIndex = (posIndex + 1) % posNames.length;
                        }
                        processed = true;
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
//                    if (testChoices.test == Test.VISION_TEST)
//                    {
//                        if (pressed && robot.vision != null && robot.vision.eocvVision != null)
//                        {
//                            robot.vision.eocvVision.setNextObjectType();
//                        }
//                        processed = true;
//                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
//                    if (testChoices.test == Test.VISION_TEST)
//                    {
//                        if (pressed && robot.vision != null && robot.vision.eocvVision != null)
//                        {
//                            robot.vision.eocvVision.setNextVideoOutput();
//                        }
//                        processed = true;
//                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                    {
                        if (pressed)
                        {
                            ((SwerveDrive) robot.robotDrive).saveSteeringCalibrationData();
                        }
                        processed = true;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed && testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;

                        if (posDataIndices[posIndex] != -1 &&
                            swerveDrive.servoPositions[wheelIndex][posDataIndices[posIndex]] + STEER_CALIBRATE_STEP
                            <= 1.0)
                        {
                            swerveDrive.servoPositions[wheelIndex][posDataIndices[posIndex]] += STEER_CALIBRATE_STEP;
                        }
                        processed = true;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed && testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;

                        if (posDataIndices[posIndex] != -1 &&
                            swerveDrive.servoPositions[wheelIndex][posDataIndices[posIndex]] - STEER_CALIBRATE_STEP
                            >= 0.0)
                        {
                            swerveDrive.servoPositions[wheelIndex][posDataIndices[posIndex]] -= STEER_CALIBRATE_STEP;
                        }
                        processed = true;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                    {
                        if (pressed)
                        {
                            wheelIndex = (wheelIndex + 1) % SwerveDrive.servoNames.length;
                        }
                        processed = true;
                    }
                    break;
            }
            //
            // If the control was not processed by this method, pass it back to TeleOp.
            //
            if (!processed)
            {
                super.driverButtonEvent(gamepad, button, pressed);
            }
        }
    }   //driverButtonEvent

    /**
     * This method is called when an operator gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (allowButtonControl())
        {
            boolean processed = false;
            //
            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.
            //
            robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
            //
            // If the control was not processed by this method, pass it back to TeleOp.
            //
            if (!processed)
            {
                super.operatorButtonEvent(gamepad, button, pressed);
            }
        }
    }   //operatorButtonEvent

    /**
     * This method creates and displays the test menus and record the selected choices.
     */
    private void doTestMenus()
    {
        //
        // Create menus.
        //
        testMenu = new FtcChoiceMenu<>("Tests:", null);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", testMenu, -180.0, 180.0, 5.0, 0.0, " %.0f deg");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", testMenu, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", testMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
        //
        // PID Tuning menus.
        //
        FtcValueMenu tuneKpMenu = new FtcValueMenu(
            "Kp:", testMenu, 0.0, 1.0, 0.001, this::getTuneKp, " %f");
        FtcValueMenu tuneKiMenu = new FtcValueMenu(
            "Ki:", tuneKpMenu, 0.0, 1.0, 0.001, this::getTuneKi, " %f");
        FtcValueMenu tuneKdMenu = new FtcValueMenu(
            "Kd:", tuneKiMenu, 0.0, 1.0, 0.001, this::getTuneKd, " %f");
        FtcValueMenu tuneKfMenu = new FtcValueMenu(
            "Kf:", tuneKdMenu, 0.0, 1.0, 0.001, this::getTuneKf, " %f");
        FtcValueMenu tuneDistanceMenu = new FtcValueMenu(
            "PID Tune distance:", tuneKfMenu, -10.0, 10.0, 0.5, 0.0,
            " %.1f ft");
        FtcValueMenu tuneHeadingMenu = new FtcValueMenu(
            "PID Tune heading:", tuneDistanceMenu, -180.0, 180.0, 5.0, 0.0,
            " %.0f deg");
        FtcValueMenu tuneDrivePowerMenu = new FtcValueMenu(
            "PID Tune drive power:", tuneHeadingMenu, -1.0, 1.0, 0.1, 1.0,
            " %.1f");
        //
        // Populate menus.
        //
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false, xTargetMenu);
        testMenu.addChoice("Tune X PID", Test.TUNE_X_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID, false, tuneKpMenu);
        testMenu.addChoice("Pure Pursuit Drive", Test.PURE_PURSUIT_DRIVE, false);
        testMenu.addChoice("Calibrate Swerve Steering", Test.CALIBRATE_SWERVE_STEERING, false);

        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        tuneKpMenu.setChildMenu(tuneKiMenu);
        tuneKiMenu.setChildMenu(tuneKdMenu);
        tuneKdMenu.setChildMenu(tuneKfMenu);
        tuneKfMenu.setChildMenu(tuneDistanceMenu);
        tuneDistanceMenu.setChildMenu(tuneHeadingMenu);
        tuneHeadingMenu.setChildMenu(tuneDrivePowerMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu);
        //
        // Fetch choices.
        //
        testChoices.test = testMenu.getCurrentChoiceObject();
        testChoices.xTarget = xTargetMenu.getCurrentValue();
        testChoices.yTarget = yTargetMenu.getCurrentValue();
        testChoices.turnTarget = turnTargetMenu.getCurrentValue();
        testChoices.driveTime = driveTimeMenu.getCurrentValue();
        testChoices.drivePower = drivePowerMenu.getCurrentValue();
        testChoices.tunePidCoeff = new TrcPidController.PidCoefficients(
            tuneKpMenu.getCurrentValue(), tuneKiMenu.getCurrentValue(),
            tuneKdMenu.getCurrentValue(),tuneKfMenu.getCurrentValue());
        testChoices.tuneDistance = tuneDistanceMenu.getCurrentValue();
        testChoices.tuneHeading = tuneHeadingMenu.getCurrentValue();
        testChoices.tuneDrivePower = tuneDrivePowerMenu.getCurrentValue();

        TrcPidController tunePidCtrl = getTunePidController(testChoices.test);
        if (tunePidCtrl != null)
        {
            //
            // Write the user input PID coefficients to a cache file so tune PID menu can read them as start value
            // next time.
            //
            pidCoeffCache.writeCachedPidCoeff(tunePidCtrl, testChoices.tunePidCoeff);
        }
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(0, "Test Choices: %s", testChoices);
    }   //doTestMenus

    /**
     * This method returns the PID controller for the tune test.
     *
     * @param test specifies the selected test.
     * @return tune PID controller.
     */
    private TrcPidController getTunePidController(Test test)
    {
        TrcPidController pidCtrl;

        switch (test)
        {
            case TUNE_X_PID:
                pidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                break;

            case TUNE_Y_PID:
                pidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                break;

            case TUNE_TURN_PID:
                pidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                break;

            default:
                pidCtrl = null;
        }

        return pidCtrl;
    }   //getTunePidController

    /**
     * This method is called by the tuneKpMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kp value of the PID controller being tuned.
     */
    private double getTuneKp()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kP;
        }

        return value;
    }   //getTuneKp

    /**
     * This method is called by the tuneKiMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Ki value of the PID controller being tuned.
     */
    private double getTuneKi()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kI;
        }

        return value;
    }   //getTuneKi

    /**
     * This method is called by the tuneKdMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kd value of the PID controller being tuned.
     */
    private double getTuneKd()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kD;
        }

        return value;
    }   //getTuneKd

    /**
     * This method is called by the tuneKfMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kf value of the PID controller being tuned.
     */
    double getTuneKf()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kF;
        }

        return value;
    }   //getTuneKF

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        int lineNum = 8;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        if (!RobotParams.Preferences.noRobot)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "DriveEnc: lf=%.0f,rf=%.0f,lb=%.0f,rb=%.0f",
                robot.robotDrive.lfDriveMotor.getPosition(), robot.robotDrive.rfDriveMotor.getPosition(),
                robot.robotDrive.lbDriveMotor.getPosition(), robot.robotDrive.rbDriveMotor.getPosition());

            if (robot.robotDrive instanceof SwerveDrive)
            {
                SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                robot.dashboard.displayPrintf(
                    lineNum++, "SteerEnc: lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
                    swerveDrive.lfSteerEncoder.getPosition(), swerveDrive.rfSteerEncoder.getPosition(),
                    swerveDrive.lbSteerEncoder.getPosition(), swerveDrive.rbSteerEncoder.getPosition());
            }
        }

        if (robot.robotDrive.gyro != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "Gyro(x,y,z): Heading=(%.1f,%.1f,%.1f), Rate=(%.3f,%.3f,%.3f)",
                robot.robotDrive.gyro.getXHeading().value, robot.robotDrive.gyro.getYHeading().value,
                robot.robotDrive.gyro.getZHeading().value, robot.robotDrive.gyro.getXRotationRate().value,
                robot.robotDrive.gyro.getYRotationRate().value, robot.robotDrive.gyro.getZRotationRate().value);
        }
    }   //doSensorsTest

    /**
     * This method calls vision code to detect target objects and display their info.
     */
    private void doVisionTest()
    {
        if (robot.vision != null)
        {
//            TrcVisionTargetInfo<?> signalInfo = robot.vision.getDetectedSignalInfo();
//            if (signalInfo != null)
//            {
//                int signalPos = robot.vision.determineDetectedSignal(signalInfo);
//                robot.dashboard.displayPrintf(10, "Signal: %s (pos=%d)", signalInfo, signalPos);
//            }
//            else
//            {
//                robot.dashboard.displayPrintf(10, "Signal: Not found.");
//            }
//
//            TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
//                robot.vision.getDetectedConeInfo();
//            if (coneInfo != null)
//            {
//                EocvVision.ObjectType objectType = robot.vision.eocvVision.getDetectObjectType();
//                robot.dashboard.displayPrintf(
//                    11, "%s Cone: %s", objectType == EocvVision.ObjectType.RED_CONE? "Red": "Blue", coneInfo);
//            }
//            else
//            {
//                robot.dashboard.displayPrintf(11, "Cone: Not found.");
//            }
        }
    }   //doVisionTest

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        return !RobotParams.Preferences.noRobot &&
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.DRIVE_SPEED_TEST);
    }   //allowTeleOp

    /**
     * This method is called to determine if Test mode is allowed to do button control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowButtonControl()
    {
        return !RobotParams.Preferences.noRobot &&
               (testChoices.test == Test.SUBSYSTEMS_TEST ||
                testChoices.test == Test.DRIVE_SPEED_TEST ||
                testChoices.test == Test.CALIBRATE_SWERVE_STEERING);
    }   //allowButtonControl

}   //class FtcTest
