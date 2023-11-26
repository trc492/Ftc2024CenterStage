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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;
import teamcode.drivebases.SwerveDrive;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftc3543")
public class FtcTeleOp extends FtcOpMode
{
    private static final String moduleName = FtcTeleOp.class.getSimpleName();
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale = RobotParams.DRIVE_POWER_SCALE_NORMAL;
    private double turnPowerScale = RobotParams.TURN_POWER_SCALE_NORMAL;
    private boolean manualOverride = false;
    private boolean autoAssistPlaceActive = false;
    private int aprilTagIndex = 0;
    private int scoreLevelIndex = 0;
    private double elevatorPrevPower = 0.0;
    private double armPrevPower = 0.0;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
    private boolean elevatorArmAtScorePos = false;
    private boolean pixelTrayLowerGateOpened = false;
    private boolean pixelTrayUpperGateOpened = false;
    private boolean wristUp = false;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Unknown_TeleOp";
            robot.globalTracer.openTraceLog(RobotParams.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::operatorButtonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
        setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (robot.globalTracer.isTraceLogOpened())
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting TeleOp *****");
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        //
        // Enable AprilTag vision for re-localization and auto-assist placing pixel.
        //
        if (robot.vision != null && robot.vision.aprilTagVision != null)
        {
            robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
            robot.vision.setActiveWebcam(robot.vision.getRearWebcam());
            robot.vision.setAprilTagVisionEnabled(true);
        }
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        printPerformanceMetrics(robot.globalTracer);
        robot.globalTracer.traceInfo(moduleName, "***** Stopping TeleOp *****");

        if (robot.globalTracer.isTraceLogOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
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
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            if (robot.robotDrive != null)
            {
                double[] inputs = driverGamepad.getDriveInputs(
                    RobotParams.ROBOT_DRIVE_MODE, true, drivePowerScale, turnPowerScale);

                if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    robot.robotDrive.driveBase.holonomicDrive(
                        null, inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveGyroAngle());
                }
                else
                {
                    robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                }
                robot.dashboard.displayPrintf(
                    1, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                    inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveOrientation());

                // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                if (relocalizing && robotFieldPose == null)
                {
                    robotFieldPose = robot.vision.getRobotFieldPose();
                }
            }
            //
            // Other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                // Note: manualOverride is used by multiple subsystems.
                manualOverride = operatorGamepad.getRightTrigger() >= 0.5;
                if (robot.elevatorArm != null)
                {
                    // Elevator subsystem.
                    double elevatorPower = operatorGamepad.getLeftStickY(true) * RobotParams.ELEVATOR_POWER_LIMIT;
                    if (elevatorPower != elevatorPrevPower)
                    {
                        if (manualOverride)
                        {
                            // By definition, manualOverride should not observe any safety.
                            // Therefore, we call elevator directly bypassing all safety checks.
                            robot.elevatorArm.elevator.setPower(elevatorPower);
                        }
                        else
                        {
                            robot.elevatorArm.elevatorSetPidPower(
                                null, elevatorPower, RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS);
                        }
                        elevatorPrevPower = elevatorPower;
                    }
                    // Arm subsystem.
                    double armPower = operatorGamepad.getRightStickY(true);
                    if (manualOverride)
                    {
                        // By definition, manualOverride should not observe any safety.
                        // Therefore, we call arm directly bypassing all safety checks.
                        armPower *= RobotParams.ARM_MANUAL_POWER_LIMIT;
                        if (armPower != armPrevPower)
                        {
                            robot.elevatorArm.arm.setPower(armPower);
                            armPrevPower = armPower;
                        }
                    }
                    else
                    {
                        armPower *= RobotParams.ARM_POWER_LIMIT;
                        if (armPower != armPrevPower)
                        {
                            robot.elevatorArm.armSetPidPower(
                                null, armPower, RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS);
                            armPrevPower = armPower;
                        }
                    }
                }
            }

            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotDrive != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=%s", orientation);
            robot.robotDrive.driveBase.setDriveOrientation(
                orientation, orientation == TrcDriveBase.DriveOrientation.FIELD);
            if (robot.blinkin != null)
            {
                robot.blinkin.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (pressed)
                {
                    if (robot.placePixelTask != null)
                    {
                        robot.placePixelTask.autoAssistCancel();
                    }

                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.cancel(moduleName);
                    }

                    if (robot.robotDrive != null)
                    {
                        robot.robotDrive.cancel();
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (robot.launcher != null)
                {
                    robot.launcher.setPower(pressed ? RobotParams.LAUNCHER_POWER : 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (pressed && robot.robotDrive != null)
                {
                    if (robot.robotDrive.driveBase.isGyroAssistEnabled())
                    {
                        // Disable GyroAssist drive.
                        robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                    }
                    else
                    {
                        // Enable GyroAssist drive.
                        robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (pressed && robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    if (robot.robotDrive.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD)
                    {
                        setDriveOrientation(TrcDriveBase.DriveOrientation.FIELD);
                    }
                    else
                    {
                        setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.DRIVE_POWER_SCALE_SLOW: RobotParams.DRIVE_POWER_SCALE_NORMAL;
                turnPowerScale = pressed? RobotParams.TURN_POWER_SCALE_SLOW: RobotParams.TURN_POWER_SCALE_NORMAL;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (pressed && robot.elevatorArm != null)
                {
                    robot.elevatorArm.setHangingPosition(moduleName, 0.0, null, 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (pressed && robot.elevatorArm != null)
                {
                    robot.elevatorArm.elevatorSetPosition(
                        moduleName, 0.0, RobotParams.ELEVATOR_HANG_POS, 1.0, null, 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;

            case FtcGamepad.GAMEPAD_START:
                if (robot.vision != null && robot.vision.aprilTagVision != null && robot.robotDrive != null)
                {
                    // On press of the button, we will start looking for AprilTag for re-localization.
                    // On release of the button, we will set the robot's field location if we found the AprilTag.
                    relocalizing = pressed;
                    if (!pressed)
                    {
                        if (robotFieldPose != null)
                        {
                            // Vision found an AprilTag, set the new robot field location.
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            robotFieldPose = null;
                        }
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                {
                    // Drive base is a Swerve Drive, align all steering wheels forward.
                    ((SwerveDrive) robot.robotDrive).setSteerAngle(0.0, false, false);
                }
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (pressed && robot.pixelTray != null)
                {
                    pixelTrayLowerGateOpened = !pixelTrayLowerGateOpened;
                    robot.pixelTray.setLowerGateOpened(pixelTrayLowerGateOpened, null);
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (pressed && robot.pixelTray != null)
                {
                    pixelTrayUpperGateOpened = !pixelTrayUpperGateOpened;
                    robot.pixelTray.setUpperGateOpened(pixelTrayUpperGateOpened, null);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                if (robot.intake != null)
                {
                    if (!manualOverride)
                    {
                        robot.intake.setOn(pressed);
                    }
                    else
                    {
                        robot.intake.setReverse(pressed);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (pressed && robot.elevatorArm != null)
                {
                    elevatorArmAtScorePos = !elevatorArmAtScorePos;
                    if (elevatorArmAtScorePos)
                    {
                        robot.elevatorArm.setScoringPosition(
                            moduleName, 0.0, RobotParams.ELEVATOR_LEVEL1_POS, null, 0.0);
                    }
                    else
                    {
                        robot.elevatorArm.setLoadingPosition(moduleName, 0.0, null, 0.0);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (pressed && robot.elevatorArm != null)
                {
                    wristUp = !wristUp;
                    robot.elevatorArm.wristSetPosition(wristUp? RobotParams.WRIST_UP_POS: RobotParams.WRIST_DOWN_POS);
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                autoAssistPlaceActive = pressed;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (autoAssistPlaceActive)
                {
                    if (pressed && scoreLevelIndex < RobotParams.ELEVATOR_PRESETS.length - 1)
                    {
                        scoreLevelIndex++;
                        if (robot.blinkin != null)
                        {
                            robot.blinkin.setScoreLevelIndex(scoreLevelIndex);
                        }
                    }
                }
                else
                {
                    if (pressed && robot.elevatorArm != null)
                    {
                        robot.elevatorArm.elevatorPresetPositionUp(moduleName, RobotParams.ELEVATOR_POWER_LIMIT);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (autoAssistPlaceActive)
                {
                    if (pressed && scoreLevelIndex > 0)
                    {
                        scoreLevelIndex--;
                        if (robot.blinkin != null)
                        {
                            robot.blinkin.setScoreLevelIndex(scoreLevelIndex);
                        }
                    }
                }
                else
                {
                    if (pressed && robot.elevatorArm != null)
                    {
                        robot.elevatorArm.elevatorPresetPositionDown(moduleName, RobotParams.ELEVATOR_POWER_LIMIT);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (autoAssistPlaceActive)
                {
                    if (pressed && aprilTagIndex > 0)
                    {
                        aprilTagIndex--;
                        if (robot.blinkin != null)
                        {
                            robot.blinkin.setAprilTagIndex(aprilTagIndex);
                        }
                    }
                }
                else
                {
                    if (pressed && robot.elevatorArm != null)
                    {
                        robot.elevatorArm.armPresetPositionDown(moduleName, RobotParams.ARM_POWER_LIMIT);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (autoAssistPlaceActive)
                {
                    if (pressed && aprilTagIndex < RobotParams.BLUE_BACKDROP_APRILTAGS.length - 1)
                    {
                        aprilTagIndex++;
                        if (robot.blinkin != null)
                        {
                            robot.blinkin.setAprilTagIndex(aprilTagIndex);
                        }
                    }
                }
                else
                {
                    if (pressed && robot.elevatorArm != null)
                    {
                        robot.elevatorArm.armPresetPositionUp(moduleName, RobotParams.ARM_POWER_LIMIT);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (autoAssistPlaceActive)
                {
                    if (pressed && robot.placePixelTask != null)
                    {
                        // If we are not running a match and just ran TeleOp, autoChoices.autoMenuRan will be false.
                        // In this case, we don't really know what alliance we are in. We will look for any AprilTag
                        // in front of us.
                        robot.placePixelTask.autoAssistPlace(
                            true, aprilTagIndex, RobotParams.ELEVATOR_PRESETS[scoreLevelIndex], false, null);
                    }
                }
                else
                {
                    if (pressed)
                    {
                        // Zero calibrate all subsystems (arm, elevator and turret).
                        robot.zeroCalibrate(moduleName);
                    }
                }
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
