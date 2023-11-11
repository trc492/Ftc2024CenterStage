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
    private static final String moduleName = "FtcTeleOp";
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale = RobotParams.DRIVE_POWER_SCALE_NORMAL;
    private double turnPowerScale = RobotParams.TURN_POWER_SCALE_NORMAL;
    private boolean manualOverride = false;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
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
        final String funcName = "startMode";

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

        if (robot.vision != null)
        {
            // Enabling AprilTag vision to support robot re-localization.
            if (robot.vision.aprilTagVision != null)
            {
                robot.globalTracer.traceInfo(funcName, "Enabling AprilTagVision.");
                robot.vision.setAprilTagVisionEnabled(true);
            }
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
                        null, inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveGyroAngle(inputs[2]));
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
                if (robot.elevatorArm != null)
                {
                    // Elevator subsystem.
                    double elevatorPower = operatorGamepad.getLeftStickY(true);
                    if (manualOverride)
                    {
                        robot.elevatorArm.elevatorSetPower(null, 0.0, elevatorPower, 0.0, null);
                    }
                    else
                    {
                        robot.elevatorArm.elevatorSetPidPower(
                            null, elevatorPower, RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS);
                    }
                    // Arm subsystem.
                    double armPower = operatorGamepad.getRightStickY(true) * RobotParams.ARM_POWER_LIMIT;
                    if (manualOverride)
                    {
                        robot.elevatorArm.armSetPower(null, 0.0, armPower, 0.0, null);
                    }
                    else
                    {
                        robot.elevatorArm.armSetPidPower(
                            null, armPower, RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS);
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
        final String funcName = "setDriveOrientation";

        if (robot.robotDrive != null)
        {
            robot.globalTracer.traceInfo(funcName, "driveOrientation=%s", orientation);
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
                if (pressed && robot.robotDrive != null)
                {
                    // Cancel all auto-assist driving.
                    robot.robotDrive.cancel();
                    robot.placePixelTask.autoAssistCancel();
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (robot.launcher != null)
                {
                    robot.launcher.setPower(pressed? RobotParams.LAUNCHER_POWER: 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
//                if (pressed && robot.robotDrive != null && robot.elevatorArm != null && robot.vision != null)
//                {
//                    robot.placePixelTask.autoAssistPlace(FtcAuto.autoChoices.alliance, null);
//                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                  // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
//                if (pressed && robot.robotDrive != null)
//                {
//                    if (robot.robotDrive.driveBase.isGyroAssistEnabled())
//                    {
//                        // Disable GyroAssist drive.
//                        robot.robotDrive.driveBase.setGyroAssistEnabled(null);
//                    }
//                    else
//                    {
//                        // Enable GyroAssist drive.
//                        robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
//                    }
//                }
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

            case FtcGamepad.GAMEPAD_LBUMPER:
            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.DRIVE_POWER_SCALE_SLOW: RobotParams.DRIVE_POWER_SCALE_NORMAL;
                turnPowerScale = pressed? RobotParams.TURN_POWER_SCALE_SLOW: RobotParams.TURN_POWER_SCALE_NORMAL;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (pressed && robot.elevatorArm != null)
                {
                    robot.elevatorArm.setClimbingPosition(null, 0.0, null, 5.0);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
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
                    if (!pressed && robotFieldPose != null)
                    {
                        // Vision found an AprilTag, set the new robot field location but don't disturb the robot's
                        // heading because it may be set for field oriented driving.
                        robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, true);
                        robotFieldPose = null;
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
                if (robot.pixelTray != null && pressed)
                {
                    pixelTrayLowerGateOpened = !pixelTrayLowerGateOpened;
                    robot.pixelTray.setLowerGateOpened(pixelTrayLowerGateOpened, null);
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (robot.pixelTray != null && pressed)
                {
                    pixelTrayUpperGateOpened = !pixelTrayUpperGateOpened;
                    robot.pixelTray.setUpperGateOpened(pixelTrayUpperGateOpened, null);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                if (robot.intake != null)
                {
                    robot.intake.pickUp(pressed);
                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (robot.intake != null)
                {
                    robot.intake.spitOut(pressed);
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (robot.elevatorArm != null && pressed)
                {
                    wristUp = !wristUp;
                    robot.elevatorArm.wristSetPosition(wristUp? RobotParams.WRIST_UP_POS: RobotParams.WRIST_DOWN_POS);
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                manualOverride = pressed;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (robot.elevatorArm != null && pressed)
                {
                    robot.elevatorArm.elevatorPresetPositionUp(moduleName, RobotParams.ELEVATOR_POWER_LIMIT);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (robot.elevatorArm != null && pressed)
                {
                    robot.elevatorArm.elevatorPresetPositionDown(moduleName, RobotParams.ELEVATOR_POWER_LIMIT);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (robot.elevatorArm != null && pressed)
                {
                    wristUp = false;
                    robot.elevatorArm.armPresetPositionDown(moduleName, RobotParams.ARM_POWER_LIMIT);
                    robot.elevatorArm.wristSetPosition(wristUp? RobotParams.WRIST_UP_POS: RobotParams.WRIST_DOWN_POS);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (robot.elevatorArm != null && pressed)
                {
                    wristUp = true;
                    robot.elevatorArm.armPresetPositionUp(moduleName, RobotParams.ARM_POWER_LIMIT);
                    robot.elevatorArm.wristSetPosition(wristUp? RobotParams.WRIST_UP_POS: RobotParams.WRIST_DOWN_POS);
                }
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (pressed)
                {
                    // Zero calibrate all subsystems (arm, elevator and turret).
                    robot.zeroCalibrate(moduleName);
                }
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
