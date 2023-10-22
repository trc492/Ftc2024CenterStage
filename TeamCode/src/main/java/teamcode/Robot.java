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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRobotBattery;
import teamcode.drivebases.MecanumDrive;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.subsystems.ElevatorArm;
import teamcode.subsystems.Intake;
import teamcode.subsystems.PixelTray;
import teamcode.vision.Vision;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Global objects.
    //
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public ElevatorArm elevatorArm;
    public Intake intake;
    public PixelTray pixelTray;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        dashboard = FtcDashboard.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        checkRobotSupport();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if (RobotParams.Preferences.tuneColorBlobVision ||
            RobotParams.Preferences.useAprilTagVision ||
            RobotParams.Preferences.useColorBlobVision ||
            RobotParams.Preferences.useTensorFlowVision)
        {
            vision = new Vision(this, null);
        }
        //
        // If noRobot is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!RobotParams.Preferences.noRobot)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new BlinkinLEDs(RobotParams.HWNAME_BLINKIN);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize RobotDrive.
            //
            robotDrive = RobotParams.Preferences.swerveRobot? new SwerveDrive(): new MecanumDrive();
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useElevatorArm)
                {
                    elevatorArm = new ElevatorArm(globalTracer, false);
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake(RobotParams.HWNAME_INTAKE, globalTracer);
                }

                if (RobotParams.Preferences.usePixelTray)
                {
                    pixelTray = new PixelTray(RobotParams.HWNAME_PIXELTRAY, globalTracer);
                }
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    }   //toString

    private void checkRobotSupport()
    {
        if (RobotParams.Preferences.swerveRobot)
        {
            RobotParams.Preferences.useBlinkin = false;
            RobotParams.Preferences.useExternalOdometry = false;
            RobotParams.Preferences.useSubsystems = false;
        }
        else if (RobotParams.Preferences.powerPlayRobot)
        {
            RobotParams.Preferences.useBlinkin = true;
            RobotParams.Preferences.hasWebCam2 = false;
            RobotParams.Preferences.useExternalOdometry = true;
            RobotParams.Preferences.useSubsystems = true;
            RobotParams.Preferences.useElevatorArm = false;
        }
        else if (RobotParams.Preferences.centerStageRobot)
        {
            RobotParams.Preferences.useBlinkin = false;
            RobotParams.Preferences.hasWebCam2 = true;
            RobotParams.Preferences.useExternalOdometry = false;
            RobotParams.Preferences.useSubsystems = false;
            RobotParams.Preferences.useAprilTagVision = true;
            RobotParams.Preferences.useColorBlobVision = true;
            RobotParams.Preferences.useTensorFlowVision = false;
        }
    }   //checkRobotSupport

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "startMode";

        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode != TrcRobot.RunMode.AUTO_MODE)
            {
                // In TeleOp or Test mode. If we are not running a competition match, autonomous may not have run
                // prior to this. Therefore, we cannot inherit the robot position from previous autonomous mode.
                // In this case, we will just assume previous robot start position.
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(funcName, "Restore saved RobotPose=%s", endOfAutoRobotPose);
                }
                else
                {
                    // There was no saved robotPose, use previous autonomous start position. In case we didn't even
                    // have a previous autonomous run (e.g. just powering up the robot and go into TeleOp), then we
                    // will default to starting position of the AutoChoices default.
                    setRobotStartPosition(FtcAuto.autoChoices);
                    globalTracer.traceInfo(
                        funcName, "No saved RobotPose, use autoChoiceStartPos=%s",
                        robotDrive.driveBase.getFieldPosition());
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopMode";
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.purplePixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling PurplePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.PurplePixel, false);
            }

            if (vision.greenPixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling GreenPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.GreenPixel, false);
            }

            if (vision.yellowPixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling YellowPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.YellowPixel, false);
            }

            if (vision.whitePixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling WhitePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.WhitePixel, false);
            }

            if (vision.redConeVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling RedConeVision.");
                vision.setRedConeVisionEnabled(false);
            }

            if (vision.blueConeVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling BlueConeVision.");
                vision.setBlueConeVisionEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling TensorFlowVision.");
                vision.setTensorFlowVisionEnabled(false);
            }
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(funcName, "Saved robot pose=%s", endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method update all subsystem status on the dashboard.
     */
    public void updateStatus()
    {
        int lineNum = 2;

        if (elevatorArm != null)
        {
            dashboard.displayPrintf(
                lineNum++, "Elevator: power=%.1f, pos=%.1f, lowerLimitSw=%s",
                elevatorArm.elevator.getPower(), elevatorArm.elevator.getPosition(),
                elevatorArm.elevator.isLowerLimitSwitchActive());
            dashboard.displayPrintf(
                lineNum++, "Arm: power=%.1f, pos=%.1f, lowerLimitSw=%s",
                elevatorArm.arm.getPower(), elevatorArm.arm.getPosition(), elevatorArm.arm.isLowerLimitSwitchActive());
        }

        if (intake != null)
        {
            dashboard.displayPrintf(
                lineNum++, "Intake: power=%.1f", intake.getIntakeMotor().getPower());
        }

        if (pixelTray != null)
        {
            dashboard.displayPrintf(
                lineNum++, "PixelTray: Gate1=%s, Gate2=%s", pixelTray.isGate1Opened(), pixelTray.isGate2Opened());
        }
    }   //updateStatus

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (elevatorArm != null)
        {
            elevatorArm.zeroCalibrate(owner);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        robotDrive.driveBase.setFieldPosition(
            adjustPoseByAlliance(
                autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                    RobotParams.STARTPOS_BLUE_AUDIENCE: RobotParams.STARTPOS_BLUE_BACKSTAGE,
                autoChoices.alliance, false));
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = -newPose.y;
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.FULL_TILE_INCHES;
            newPose.y *= RobotParams.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, true);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, true);
    }   //adjustPoseByAlliance

    private final TrcEvent elevatorEvent = new TrcEvent("robot.elevatorEvent");
    private final TrcEvent armEvent = new TrcEvent("robot.armEvent");
    private TrcEvent completionEvent = null;

    /**
     * This method is called when one of the subsystem operations has completed. It signals the completion event only
     * if all subsystem operations have completed.
     *
     * @param context not used.
     */
    private void setupCompletion(Object context)
    {
        if (completionEvent != null && elevatorEvent.isSignaled() && armEvent.isSignaled())
        {
            completionEvent.signal();
            completionEvent = null;
        }
    }   //setupCompletion

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     * @param timeout specifies the maximum time allowed for the operation.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void setupElevatorArm(
        String owner, boolean moveArmFirst, double elevatorDelay, double elevatorPos, double armDelay, double armPos,
        double timeout,
        TrcEvent event)
    {
        if (elevatorArm != null)
        {
            if (event != null)
            {
                elevatorEvent.clear();
                armEvent.clear();
                elevatorEvent.setCallback(this::setupCompletion, null);
                armEvent.setCallback(this::setupCompletion, null);
                completionEvent = event;
            }

            if (moveArmFirst)
            {
                elevatorArm.setArmPosition(
                    owner, armDelay, armPos, true, 1.0, event != null ? armEvent : null, timeout);
                elevatorArm.setElevatorPosition(
                    owner, elevatorDelay, elevatorPos, true, 1.0, event != null ? elevatorEvent : null, timeout);
            }
            else
            {
                elevatorArm.setElevatorPosition(
                    owner, elevatorDelay, elevatorPos, true, 1.0, event != null ? elevatorEvent : null, timeout);
                elevatorArm.setArmPosition(
                    owner, armDelay, armPos, true, 1.0, event != null ? armEvent : null, timeout);
            }
        }
    }   //setupElevatorArm

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param timeout specifies the maximum time allowed for the operation.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void setupElevatorArm(
        String owner, boolean moveArmFirst, double elevatorPos, double armPos, double timeout, TrcEvent event)
    {
        setupElevatorArm(owner, moveArmFirst, 0.0, elevatorPos, 0.0, armPos, timeout, event);
    }   //setupElevatorArm

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     * @param timeout specifies the maximum time allowed for the operation.
     */
    public void setupElevatorArm(
        String owner, boolean moveArmFirst, double elevatorDelay, double elevatorPos, double armDelay, double armPos,
        double timeout)
    {
        setupElevatorArm(owner, moveArmFirst, elevatorDelay, elevatorPos, armDelay, armPos, timeout, null);
    }   //setupElevatorArm

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     */
    public void setupElevatorArm(
        String owner, boolean moveArmFirst, double elevatorDelay, double elevatorPos, double armDelay, double armPos)
    {
        setupElevatorArm(owner, moveArmFirst, elevatorDelay, elevatorPos, armDelay, armPos, 0.0, null);
    }   //setupElevatorArm

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param timeout specifies the maximum time allowed for the operation.
     */
    public void setupElevatorArm(String owner, boolean moveArmFirst, double elevatorPos, double armPos, double timeout)
    {
        setupElevatorArm(owner, moveArmFirst, 0.0, elevatorPos, 0.0, armPos, timeout, null);
    }   //setupElevatorArm

    /**
     * This method set up the elevator and arm subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param moveArmFirst specifies true to move the arm first then elevator, false the other way round.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     */
    public void setupElevatorArm(String owner, boolean moveArmFirst, double elevatorPos, double armPos)
    {
        setupElevatorArm(owner, moveArmFirst, 0.0, elevatorPos, 0.0, armPos, 0.0, null);
    }   //setupElevatorArm

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
