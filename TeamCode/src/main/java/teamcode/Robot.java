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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcServoGrabber;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcRobotBattery;
import teamcode.autotasks.TaskPickupCone;
import teamcode.autotasks.TaskScoreCone;
import teamcode.drivebases.MecanumDrive;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.subsysstems.BlinkinLEDs;
import teamcode.subsysstems.Grabber;
import teamcode.subsysstems.Turret;
import teamcode.vision.EocvVision;
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
    public FtcRevBlinkin blinkin;
    public FtcRobotBattery battery;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public Turret turret;
    public TrcPidActuator elevator = null;
    public TrcPidActuator arm = null;
    public TrcServoGrabber grabber = null;
    public TaskScoreCone scoreConeTask;
    public TaskPickupCone pickupConeTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTextView(
            ((FtcRobotControllerActivity)opMode.hardwareMap.appContext)
                .findViewById(com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        globalTracer = TrcDbgTrace.getGlobalTracer();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if (runMode != TrcRobot.RunMode.TELEOP_MODE &&
            (RobotParams.Preferences.useVuforia ||
             RobotParams.Preferences.useTensorFlow ||
             RobotParams.Preferences.useEasyOpenCV))
        {
            // Don't need to enable vision for TeleOp because we are not doing auto-assist cone pickup (yet).
            vision = new Vision(this);
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
            robotDrive = RobotParams.Preferences.swerveRobot? new SwerveDrive(this): new MecanumDrive(this);
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.initSubsystems)
            {
                if(RobotParams.Preferences.useTurret)
                {
                    turret = new Turret(RobotParams.HWNAME_TURRET, globalTracer, false);
                }

                if (RobotParams.Preferences.useElevator)
                {
                    final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
                        RobotParams.ELEVATOR_MOTOR_INVERTED,
                        RobotParams.ELEVATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED,
                        RobotParams.ELEVATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters elevatorParams = new TrcPidActuator.Parameters()
                        .setPosRange(RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS)
                        .setScaleAndOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
                        .setPidParams(new TrcPidController.PidParameters(
                            RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD,
                            RobotParams.ELEVATOR_TOLERANCE, null))
                        .setPowerCompensation(this::getElevatorPowerCompensation)
//                        .setStallProtectionParams(
//                            RobotParams.ELEVATOR_STALL_MIN_POWER, RobotParams.ELEVATOR_STALL_TOLERANCE,
//                            RobotParams.ELEVATOR_STALL_TIMEOUT, RobotParams.ELEVATOR_RESET_TIMEOUT)
                        .setZeroCalibratePower(RobotParams.ELEVATOR_CAL_POWER)
                        .setPosPresets(RobotParams.ELEVATOR_PRESET_TOLERANCE, RobotParams.ELEVATOR_PRESET_LEVELS);
                    elevator = new FtcMotorActuator(
                        RobotParams.HWNAME_ELEVATOR, motorParams, elevatorParams).getPidActuator();
                    // Set asymmetric power limits so down power is smaller since it's helped by gravity.
                    elevator.getPidController().setOutputRange(-RobotParams.ELEVATOR_DOWN_POWER_SCALE, 1.0);
                    elevator.setMsgTracer(globalTracer);
                }

                if (RobotParams.Preferences.useArm)
                {
                    final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
                        RobotParams.ARM_MOTOR_INVERTED,
                        RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED,
                        RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters armParams = new TrcPidActuator.Parameters()
                        .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
                        .setScaleAndOffset(RobotParams.ARM_DEG_PER_COUNT, RobotParams.ARM_OFFSET)
                        .setPidParams(new TrcPidController.PidParameters(
                            RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_TOLERANCE,
                            null))
                        .resetPositionOnLowerLimit(true)
//                        .setStallProtectionParams(
//                            RobotParams.ARM_STALL_MIN_POWER, RobotParams.ARM_STALL_TOLERANCE,
//                            RobotParams.ARM_STALL_TIMEOUT, RobotParams.ARM_RESET_TIMEOUT)
                        .setZeroCalibratePower(RobotParams.ARM_CAL_POWER)
                        .setPosPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.ARM_PICKUP_PRESETS);
                    arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, motorParams, armParams).getPidActuator();
                    arm.setMsgTracer(globalTracer);
                }

                if (RobotParams.Preferences.useGrabber)
                {
                    grabber = new Grabber(RobotParams.HWNAME_GRABBER, globalTracer).getServoGrabber();
                }
                //
                // Create and initialize auto-assist tasks.
                //
                scoreConeTask = new TaskScoreCone("ScoreConeTask", this, globalTracer);
                pickupConeTask = new TaskPickupCone("PickupConeTask", this, globalTracer);
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
            }
            //
            // Enable odometry for all opmodes. We need odometry in TeleOp for GridDrive.
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
                    // will default to RED_LEFT starting position which is the AutoChoices default.
                    robotDrive.setAutoStartPosition(FtcAuto.autoChoices);
                    globalTracer.traceInfo(
                        funcName, "No saved RobotPose, use autoChoiceStartPos=%s",
                        robotDrive.driveBase.getFieldPosition());
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        //
        // The following are performance counters, could be disabled for competition if you want.
        // But it might give you some insight if somehow autonomous wasn't performing as expected.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.setElapsedTimerEnabled(true);
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
            if (vision.vuforiaVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling Vuforia.");
                vision.vuforiaVision.setEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
                vision.tensorFlowShutdown();
            }

            if (vision.eocvVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling EocvVision.");
                vision.eocvVision.setDetectObjectType(EocvVision.ObjectType.NONE);
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
     * This method zero calibrates all subsystems. We zero calibrate the arm and elevator first before the turret
     * because if the arm is down, it will hit the drive motors when turning the turret.
     */
    public void zeroCalibrate()
    {
        final String funcName = "zeroCalibrate";

        if (elevator != null)
        {
            globalTracer.traceInfo(funcName, "Zero calibrating elevator.");
            elevator.zeroCalibrate();
        }

        if (arm != null)
        {
            // Set up a notification event for zero calibrating the arm so we get a callback.
            TrcEvent callbackEvent = new TrcEvent("zeroCalEvent");
            callbackEvent.setCallback(this::zeroCalTurret, null);
            globalTracer.traceInfo(funcName, "Zero calibrating arm.");
            arm.zeroCalibrate(callbackEvent);
        }
        else
        {
            // There is no arm to worry about, so just zero calibrate the turret.
            zeroCalTurret(null);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrate the turret only if it's safe to do so (the arm is zero calibrated).
     *
     * @param context not used.
     */
    private void zeroCalTurret(Object context)
    {
        final String funcName = "zeroCalTurret";

        if (turret != null)
        {
            globalTracer.traceInfo(funcName, "Zero calibrating turret.");
            turret.zeroCalibrate();
        }
    }   //zeroCalTurret

    /**
     * This method enables/disables grabber auto-assist and updated the LED state.
     *
     * @param enabled specifies true to enable grabber auto-assist, false to disable.
     */
    public void setGrabberAutoAssistOn(boolean enabled)
    {
        if (enabled)
        {
            grabber.enableAutoAssist(null, 0.0, null, 0.0);
            if (blinkin != null)
            {
                blinkin.setPatternState(BlinkinLEDs.AUTOASSIST_GRABBER_ON, true);
            }
        }
        else
        {
            grabber.cancelAutoAssist();
            if (blinkin != null)
            {
                blinkin.setPatternState(BlinkinLEDs.AUTOASSIST_GRABBER_ON, false);
            }
        }
    }   //setGrabberAutoAssistOn

    /**
     * This method returns the power required to make the elevator gravity neutral (i.e. held in place at any height).
     *
     * @param currPower specifies the current motor power.
     * @return elevator gravity compensation power.
     */
    private double getElevatorPowerCompensation(double currPower)
    {
        // Don't apply power if it's already at the bottom.
        return Math.abs(elevator.getPosition() - RobotParams.ELEVATOR_MIN_POS) <= RobotParams.ELEVATOR_TOLERANCE?
                0.0: RobotParams.ELEVATOR_POWER_COMPENSATION;
    }   //getElevatorPowerCompensation

    /**
     * This method calculates the scoring arm angle from the distance sensor value.
     *
     * @return arm angle for scoring the cone.
     */
    public Double getScoringArmAngle()
    {
        final String funcName = "getScoringArmAngle";
        Double armAngle = null;

        if (turret != null)
        {
            // Compare the current sensor value to the settling data and get the minimum value. That's the most
            // accurate pole distance.
            double sensorValue = turret.getSensorValue();
            Double minValue = turret.getMinValue();
            double poleDistance = minValue == null? sensorValue: Math.min(sensorValue, minValue);

            globalTracer.traceInfo(funcName, "sensorValue=%.2f, minValue=%.2f", sensorValue, minValue);
            poleDistance += RobotParams.CLAW_DISTANCE_ADUSTMENT;
            if (poleDistance < RobotParams.ARM_JOINT_LENGTH)
            {
                armAngle = 90.0 + RobotParams.ARM_ANGLE_OFFSET -
                           Math.toDegrees(Math.acos(poleDistance/RobotParams.ARM_JOINT_LENGTH));
            }
            globalTracer.traceInfo(
                funcName, "sensorValue=%.2f, poleDist=%.2f, armAngle=%s", sensorValue, poleDistance, armAngle);
        }

        return armAngle;
   }   //getScoringArmAngle

    /**
     * This method starts auto-assist navigation to the pickup/scoring position according to the caller's selection.
     *
     * @param buttonIndex specifies the caller selection.
     */
    public void startAutoNavigate(int buttonIndex)
    {
        TrcPose2D endPoint =
            grabber.hasObject() ?
                robotDrive.getNearestHighPoleGridCell() :
                robotDrive.autoPickupGridCell(FtcAuto.autoChoices.alliance, buttonIndex);

        robotDrive.gridDrive.driveToEndPoint(robotDrive.gridDrive.gridCellToPose(endPoint));
    }   //startAutoNavigate

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
