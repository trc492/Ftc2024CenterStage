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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDriveBaseOdometry;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcFtcLib.ftclib.FtcAnalogEncoder;
import TrcFtcLib.ftclib.FtcCRServo;
import TrcFtcLib.ftclib.FtcDcMotor;
import teamcode.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    private final String[] steerEncoderNames = {
        RobotParams.HWNAME_LFSTEER_ENCODER, RobotParams.HWNAME_RFSTEER_ENCODER,
        RobotParams.HWNAME_LBSTEER_ENCODER, RobotParams.HWNAME_RBSTEER_ENCODER};
    public double[] zeroPositions = {
        RobotParams.LFSTEER_ZERO_POS, RobotParams.RFSTEER_ZERO_POS,
        RobotParams.LBSTEER_ZERO_POS, RobotParams.RBSTEER_ZERO_POS};
    private final String[] steerServoNames = {
        RobotParams.HWNAME_LFSTEER_SERVO, RobotParams.HWNAME_RFSTEER_SERVO,
        RobotParams.HWNAME_LBSTEER_SERVO, RobotParams.HWNAME_RBSTEER_SERVO};
    private final boolean[] steerServoInverted = {
        RobotParams.LFSTEER_INVERTED, RobotParams.RFSTEER_INVERTED,
        RobotParams.LBSTEER_INVERTED, RobotParams.RBSTEER_INVERTED};
    private final String[] swerveModuleNames = {"lfSwerveModule", "rfSwerveModule", "lbSwerveModule", "rbSwerveModule"};
    //
    // Swerve steering motors and modules.
    //
    public final FtcAnalogEncoder[] steerEncoders;
    public final FtcCRServo[] steerServos;
    public final TrcSwerveModule[] swerveModules;
    public int calibrationCount = 0;

    /**
     * Constructor: Create an instance of the object.
     */
    public SwerveDrive()
    {
        super();

        final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
        readSteeringCalibrationData();
        driveMotors = createDriveMotors(driveMotorNames, driveMotorInverted);
        steerEncoders = createSteerEncoders(
            steerEncoderNames,
            RobotParams.Preferences.doSwervePhysicalAlignment ? new double[] {0.0, 0.0, 0.0, 0.0} : zeroPositions);
        steerServos = createSteerServos(steerServoNames, steerServoInverted, steerEncoders);
        swerveModules = createSwerveModules(swerveModuleNames, driveMotors, steerServos);

        driveBase = new TrcSwerveDriveBase(
            swerveModules[INDEX_LEFT_FRONT], swerveModules[INDEX_LEFT_BACK],
            swerveModules[INDEX_RIGHT_FRONT], swerveModules[INDEX_RIGHT_BACK],
            gyro, RobotParams.DRIVE_BASE_WIDTH, RobotParams.DRIVE_BASE_LENGTH);
        driveBase.setDriveOrientation(RobotParams.DEF_DRIVE_ORIENTATION, true);

         if (RobotParams.Preferences.useExternalOdometry)
         {
             //
             // Create the external odometry device that uses the right back encoder port as the X odometry and
             // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
             // odometry.
             //
             TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                 new TrcDriveBaseOdometry.AxisSensor(
                     driveMotors[INDEX_RIGHT_BACK], RobotParams.X_ODOMETRY_WHEEL_OFFSET),
                 new TrcDriveBaseOdometry.AxisSensor[] {
                     new TrcDriveBaseOdometry.AxisSensor(
                         driveMotors[INDEX_LEFT_FRONT], RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
                     new TrcDriveBaseOdometry.AxisSensor(
                         driveMotors[INDEX_RIGHT_FRONT], RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
                 gyro);
             //
             // Set the drive base to use the external odometry device overriding the built-in one.
             //
             driveBase.setDriveBaseOdometry(driveBaseOdometry);
             driveBase.setOdometryScales(
                 RobotParams.X_ODWHEEL_INCHES_PER_COUNT, RobotParams.Y_ODWHEEL_INCHES_PER_COUNT);
         }
         else
         {
             driveBase.setOdometryScales(RobotParams.YPOS_INCHES_PER_COUNT, RobotParams.YPOS_INCHES_PER_COUNT);
         }

        //
        // Create and initialize PID controllers.
        //
        TrcPidController.PidParameters xPosPidParams = new TrcPidController.PidParameters(
            RobotParams.xPosPidCoeff, RobotParams.XPOS_TOLERANCE, driveBase::getXPosition);
        TrcPidController.PidParameters yPosPidParams = new TrcPidController.PidParameters(
            RobotParams.yPosPidCoeff, RobotParams.YPOS_TOLERANCE, driveBase::getYPosition);
        TrcPidController.PidParameters turnPidParams = new TrcPidController.PidParameters(
            RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, RobotParams.TURN_SETTLING,
            RobotParams.TURN_STEADY_STATE_ERR, RobotParams.TURN_STALL_ERRRATE_THRESHOLD, driveBase::getHeading, null);

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
        pidDrive.setMsgTracer(globalTracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff, RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(globalTracer, logPoseEvents, tracePidInfo);
    }   //SwerveDrive

    /**
     * This method creates and configures all steer encoders.
     *
     * @param encoderNames specifies an array of names for each steer encoder.
     * @param zeroOffsets specifies an array of zero offsets for each steer encoder.
     * @return an array of created steer encoders.
     */
    private FtcAnalogEncoder[] createSteerEncoders(String[] encoderNames, double[] zeroOffsets)
    {
        FtcAnalogEncoder[] encoders = new FtcAnalogEncoder[encoderNames.length];

        for (int i = 0; i < steerEncoderNames.length; i++)
        {
            encoders[i] = new FtcAnalogEncoder(encoderNames[i]);
            encoders[i].setScaleAndOffset(180.0, 0.0);
            encoders[i].setZeroOffset(zeroOffsets[i]);
            // Enable Cartesian converter.
            encoders[i].setEnabled(true);
        }

        return encoders;
    }   //createSteerEncoders

    /**
     * This method creates and configures all steer servos.
     *
     * @param servoNames specifies an array of names for each steer servo.
     * @param inverted specifies an array of boolean indicating if the servo needs to be inverted.
     * @param encoders specifies an array of encoders for each steer servo.
     * @return an array of created steer servos.
     */
    private FtcCRServo[] createSteerServos(String[] servoNames, boolean[] inverted, FtcAnalogEncoder[] encoders)
    {
        FtcCRServo[] servos = new FtcCRServo[servoNames.length];

        for (int i = 0; i < servoNames.length; i++)
        {
            servos[i] = new FtcCRServo(servoNames[i], encoders[i]);
            servos[i].setMotorInverted(inverted[i]);
            servos[i].setSoftwarePidEnabled(true);
            servos[i].setPositionPidCoefficients(
                RobotParams.STEER_SERVO_KP, RobotParams.STEER_SERVO_KI,
                RobotParams.STEER_SERVO_KD, RobotParams.STEER_SERVO_KF, RobotParams.STEER_SERVO_IZONE);
            servos[i].setPositionPidTolerance(RobotParams.STEER_SERVO_TOLERANCE);
        }
        return servos;
    }   //createSteerServos

    /**
     * This method creates and configures all swerve modules.
     *
     * @param moduleNames specifies an array of names for each swerve module.needs to be inverted.
     * @param driveMotors specifies an array of drive motors.
     * @param steerServos specifies an array of steer servos.
     * @return an array of created swerve modules.
     */
    private TrcSwerveModule[] createSwerveModules(
        String[] moduleNames, FtcDcMotor[] driveMotors, FtcCRServo[] steerServos)
    {
        TrcSwerveModule[] modules = new TrcSwerveModule[moduleNames.length];

        for (int i = 0; i < moduleNames.length; i++)
        {
            modules[i] = new TrcSwerveModule(moduleNames[i], driveMotors[i], steerServos[i]);
        }

        return modules;
    }   //createSwerveModules

    /**
     * This method enables/disables performance monitoring of all steering servo motors.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setSteerPerformanceMonitorEnabled(boolean enabled)
    {
        for (FtcCRServo servo: steerServos)
        {
            servo.setPerformanceMonitorEnabled(enabled);
        }
    }   //setSteerPerformanceMonitorEnabled

    /**
     * This method prints the performance info to the trace log.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public void printSteerPerformanceInfo(TrcDbgTrace tracer)
    {
        for (FtcCRServo servo : steerServos)
        {
            servo.printPidControlTaskPerformance(tracer);
        }
    }   //printSteerPerformanceInfo

    /**
     * This method sets the steering angle of all swerve modules.
     *
     * @param angle specifies the steer angle.
     * @param optimize specifies true to optimize (only turns within +/- 90 degrees), false otherwse.
     * @param hold specifies true to hold the angle, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize, boolean hold)
    {
        for (TrcSwerveModule module: swerveModules)
        {
            module.setSteerAngle(angle, optimize, hold);
        }
    }   //setSteerAngle

    /**
     * This method checks if anti-defense mode is enabled.
     *
     * @return true if anti-defense mode is enabled, false if disabled.
     */
    public boolean isAntiDefenseEnabled()
    {
        return ((TrcSwerveDriveBase) driveBase).isAntiDefenseEnabled();
    }   //isAntiDefenseEnabled

    /**
     * This method enables/disables the anti-defense mode where it puts all swerve wheels into an X-formation.
     * By doing so, it is very difficult for others to push us around.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setAntiDefenseEnabled(String owner, boolean enabled)
    {
        if (owner == null || !enabled || driveBase.acquireExclusiveAccess(owner))
        {
            ((TrcSwerveDriveBase) driveBase).setAntiDefenseEnabled(owner, enabled);
            if (!enabled)
            {
                driveBase.releaseExclusiveAccess(owner);
            }
        }
    }   //setAntiDefenseEnabled

    /**
     * This method starts the steering calibration.
     */
    public void startSteeringCalibration()
    {
        calibrationCount = 0;
        Arrays.fill(zeroPositions, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     */
    public void stopSteeringCalibration()
    {
        for (int i = 0; i < zeroPositions.length; i++)
        {
            zeroPositions[i] /= calibrationCount;
        }
        calibrationCount = 0;
        saveSteeringCalibrationData();
    }   //stopSteeringCalibration

    /**
     * This method is called periodically to sample the steer encoders for averaging the zero position data.
     */
    public void runSteeringCalibration()
    {
        for (int i = 0; i < zeroPositions.length; i++)
        {
            zeroPositions[i] += steerEncoders[i].getRawPosition();
        }
        calibrationCount++;
    }   //runSteeringCalibration

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     */
    public void saveSteeringCalibrationData()
    {
        final String funcName = "saveSteeringCalibrationData";

        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < steerServoNames.length; i++)
            {
                out.printf("%s: %f\n", steerServoNames[i], zeroPositions[i]);
            }
            out.close();
            TrcDbgTrace.getGlobalTracer().traceInfo(
                funcName, "SteeringCalibrationData%s=%s",
                Arrays.toString(steerServoNames), Arrays.toString(zeroPositions));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering calibration data from a file on the Robot Controller.
     *
     * @throws RuntimeException if file contains invalid data.
     */
    public void readSteeringCalibrationData()
    {
        final String funcName = "readSteeringCalibrationData";
        TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
        String line = null;

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < steerServoNames.length; i++)
            {
                line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(steerServoNames[i]))
                {
                    throw new RuntimeException("Invalid servo name in line " + line);
                }

                zeroPositions[i] = Double.parseDouble(line.substring(colonPos + 1));
            }
        }
        catch (FileNotFoundException e)
        {
            tracer.traceWarn(funcName, "Steering calibration data file not found, using built-in defaults.");
        }
        catch (NumberFormatException e)
        {
            tracer.traceErr(funcName, "Invalid zero position value in line %s", line);
        }
        catch (RuntimeException e)
        {
            tracer.traceErr(funcName, "Invalid servo name in line %s", line);
        }

        tracer.traceInfo(
            funcName, "SteeringCalibrationData%s=%s", Arrays.toString(steerServoNames), Arrays.toString(zeroPositions));
    }   //readSteeringCalibrationData

}   //class SwerveDrive
