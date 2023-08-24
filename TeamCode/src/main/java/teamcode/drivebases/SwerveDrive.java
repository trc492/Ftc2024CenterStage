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
import TrcFtcLib.ftclib.FtcServo;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    public static final String[] servoNames = {
        RobotParams.HWNAME_LFSTEER_SERVO, RobotParams.HWNAME_RFSTEER_SERVO,
        RobotParams.HWNAME_LBSTEER_SERVO, RobotParams.HWNAME_RBSTEER_SERVO};
    public double[][] servoPositions = {
        {RobotParams.LFSTEER_MINUS90, RobotParams.LFSTEER_PLUS90},
        {RobotParams.RFSTEER_MINUS90, RobotParams.RFSTEER_PLUS90},
        {RobotParams.LBSTEER_MINUS90, RobotParams.LBSTEER_PLUS90},
        {RobotParams.RBSTEER_MINUS90, RobotParams.RBSTEER_PLUS90}
    };
    //
    // Swerve steering motors and modules.
    //
    public final FtcAnalogEncoder lfSteerEncoder, rfSteerEncoder, lbSteerEncoder, rbSteerEncoder;
    public final FtcServo lfSteerServo, rfSteerServo, lbSteerServo, rbSteerServo;
//    public final FtcServo lfSteerServo2, rfSteerServo2, lbSteerServo2, rbSteerServo2;
    public final TrcSwerveModule lfSwerveModule, rfSwerveModule, lbSwerveModule, rbSwerveModule;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super();

        readSteeringCalibrationData();

        lfDriveMotor = createDriveMotor(RobotParams.HWNAME_LFDRIVE_MOTOR, RobotParams.LFDRIVE_INVERTED);
        rfDriveMotor = createDriveMotor(RobotParams.HWNAME_RFDRIVE_MOTOR, RobotParams.RFDRIVE_INVERTED);
        lbDriveMotor = createDriveMotor(RobotParams.HWNAME_LBDRIVE_MOTOR, RobotParams.LBDRIVE_INVERTED);
        rbDriveMotor = createDriveMotor(RobotParams.HWNAME_RBDRIVE_MOTOR, RobotParams.RBDRIVE_INVERTED);

        lfSteerEncoder = new FtcAnalogEncoder(RobotParams.HWNAME_LFSTEER_ENCODER);
        rfSteerEncoder = new FtcAnalogEncoder(RobotParams.HWNAME_RFSTEER_ENCODER);
        lbSteerEncoder = new FtcAnalogEncoder(RobotParams.HWNAME_LBSTEER_ENCODER);
        rbSteerEncoder = new FtcAnalogEncoder(RobotParams.HWNAME_RBSTEER_ENCODER);

        lfSteerServo = createSteerServo(
            RobotParams.HWNAME_LFSTEER_SERVO, lfSteerEncoder, servoPositions[0][0], servoPositions[0][1],
            RobotParams.LFSTEER_INVERTED);
//        lfSteerServo2 = null;
//        lfSteerServo2 = createSteerServo(
//            RobotParams.HWNAME_LFSTEER_SERVO2, null, servoPositions[0][0], servoPositions[0][1],
//            RobotParams.LFSTEER_INVERTED);
//        lfSteerServo1.addFollower(lfSteerServo2);

        rfSteerServo = createSteerServo(
            RobotParams.HWNAME_RFSTEER_SERVO, rfSteerEncoder, servoPositions[1][0], servoPositions[1][1],
            RobotParams.RFSTEER_INVERTED);
//        rfSteerServo2 = null;
//        rfSteerServo2 = createSteerServo(
//            RobotParams.HWNAME_RFSTEER_SERVO2, null, servoPositions[1][0], servoPositions[1][1],
//            RobotParams.RFSTEER_INVERTED);
//        rfSteerServo1.addFollower(rfSteerServo2);

        lbSteerServo = createSteerServo(
            RobotParams.HWNAME_LBSTEER_SERVO, lbSteerEncoder, servoPositions[2][0], servoPositions[2][1],
            RobotParams.LBSTEER_INVERTED);
//        lbSteerServo2 = null;
//        lbSteerServo2 = createSteerServo(
//            RobotParams.HWNAME_LBSTEER_SERVO2, null, servoPositions[2][0], servoPositions[2][1],
//            RobotParams.LBSTEER_INVERTED);
//        lbSteerServo1.addFollower(lbSteerServo2);

        rbSteerServo = createSteerServo(
            RobotParams.HWNAME_RBSTEER_SERVO, rbSteerEncoder, servoPositions[3][0], servoPositions[3][1],
            RobotParams.RBSTEER_INVERTED);
//        rbSteerServo2 = null;
//        rbSteerServo2 = createSteerServo(
//            RobotParams.HWNAME_RBSTEER_SERVO2, null, servoPositions[3][0], servoPositions[3][1],
//            RobotParams.RBSTEER_INVERTED);
//        rbSteerServo1.addFollower(rbSteerServo2);

        lfSwerveModule = new TrcSwerveModule("lfSwerveModule", lfDriveMotor, lfSteerServo);
        rfSwerveModule = new TrcSwerveModule("rfSwerveModule", rfDriveMotor, rfSteerServo);
        lbSwerveModule = new TrcSwerveModule("lbSwerveModule", lbDriveMotor, lbSteerServo);
        rbSwerveModule = new TrcSwerveModule("rbSwerveModule", rbDriveMotor, rbSteerServo);
//        lfSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
//        rfSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
//        lbSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
//        rbSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);

        driveBase = new TrcSwerveDriveBase(
            lfSwerveModule, lbSwerveModule, rfSwerveModule, rbSwerveModule, gyro,
            RobotParams.DRIVE_BASE_WIDTH, RobotParams.DRIVE_BASE_LENGTH);

         if (RobotParams.Preferences.useExternalOdometry)
         {
             //
             // Create the external odometry device that uses the right back encoder port as the X odometry and
             // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
             // odometry.
             //
             TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                 new TrcDriveBaseOdometry.AxisSensor(rbDriveMotor, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
                 new TrcDriveBaseOdometry.AxisSensor[] {
                     new TrcDriveBaseOdometry.AxisSensor(lfDriveMotor, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
                     new TrcDriveBaseOdometry.AxisSensor(rfDriveMotor, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
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
        pidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff, RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
    }   //SwerveDrive

    /**
     * This method creates and configures a steering servo.
     *
     * @param servoName specifies the name of the servo.
     * @param encoder specifies the encoder for reporting servo position, can be null if none provided.
     * @param steerMinus90 specifies the logical position of -90 degree.
     * @param steerPlus90 specifies the logical position of +90 degree.
     * @param inverted specifies true if servo direction is reversed, false otherwise.
     * @return created steering servo.
     */
    private FtcServo createSteerServo(
        String servoName, FtcAnalogEncoder encoder, double steerMinus90, double steerPlus90, boolean inverted)
    {
        FtcServo servo = new FtcServo(servoName, true, encoder);

        servo.setInverted(inverted);
        servo.setPhysicalRange(-90.0, 90.0);
        servo.setLogicalRange(steerMinus90, steerPlus90);

        encoder.setScaleAndOffset(180.0, 0.0);
        // Enable Cartesian converter.
        encoder.setEnabled(true);

        return servo;
    }   //createSteerServo

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
     * This method returns the servo position value for the given wheel index and position index.
     *
     * @param wheelIndex specifies the wheel index.
     * @param posIndex specifies -1 for zero position, 0 for minus90 and 1 for plus90.
     * @return servo position value.
     */
    public double getSteeringServoPosition(int wheelIndex, int posIndex)
    {
        return posIndex == -1?
            (servoPositions[wheelIndex][0] + servoPositions[wheelIndex][1])/2.0:
            servoPositions[wheelIndex][posIndex];
    }   //getSterringServoPosition

    /**
     * This method sets all the swerve steering servos to the selected angle.
     *
     * @param posIndex specifies -1 for zero position, 0 for minus90 and 1 for plus90.
     */
    public void setSteeringServoPosition(int posIndex)
    {
        double pos;

        pos = getSteeringServoPosition(0, posIndex);
        lfSteerServo.setLogicalPosition(pos);
//        lfSteerServo2.setLogicalPosition(pos);
        pos = getSteeringServoPosition(1, posIndex);
        rfSteerServo.setLogicalPosition(pos);
//        rfSteerServo2.setLogicalPosition(pos);
        pos = getSteeringServoPosition(2, posIndex);
        lbSteerServo.setLogicalPosition(pos);
//        lbSteerServo2.setLogicalPosition(pos);
        pos = getSteeringServoPosition(3, posIndex);
        rbSteerServo.setLogicalPosition(pos);
//        rbSteerServo2.setLogicalPosition(pos);
    }  //setSteeringServoPosition

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     */
    public void saveSteeringCalibrationData()
    {
        final String funcName = "saveSteeringCalibrationData";

        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < servoNames.length; i++)
            {
                out.printf("%s: %f, %f\n", servoNames[i], servoPositions[i][0], servoPositions[i][1]);
            }
            out.close();
            TrcDbgTrace.getGlobalTracer().traceInfo(funcName, "Saved steering calibration data!");
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

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < servoNames.length; i++)
            {
                String line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(servoNames[i]))
                {
                    throw new RuntimeException("Invalid servo name in line " + line);
                }

                String[] numbers = line.substring(colonPos + 1).split(",", 2);

                for (int j = 0; j < servoPositions[0].length; j++)
                {
                    servoPositions[i][j] = Double.parseDouble(numbers[j]);
                }

                tracer.traceInfo(
                    funcName, "SteeringCalibrationData[%s]: %s", servoNames[i], Arrays.toString(servoPositions[i]));
            }
        }
        catch (FileNotFoundException e)
        {
            tracer.traceWarn(funcName, "Steering calibration data file not found, using built-in defaults.");
        }
    }   //readSteeringCalibrationData

}   //class SwerveDrive
