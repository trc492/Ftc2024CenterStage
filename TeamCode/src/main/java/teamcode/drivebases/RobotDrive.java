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

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcImu;
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
        gyro = new FtcImu(RobotParams.HWNAME_IMU, RobotParams.hubLogoDirection, RobotParams.hubUsbDirection);
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
            motors[i].setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        }

        return motors;
    }   //createDriveMotors

}   //class RobotDrive
