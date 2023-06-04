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

import org.opencv.core.Point;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGridDrive;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcFtcLib.ftclib.FtcBNO055Imu;
import TrcFtcLib.ftclib.FtcDcMotor;
import teamcode.FtcAuto;
import teamcode.RobotParams;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class RobotDrive
{
    //
    // Sensors.
    //
    public final FtcBNO055Imu imu;
    public final TrcGyro gyro;

    //
    // Subclass needs to initialize the following variables.
    //
    // Drive motors.
    public FtcDcMotor lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor;
    // Drive Base.
    public TrcDriveBase driveBase;
    // Drive Controllers.
    public TrcPidDrive pidDrive;
    public TrcPurePursuitDrive purePursuitDrive;
    public TrcGridDrive gridDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotDrive()
    {
        imu = new FtcBNO055Imu(RobotParams.HWNAME_IMU);
        gyro = imu.gyro;
    }   //RobotDrive

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }

        driveBase.stop();
    }   //cancel

    /**
     * This method creates and configures a drive motor.
     *
     * @param name specifies the hardware name of the drive motor to be created.
     * @param inverted specifies true to configure the motor inverted, false otherwise.
     * @return created drive motor.
     */
    protected FtcDcMotor createDriveMotor(String name, boolean inverted)
    {
        FtcDcMotor driveMotor = new FtcDcMotor(name);

        driveMotor.motor.setMode(RobotParams.DRIVE_MOTOR_MODE);
        driveMotor.setVelocityPidCoefficients(RobotParams.DRIVE_VELPID_COEFFS);
        driveMotor.setPidCoefficients(RobotParams.DRIVE_POSPID_COEFFS);
        driveMotor.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE_ON);
        driveMotor.setMotorInverted(inverted);

        if (RobotParams.Preferences.useVelocityControl)
        {
            driveMotor.enableVelocityMode(RobotParams.DRIVE_MOTOR_MAX_VELOCITY_PPS);
        }

        return driveMotor;
    }   //createDriveMotor

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
        driveBase.setFieldPosition(
            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                (autoChoices.startPos == FtcAuto.StartPos.LEFT ?
                    RobotParams.STARTPOS_RED_LEFT : RobotParams.STARTPOS_RED_RIGHT) :
                (autoChoices.startPos == FtcAuto.StartPos.LEFT ?
                    RobotParams.STARTPOS_BLUE_LEFT : RobotParams.STARTPOS_BLUE_RIGHT));
    }   //setAutoStartPosition

    /**
     * This method adjusts the target cell according to the alliance and startPos in autoChoices.
     *
     * @param tileX specifies X tile coordinate for RED LEFT.
     * @param tileY specifies Y tile coordinate for RED LEFT.
     * @param heading specifies heading for RED LEFT.
     * @param autoChoices specifies auto choices.
     * @return adjusted target cell as TrcPose2D.
     */
    public TrcPose2D getAutoTargetCell(double tileX, double tileY, double heading, FtcAuto.AutoChoices autoChoices)
    {
        if (autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE)
        {
            tileY = -tileY;
            heading = (heading + 180.0) % 360.0;
            if (autoChoices.startPos == FtcAuto.StartPos.LEFT)
            {
                tileX = -tileX;
            }
            else
            {
                heading = -heading;
            }
        }
        else if (autoChoices.startPos == FtcAuto.StartPos.RIGHT)
        {
            tileX = -tileX;
            heading = -heading;
        }

        return new TrcPose2D(tileX, tileY, heading);
    }   //getAutoTargetCell

    /**
     * This method adjusts the target cell according to the alliance and startPos in autoChoices.
     *
     * @param targetPos specifies the target position in tile units.
     * @param autoChoices specifies auto choices.
     * @return adjusted target cell as TrcPose2D.
     */
    public TrcPose2D getAutoTargetCell(TrcPose2D targetPos, FtcAuto.AutoChoices autoChoices)
    {
        return getAutoTargetCell(targetPos.x, targetPos.y, targetPos.angle, autoChoices);
    }   //getAutoTargetCell

    /**
     * This method adjusts the target heading according to the alliance and startPos in autoChoices.
     *
     * @param heading specifies heading for RED LEFT.
     * @param autoChoices specifies auto choices.
     * @return adjusted target heading.
     */
    public double getAutoTargetHeading(double heading, FtcAuto.AutoChoices autoChoices)
    {
        TrcPose2D adjPose = getAutoTargetCell(0.0, 0.0, heading, autoChoices);
        return adjPose.angle;
    }   //getAutoTargetHeading

    /**
     * This method adjusts the target point according to the alliance and startPos in autoChoices.
     *
     * @param tileX specifies X tile coordinate for RED LEFT.
     * @param tileY specifies Y tile coordinate for RED LEFT.
     * @param heading specifies heading for RED LEFT.
     * @param autoChoices specifies auto choices.
     * @return adjusted target point as TrcPose2D.
     */
    public TrcPose2D getAutoTargetPoint(double tileX, double tileY, double heading, FtcAuto.AutoChoices autoChoices)
    {
        return gridDrive.gridCellToPose(getAutoTargetCell(tileX, tileY, heading, autoChoices));
    }   //getAutoTargetPoint

    /**
     * This method adjusts the target point according to the alliance and startPos in autoChoices.
     *
     * @param targetPos specifies the target position in tile units.
     * @param autoChoices specifies auto choices.
     * @return adjusted target point as TrcPose2D.
     */
    public TrcPose2D getAutoTargetPoint(TrcPose2D targetPos, FtcAuto.AutoChoices autoChoices)
    {
        return getAutoTargetPoint(targetPos.x, targetPos.y, targetPos.angle, autoChoices);
    }   //getAutoTargetPoint

    /**
     * This method returns the grid cell position of the pickup point according to the alliance color and the caller's
     * selection.
     * <p>
     * Button Index:
     *  0 - Left substation
     *  1 - Right substation
     *  2 - Left cone stack
     *  3 - Right cone stack
     * </p>
     * @param alliance specifies the alliance color.
     * @param buttonIndex speicifes the caller's selection.
     * @return pickup grid cell position.
     */
    public TrcPose2D autoPickupGridCell(FtcAuto.Alliance alliance, int buttonIndex)
    {
        return alliance == FtcAuto.Alliance.RED_ALLIANCE ?
            RobotParams.AUTONAV_PICKUP_RED[buttonIndex] : RobotParams.AUTONAV_PICKUP_BLUE[buttonIndex];
    }   //autoPickupGridCell

    /**
     * This method returns the grid cell position of the nearest high pole from the robot.
     *
     * @return grid cell position of the nearest high pole.
     */
    public TrcPose2D getNearestHighPoleGridCell()
    {
        TrcPose2D robotPose = driveBase.getFieldPosition();
        // robotGridCell is the cell position the robot snaps to the center of the cell.
        TrcPose2D robotGridCell = gridDrive.poseToGridCell(robotPose);
        robotGridCell = gridDrive.adjustGridCellCenter(robotGridCell);
        double minDistance = Double.MAX_VALUE;
        // Initialize closestPole to something so the compiler won't complain about NullPointerException.
        TrcPose2D closestPole = RobotParams.AUTONAV_HIGHPOLES[0];
        TrcPose2D finalRobotGridCell = new TrcPose2D();
        // Find the closest high pole (the pole that has smallest distance from the robot).
        for(TrcPose2D pole : RobotParams.AUTONAV_HIGHPOLES)
        {
            double distance = Math.pow(robotGridCell.x - pole.x, 2) + Math.pow(robotGridCell.y - pole.y, 2);
            if(distance < minDistance)
            {
                closestPole = pole;
                minDistance = distance;
            }
        }
        // We can't just return the pole's coordinate because the robot will be sitting on top of it.
        // We need to adjust the pole coordinate to one of its sides: north side, east side, south side, west side.
        // We will pick the side that's closest to the current robot location and most convenient for its heading.
        double northDeltaDistance = closestPole.y - robotGridCell.y;
        double eastDeltaDistance = closestPole.x - robotGridCell.x;
        boolean poleIsNorthOfRobot = northDeltaDistance > 0.0;
        boolean poleIsEastOfRobot = eastDeltaDistance > 0.0;
        boolean sameRow = Math.abs(northDeltaDistance) < 1.0;
        boolean sameColumn = Math.abs(eastDeltaDistance) < 1.0;

        if (!sameRow && !sameColumn)
        {
            // Robot must travel in both X and Y directions (two movement segments) to get to the pole.
            if (robotGridCell.angle == 0.0 || robotGridCell.angle == 180.0)
            {
                // Robot is heading north/south.
                // First move is along the column, second move is along the row.
                // Final x is the same as the pole, final y is north or south side of the pole.
                finalRobotGridCell.x = closestPole.x;
                finalRobotGridCell.y = closestPole.y + (poleIsNorthOfRobot ? -0.5 : 0.5);
                finalRobotGridCell.angle = poleIsEastOfRobot ? 90.0 : 270.0;
            }
            else
            {
                // Robot is heading east/west.
                // First move is along the row, second move is along the column.
                // Final x is east or west side of the pole, final y is the same as the pole.
                finalRobotGridCell.x = closestPole.x + (poleIsEastOfRobot ? -0.5 : 0.5);
                finalRobotGridCell.y = closestPole.y;
                finalRobotGridCell.angle = poleIsNorthOfRobot ? 0.0 : 180.0;
            }
        }
        else if (sameRow)
        {
            // Robot is on the same row as the pole, so we just move along the row to get to the pole.
            // Final x is the same as the pole, final y is the same as the robot.
            finalRobotGridCell.x = closestPole.x;
            finalRobotGridCell.y = robotGridCell.y;
            if (robotGridCell.angle == 0.0 || robotGridCell.angle == 180.0)
            {
                // Robot heading is north/south, need to turn it to east/west.
                finalRobotGridCell.angle = poleIsEastOfRobot ? 90.0 : 270.0;
            }
            else
            {
                // Robot heading is east/west, don't have ot turn it.
                finalRobotGridCell.angle = robotGridCell.angle;
            }
        }
        else
        {
            // Robot is on the same column as the pole, so we just move along the column to get to the pole.
            // Final x is the same as the robot, final y is the same as the pole.
            finalRobotGridCell.x = robotGridCell.x;
            finalRobotGridCell.y = closestPole.y;
            if (robotGridCell.angle == 90.0 || robotGridCell.angle == 270.0)
            {
                // Robot heading is east/west, need to turn it to north/south.
                finalRobotGridCell.angle = poleIsNorthOfRobot ? 0.0 : 180.0;
            }
            else
            {
                // Robot heading is north/south, don't have to turn it.
                finalRobotGridCell.angle = robotGridCell.angle;
            }
        }

        return finalRobotGridCell;
    }   //getNearestHighPoleGridCell

}   //class RobotDrive
