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

package teamcode.autocommands;

import java.util.Locale;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcVisionAprilTag;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
        START,
        PLACE_PURPLE_PIXEL,
        DO_DELAY,
        DRIVE_TO_LOOKOUT,
        FIND_APRILTAG,
        DRIVE_TO_APRILTAG,
        MOVE_ARM,
        PLACE_YELLOW_PIXEL,
        RETRACT_ARM,
        RETRACT_ELEVATOR,
        PARK_AT_BACKSTAGE,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent elevatorArmEvent;
    private final TrcStateMachine<State> sm;
    private int aprilTagId = 0;
    private TrcPose2D aprilTagPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAuto(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        elevatorArmEvent = new TrcEvent(moduleName + ".elevatorArm");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=%s)...", sm.getNextState());
        }
        else
        {
            TrcPose2D targetPoseTile, targetPose, intermediate1, intermediate2, intermediate3, intermediate4;

            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START:
                    int teamPropPos = 0;
                    String msg;
                    // Set robot's start position on the field.
                    robot.setRobotStartPosition(autoChoices);
                    // Setup ElevatorArm.
                    robot.elevatorArm.wrist.setPosition(RobotParams.WRIST_DOWN_POS);
                    robot.elevatorArm.setupPositions(null, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ARM_LOAD_POS);
                    // Use vision to determine team prop position (0: not found, 1, 2, 3).
                    if (robot.vision != null)
                    {
                        teamPropPos = robot.vision.getLastDetectedTeamPropPosition();
                        if (teamPropPos > 0)
                        {
                            msg = "Team Prop found at position " + teamPropPos;
                            robot.globalTracer.traceInfo(moduleName, msg);
                            robot.speak(msg);
                        }
                    }

                    if (teamPropPos == 0)
                    {
                        // Vision did not find the team prop, set to default position.
                        teamPropPos = 2;
                        msg = "No team prop found, default to position " + teamPropPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    int teamPropIndex = teamPropPos - 1;
                    // Red alliance's Spike Marks are in opposite order from the Blue alliance.
                    int spikeMarkIndex =
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE? teamPropIndex: 2 - teamPropIndex;
                    // Determine AprilTag ID to look for.
                    aprilTagId =
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            RobotParams.BLUE_BACKDROP_APRILTAGS[teamPropIndex]:
                            RobotParams.RED_BACKDROP_APRILTAGS[teamPropIndex];
                    // Navigate robot to spike mark 1, 2 or 3.
                    targetPoseTile =
                        autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                            RobotParams.BLUE_AUDIENCE_SPIKE_MARKS[spikeMarkIndex]:
                            RobotParams.BLUE_BACKSTAGE_SPIKE_MARKS[spikeMarkIndex];
                    targetPose = robot.adjustPoseByAlliance(targetPoseTile, autoChoices.alliance);
//                    double xTileOffset = spikeMarkIndex == 0? 0.45: spikeMarkIndex == 2? -0.45: 0.0;
//                    targetPose = robot.adjustPoseByAlliance(
//                        targetPoseTile.x + xTileOffset, targetPoseTile.y, targetPoseTile.angle, autoChoices.alliance);
                    if (teamPropPos != 2) {
                        intermediate1 =
                                robot.adjustPoseByAlliance(
                                        targetPoseTile.x, targetPoseTile.y + 0.1, 180.0, autoChoices.alliance);
                    }
                    else {
                        intermediate1 =
                                robot.adjustPoseByAlliance(
                                        targetPoseTile.x, targetPoseTile.y - 0.1, 180.0, autoChoices.alliance);
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false, intermediate1, targetPose);
                    sm.waitForSingleEvent(event, State.PLACE_PURPLE_PIXEL);
                    break;

                case PLACE_PURPLE_PIXEL:
                    // Place purple pixel on the spike mark position 1, 2 or 3.
                    if (robot.intake != null)
                    {
                        robot.intake.spitOut(0.0, 1.0, event);
                        sm.waitForSingleEvent(event, State.DO_DELAY);
                    }
                    else
                    {
                        sm.setState(State.DO_DELAY);
                    }
                    break;

                case DO_DELAY:
                    // Do delay waiting for alliance partner to get out of the way if necessary.
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.DRIVE_TO_LOOKOUT);
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_LOOKOUT);
                    }
                    break;

                case DRIVE_TO_LOOKOUT:
                    // Drive to the lookout point where we can see the AprilTag clearly.
                    if (autoChoices.startPos == FtcAuto.StartPos.BACKSTAGE)
                    {
                        intermediate1 = robot.adjustPoseByAlliance(0.5, 2.0, 180.0, autoChoices.alliance);
                        intermediate2 = robot.adjustPoseByAlliance(1.5, 2.0, -90.0, autoChoices.alliance);
                        targetPose = robot.adjustPoseByAlliance(1.5, 1.5, -90.0, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            intermediate1, intermediate2, targetPose);
                    }
                    else
                    {
                        intermediate1 = robot.adjustPoseByAlliance(-1.5, 2.5, 180.0, autoChoices.alliance);
                        intermediate2 = robot.adjustPoseByAlliance(-2.5, 2.5, 180.0, autoChoices.alliance);
                        intermediate3 = robot.adjustPoseByAlliance(-2.5, 0.5, 180.0, autoChoices.alliance);
                        intermediate4 = robot.adjustPoseByAlliance(1.5, 0.5, -90.0, autoChoices.alliance);
                        targetPose = robot.adjustPoseByAlliance(1.5, 1.5, -90.0, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 9.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            intermediate1, intermediate2, intermediate3, intermediate4, targetPose);
                    }
                    sm.waitForSingleEvent(event, State.FIND_APRILTAG);
                    break;

                case FIND_APRILTAG:
                    // Set up elevator and arm for placing pixel on the Backdrop.
                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.setupPositions(
                            null, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ARM_SCORE_BACKDROP_POS);
                    }
                    // Use vision to determine the appropriate AprilTag location.
                    if (robot.vision != null && robot.vision.aprilTagVision != null)
                    {
                        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
                            robot.vision.getDetectedAprilTag(aprilTagId, -1);
                        if (aprilTagInfo != null)
                        {
                            // Determine the absolute field location of the AprilTag.
                            aprilTagPose =
                                robot.robotDrive.driveBase.getFieldPosition().subtractRelativePose(
                                    new TrcPose2D(
                                        aprilTagInfo.objPose.x, aprilTagInfo.objPose.y,
                                        -90.0 - robot.robotDrive.driveBase.getHeading()));
                            robot.globalTracer.traceInfo(
                                moduleName, "AprilTag %d found at %s (absPose=%s)",
                                aprilTagId, aprilTagInfo.objPose, aprilTagPose);
                            msg = String.format(
                                Locale.US, "AprilTag %d found at x %.1f, y %.1f",
                                aprilTagId, aprilTagInfo.objPose.x, aprilTagInfo.objPose.y);
                            robot.dashboard.displayPrintf(1, "%s", msg);
                            robot.speak(msg);
                            sm.setState(State.DRIVE_TO_APRILTAG);
                        }
                        else if (visionExpiredTime == null)
                        {
                            // Can't find AprilTag, set a timeout and try again.
                            visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                        }
                        else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                        {
                            // Timed out, moving on.
                            robot.globalTracer.traceInfo(moduleName, "AprilTag %d not found.", aprilTagId);
                            sm.setState(State.DRIVE_TO_APRILTAG);
                        }
                    }
                    else
                    {
                        // AprilTag Vision is not enabled, moving on.
                        robot.globalTracer.traceInfo(moduleName, "AprilTag Vision not enabled.");
                        sm.setState(State.DRIVE_TO_APRILTAG);
                    }
                    break;

                case DRIVE_TO_APRILTAG:
                    // Navigate robot to Apriltag.
                    if (aprilTagPose == null)
                    {
                        // Vision did not see AprilTag, just go to it blindly using odometry and its known location.
                        aprilTagPose = RobotParams.APRILTAG_POSES[aprilTagId - 1];
                        robot.globalTracer.traceInfo(
                            moduleName, "Drive to AprilTag using blind odometry (pose=%s).", aprilTagPose);
                    }
                    // Account for end-effector offset from the camera.
                    aprilTagPose.x -= 1.0;
                    aprilTagPose.angle = -90.0;
                    robot.robotDrive.purePursuitDrive.start(
                        event, 3.0,  robot.robotDrive.driveBase.getFieldPosition(), false, aprilTagPose);
                    sm.addEvent(event);
//                    robot.elevatorArm.setupPositions(null, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ARM_SCORE_BACKDROP_POS, elevatorArmEvent, 0.0);
                    robot.elevatorArm.setElevatorPosition(null, 0.0, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ELEVATOR_POWER_LIMIT, elevatorArmEvent, 0.0);
                    sm.addEvent(elevatorArmEvent);
                    sm.waitForEvents(State.MOVE_ARM, true);
                    break;
                case MOVE_ARM:
                    robot.elevatorArm.setArmPosition(null, 0.0, RobotParams.ARM_SCORE_BACKDROP_POS, RobotParams.ARM_POWER_LIMIT, event, 6.0);
                    robot.elevatorArm.wristSetPosition(RobotParams.WRIST_UP_POS);
                    sm.waitForSingleEvent(event, State.PLACE_YELLOW_PIXEL);
                    break;
                case PLACE_YELLOW_PIXEL:
                    // Place yellow pixel at the appropriate location on the backdrop.
                    if (robot.pixelTray != null)
                    {
                        robot.pixelTray.setLowerGateOpened(true, event);
                        robot.pixelTray.setUpperGateOpened(true, event);
                        sm.waitForSingleEvent(event, State.RETRACT_ARM);
                    }
                    else
                    {
                        sm.setState(State.RETRACT_ARM);
                    }
                    break;
                case RETRACT_ARM:
                    // Retract arm.
                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.setArmPosition(null, 0.0, RobotParams.ARM_MIN_POS, RobotParams.ARM_POWER_LIMIT, event, 5.0);
                        robot.elevatorArm.wristSetPosition(RobotParams.WRIST_DOWN_POS);
                    }
                    sm.waitForSingleEvent(event, State.RETRACT_ELEVATOR);
                    break;
                case RETRACT_ELEVATOR:
                    // Retract arm.
                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.setElevatorPosition(null, 0.0, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ELEVATOR_POWER_LIMIT, event, 5.0);
                    }
                    sm.waitForSingleEvent(event, State.PARK_AT_BACKSTAGE);
                    break;
                case PARK_AT_BACKSTAGE:
                    // Navigate robot to the backstage parking location.
                    targetPoseTile =
                        autoChoices.parkPos == FtcAuto.ParkPos.CORNER?
                            RobotParams.PARKPOS_BLUE_CORNER: RobotParams.PARKPOS_BLUE_CENTER;
                    targetPose = robot.adjustPoseByAlliance(targetPoseTile, autoChoices.alliance);
                    intermediate1 = robot.adjustPoseByAlliance(
                        2.0, targetPoseTile.y, targetPoseTile.angle, autoChoices.alliance);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false, intermediate1, targetPose);
                    sm.waitForSingleEvent(event,State.DONE);
                    break;

                default:
                case DONE:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
