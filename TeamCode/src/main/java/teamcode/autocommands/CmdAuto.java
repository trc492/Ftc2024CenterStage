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
        LOWER_ELEVATOR,
        DRIVE_TO_APRILTAG,
        PLACE_YELLOW_PIXEL,
        RAISE_ELEVATOR,
        RETRACT_ALL,
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
                    // ElevatorArm should already be at loading position. Setting it to loading position is to hold
                    // it in place for travelling.
                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.setLoadingPosition(null, 0.0, null, 0.0);
                    }
                    // If vision is enabled, use vision to determine team prop position (0: not found, 1, 2, 3).
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
                        // Either vision is not enabled, or vision did not find the team prop, set to default position.
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
                    if (teamPropPos != 2)
                    {
                        // Intermediate point of pos 1 or 3 does go as far.
                        intermediate1 =
                            robot.adjustPoseByAlliance(
                                targetPoseTile.x, targetPoseTile.y + 0.1, 180.0, autoChoices.alliance);
                    }
                    else
                    {
                        // Intermediate point of pos 2 goes further to bump out the team prop.
                        intermediate1 =
                            robot.adjustPoseByAlliance(
                                targetPoseTile.x, targetPoseTile.y - 0.1, 180.0, autoChoices.alliance);
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        event, 4.0, robot.robotDrive.driveBase.getFieldPosition(), false, intermediate1, targetPose);
                    sm.waitForSingleEvent(event, State.PLACE_PURPLE_PIXEL);
                    break;

                case PLACE_PURPLE_PIXEL:
                    // Place purple pixel on the spike mark position 1, 2 or 3.
                    if (robot.intake != null)
                    {
                        robot.intake.setOn(0.0, 1.0, event);
                        sm.waitForSingleEvent(event, State.DO_DELAY);
                    }
                    else
                    {
                        // Intake does not exist, skip placing purple pixel.
                        sm.setState(State.DO_DELAY);
                    }
                    break;

                case DO_DELAY:
                    // Do delay waiting for alliance partner to get out of the way if necessary.
                    if (autoChoices.delay == 0.0)
                    {
                        // No delay.
                        sm.setState(State.DRIVE_TO_LOOKOUT);
                    }
                    else
                    {
                        // Do delay.
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_LOOKOUT);
                    }
                    break;

                case DRIVE_TO_LOOKOUT:
                    // Drive to the lookout point where we can see the AprilTag clearly.
                    if (autoChoices.startPos == FtcAuto.StartPos.BACKSTAGE)
                    {
                        // Backstage starting position takes a shorter path to the backdrop.
                        intermediate1 = robot.adjustPoseByAlliance(0.5, 2.0, 180.0, autoChoices.alliance);
                        intermediate2 = robot.adjustPoseByAlliance(1.5, 2.0, -90.0, autoChoices.alliance);
                        targetPose = robot.adjustPoseByAlliance(1.5, 1.5, -90.0, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 4.5, robot.robotDrive.driveBase.getFieldPosition(), false,
                            intermediate1, intermediate2, targetPose);
                    }
                    else
                    {
                        // Audience starting position takes a longer path to the backdrop through the stage door.
                        intermediate1 = robot.adjustPoseByAlliance(-1.5, 2.3, 180.0, autoChoices.alliance);
                        intermediate2 = robot.adjustPoseByAlliance(1.5, 2.3, 180.0, autoChoices.alliance);
                        targetPose = robot.adjustPoseByAlliance(1.5, 1.5, -90.0, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.getYPosPidCtrl().setOutputLimit(0.5);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 9.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            intermediate1, intermediate2, targetPose);
                    }
                    sm.waitForSingleEvent(event, State.FIND_APRILTAG);
                    break;

                case FIND_APRILTAG:
                    // Use vision to determine the appropriate AprilTag location.
                    // As an optimization for time, we could skip using vision to look for AprilTag. We have absolute
                    // locations of all the AprilTag and we have absolute odometry, so we could navigate the robot
                    // there with just odometry.
                    robot.robotDrive.purePursuitDrive.getYPosPidCtrl().setOutputLimit(1.0);
                    if (autoChoices.useAprilTagVision && robot.vision != null && robot.vision.aprilTagVision != null)
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
                        // Not using vision or AprilTag Vision is not enabled, moving on.
                        robot.globalTracer.traceInfo(moduleName, "AprilTag Vision not enabled.");
                        sm.setState(State.DRIVE_TO_APRILTAG);
                    }
                    break;

                case DRIVE_TO_APRILTAG:
                    // Navigate robot to Apriltag.
                    if (aprilTagPose == null)
                    {
                        // Not using vision or vision did not see AprilTag, just go to it blindly using odometry and
                        // its known location.
                        aprilTagPose = RobotParams.APRILTAG_POSES[aprilTagId - 1];
                        robot.globalTracer.traceInfo(
                            moduleName, "Drive to AprilTag using absolute odometry (pose=%s).", aprilTagPose);
                    }
                    // Account for end-effector offset from the camera.
                    aprilTagPose.x -= 2.0;
                    aprilTagPose.angle = -90.0;
                    robot.robotDrive.purePursuitDrive.start(
                        event, 2.5,  robot.robotDrive.driveBase.getFieldPosition(), false, aprilTagPose);
                    sm.addEvent(event);
                    if (robot.elevatorArm != null)
                    {
                        // Set ElevatorArm to scoring position level 1.
                        robot.elevatorArm.setScoringPosition(
                            null, 0.0, RobotParams.ELEVATOR_LEVEL1_POS, elevatorArmEvent, 2.0);
                        sm.addEvent(elevatorArmEvent);
                    }
                    sm.waitForEvents(State.LOWER_ELEVATOR, true);
                    break;

                case LOWER_ELEVATOR:
                    if (robot.elevatorArm != null)
                    {
                        // Lower elevator to the lowest height to minimize pixel bouncing off.
                        robot.elevatorArm.elevatorSetPosition(
                            null, 0.0, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ELEVATOR_POWER_LIMIT,
                            elevatorArmEvent, 2.0);
                        sm.waitForSingleEvent(elevatorArmEvent, State.PLACE_YELLOW_PIXEL);
                    }
                    else
                    {
                        sm.setState(State.PLACE_YELLOW_PIXEL);
                    }
                    break;

                case PLACE_YELLOW_PIXEL:
                    // Place yellow pixel at the appropriate location on the backdrop.
                    if (robot.pixelTray != null)
                    {
                        robot.pixelTray.setLowerGateOpened(true, event);
                        robot.pixelTray.setUpperGateOpened(true, null);
                        sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    }
                    else
                    {
                        // PixelTray does not exist, moving on.
                        sm.setState(State.RAISE_ELEVATOR);
                    }
                    break;

                case RAISE_ELEVATOR:
                    if (robot.elevatorArm != null)
                    {
                        // Raise elevator back to the height that we can safely retract everything.
                        robot.elevatorArm.elevatorSetPosition(
                            null, 0.0, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ELEVATOR_POWER_LIMIT,
                            elevatorArmEvent, 5.0);
                        sm.waitForSingleEvent(elevatorArmEvent, State.RETRACT_ALL);
                    }
                    else
                    {
                        sm.setState(State.RETRACT_ALL);
                    }
                    break;

                case RETRACT_ALL:
                    // Retract everything.
                    if (robot.elevatorArm != null)
                    {
                        robot.elevatorArm.setLoadingPosition(null, 0.0, elevatorArmEvent, 0.0);
                        sm.waitForSingleEvent(event, State.PARK_AT_BACKSTAGE);
                    }
                    else
                    {
                        sm.setState(State.PARK_AT_BACKSTAGE);
                    }
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
