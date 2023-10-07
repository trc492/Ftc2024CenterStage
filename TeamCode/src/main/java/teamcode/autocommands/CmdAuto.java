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
        DRIVE_TO_BACKDROP,
        DETERMINE_APRILTAG_POSE,
        ALIGN_TO_APRILTAG,
        PLACE_YELLOW_PIXEL,
        PARK_AT_BACKSTAGE,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int teamPropIndex = 0;
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
            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START:
                    int teamPropPos = 0;
                    String msg;
                    // Set robot's start position on the field.
                    robot.robotDrive.setAutoStartPosition(autoChoices);
                    // Use vision to determine team prop position (0, 1, 2, 3).
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
                        // We still can't see the team prop, set to default position.
                        teamPropPos = 2;
                        msg = "No team prop found, default to position " + teamPropPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    teamPropIndex = teamPropPos - 1;
                    // Navigate robot to spike mark 1, 2 or 3.
                    TrcPose2D spikeMarkPose =
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                                RobotParams.BLUE_AUDIENCE_SPIKE_MARKS[teamPropIndex]:
                                RobotParams.BLUE_BACKSTAGE_SPIKE_MARKS[teamPropIndex]:
                            autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                                RobotParams.RED_AUDIENCE_SPIKES[teamPropIndex]:
                                RobotParams.RED_BACKSTAGE_SPIKES[teamPropIndex];
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false, spikeMarkPose);
                    sm.waitForSingleEvent(event, State.PLACE_PURPLE_PIXEL);
                    break;

                case PLACE_PURPLE_PIXEL:
                    // Place purple pixel on the spike position 1, 2 or 3.
                    sm.setState(State.DO_DELAY);
                    break;

                case DO_DELAY:
                    // Do delay waiting for alliance partner to get out of the way if necessary.
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.DRIVE_TO_BACKDROP);
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_BACKDROP);
                    }
                    break;

                case DRIVE_TO_BACKDROP:
                    // Navigate robot to backdrop.
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            RobotParams.BLUE_BACKDROP: RobotParams.RED_BACKDROP);
                    sm.waitForSingleEvent(event,State.DETERMINE_APRILTAG_POSE);
                    break;

                case DETERMINE_APRILTAG_POSE:
                    // Use vision to determine the appropriate AprilTag location.
                    if (robot.vision != null && robot.vision.aprilTagVision != null)
                    {
                        int aprilTagId =
                            autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                                RobotParams.BLUE_BACKDROP_APRILTAGS[teamPropIndex]:
                                RobotParams.RED_BACKDROP_APRILTAGS[teamPropIndex];
                        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
                            robot.vision.getDetectedAprilTag(aprilTagId, -1);
                        if (aprilTagInfo != null)
                        {
                            // Account for grabber offset from the camera.
                            aprilTagPose = new TrcPose2D(
                                aprilTagInfo.objPose.x, aprilTagInfo.objPose.y - 6.0,
                                90.0 - robot.robotDrive.driveBase.getHeading());
                            robot.globalTracer.traceInfo(
                                moduleName, "AprilTag %d found at %s", aprilTagId, aprilTagPose);
                            msg = String.format(
                                Locale.US, "AprilTag %d found at x %.1f, y %.1f",
                                aprilTagId, aprilTagInfo.objPose.x, aprilTagInfo.objPose.y);
                            robot.dashboard.displayPrintf(3, "%s", msg);
                            robot.speak(msg);
                            sm.setState(State.ALIGN_TO_APRILTAG);
                        }
                        else if (visionExpiredTime == null)
                        {
                            // Can't find AprilTag, set a timeout and try again.
                            visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                        }
                        else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                        {
                            // Timing out, moving on.
                            robot.globalTracer.traceInfo(moduleName, "AprilTag not found.");
                            sm.setState(State.ALIGN_TO_APRILTAG);
                        }
                    }
                    else
                    {
                        // Vision is not enable, move on.
                        robot.globalTracer.traceInfo(moduleName, "AprilTag Vision not enabled.");
                        sm.setState(State.ALIGN_TO_APRILTAG);
                    }
                    break;

                case ALIGN_TO_APRILTAG:
                    // Navigate robot to the AprilTag location.
                    if (aprilTagPose == null)
                    {
                        // Vision does not see AprilTag, just move forward.
                        aprilTagPose = new TrcPose2D(0.0, 4.0, 0.0);
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true, aprilTagPose);
                    sm.waitForSingleEvent(event,State.PLACE_YELLOW_PIXEL);
                    break;

                case PLACE_YELLOW_PIXEL:
                    // Place yellow pixel at the appropriate location on the backdrop.
                    sm.setState(State.PARK_AT_BACKSTAGE);
                    break;

                case PARK_AT_BACKSTAGE:
                    // Navigate robot to the backstage location.
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            autoChoices.parkPos == FtcAuto.ParkPos.CORNER?
                                RobotParams.PARKPOS_BLUE_CORNER: RobotParams.PARKPOS_BLUE_CENTER:
                            autoChoices.parkPos == FtcAuto.ParkPos.CORNER?
                                RobotParams.PARKPOS_RED_CORNER: RobotParams.PARKPOS_RED_CENTER);
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
