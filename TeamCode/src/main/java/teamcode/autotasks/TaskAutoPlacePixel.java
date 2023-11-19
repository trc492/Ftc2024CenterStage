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

package teamcode.autotasks;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcVisionAprilTag;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.BlinkinLEDs;

/**
 * This class implements auto-assist place pixel task.
 */
public class TaskAutoPlacePixel extends TrcAutoTask<TaskAutoPlacePixel.State>
{
    private static final String moduleName = "TaskAutoPlacePixel";

    public enum State
    {
        START,
        FIND_APRILTAG,
        DRIVE_TO_APRILTAG,
        LOWER_ELEVATOR,
        PLACE_PIXEL,
        RAISE_ELEVATOR,
        RETRACT_ALL,
        DONE
    }   //enum State

    private static class TaskParams
    {
        int aprilTagId;
        double scoreLevel;
        boolean inAuto;

        TaskParams(int aprilTagId, double scoreLevel, boolean inAuto)
        {
            this.aprilTagId = aprilTagId;
            this.scoreLevel = scoreLevel;
            this.inAuto = inAuto;
        }   //TaskParams

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;
    private final TrcEvent elevatorArmEvent;

    private String currOwner = null;
    private TrcPose2D aprilTagPose = null;
    private Double visionExpiredTime = null;
    private Double relocalizeDeltaHeading = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskAutoPlacePixel(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName + ".event");
        elevatorArmEvent = new TrcEvent(moduleName + ".elevatorArmEvent");
    }   //TaskAuto

    /**
     * This method starts the auto-assist place operation.
     *
     * @param aprilTagId specifies the AprilTag ID to approach for placing pixel for Autonomous, null if called by
     *        TeleOp.
     * @param scoreLevel specifies the score level.
     * @param inAuto specifies true if called by Autonomous, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPlace(int aprilTagId, double scoreLevel, boolean inAuto, TrcEvent completionEvent)
    {
        final String funcName = "autoAssistPlace";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: aprilTagId=%d, scoreLevel=%f, inAuto=%s, event=%s",
                moduleName, aprilTagId, scoreLevel, inAuto, completionEvent);
        }

        relocalizeDeltaHeading = null;
        startAutoTask(State.START, new TaskParams(aprilTagId, scoreLevel, inAuto), completionEvent);
    }   //autoAssistPlace

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist.", moduleName);
        }

        stopAutoTask(false);
    }   //autoAssistCancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        final String funcName = "acquireSubsystemsOwnership";
        boolean success = ownerName == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                           robot.elevatorArm.acquireExclusiveAccess(ownerName));

        if (success)
        {
            currOwner = ownerName;
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Successfully acquired subsystem ownerships.", moduleName);
            }
        }
        else
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "%s: Failed to acquire subsystem ownership (currOwner=%s, robotDrive=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase));
            }
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        final String funcName = "releaseSubsystemsOwnership";

        if (ownerName != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "%s: Releasing subsystem ownership (currOwner=%s, robotDrive=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase));
            }

            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.elevatorArm.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        final String funcName = "stopSubsystems";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Stopping subsystems.", moduleName);
        }

        robot.robotDrive.cancel(currOwner);
        robot.elevatorArm.cancel(currOwner);

        if (robot.vision != null && robot.vision.aprilTagVision != null)
        {
            robot.vision.setAprilTagVisionEnabled(false);
        }

        if (relocalizeDeltaHeading != null)
        {
            TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
            robotPose.angle -= relocalizeDeltaHeading;
            robot.robotDrive.driveBase.setFieldPosition(robotPose);
            robot.globalTracer.traceInfo(funcName, "Restoring robot heading to %s.", robotPose);
        }
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "runTaskState";
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                if (robot.vision != null && robot.vision.aprilTagVision != null)
                {
                    // Set up vision: switch to rear camera and enable AprilTagVision.
                    robot.vision.setActiveWebcam(robot.vision.getRearWebcam());
                    robot.vision.setAprilTagVisionEnabled(true);
                }
                sm.setState(State.FIND_APRILTAG);
                //
                // Intentionally falling through to the next state.
                //
            case FIND_APRILTAG:
                // Use vision to determine the appropriate AprilTag location.
                // As an optimization for time, we could skip using vision to look for AprilTag. We have absolute
                // locations of all the AprilTag and we have absolute odometry, so we could navigate the robot
                // there with just odometry.
                if (robot.vision != null && robot.vision.aprilTagVision != null)
                {
                    TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
                        robot.vision.getDetectedAprilTag(taskParams.aprilTagId, -1);
                    if (aprilTagInfo != null)
                    {
                        if (!taskParams.inAuto)
                        {
                            // Do re-localization only if we are not in Autonomous.
                            double prevRobotHeading = robot.robotDrive.driveBase.getHeading();
                            TrcPose2D robotFieldPose = robot.vision.getRobotFieldPose(aprilTagInfo);
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            relocalizeDeltaHeading = robot.robotDrive.driveBase.getHeading() - prevRobotHeading;
                        }
                        // Determine the absolute field location of the AprilTag.
                        aprilTagPose =
                            robot.robotDrive.driveBase.getFieldPosition().subtractRelativePose(
                                new TrcPose2D(
                                    aprilTagInfo.objPose.x, aprilTagInfo.objPose.y,
                                    -90.0 - robot.robotDrive.driveBase.getHeading()));
                        robot.globalTracer.traceInfo(
                            funcName, "AprilTag %d found at %s (absPose=%s)",
                            taskParams.aprilTagId, aprilTagInfo.objPose, aprilTagPose);
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
                        robot.globalTracer.traceInfo(funcName, "AprilTag %d not found.", taskParams.aprilTagId);
                        sm.setState(State.DRIVE_TO_APRILTAG);
                    }
                }
                else
                {
                    // Not using vision or AprilTag Vision is not enabled, moving on.
                    robot.globalTracer.traceInfo(funcName, "AprilTag Vision not enabled.");
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case DRIVE_TO_APRILTAG:
                // Navigate robot to Apriltag.
                if (aprilTagPose == null)
                {
                    // Not using vision or vision did not see AprilTag, just go to it blindly using odometry and
                    // its known location. If we are not in Autonomous, the robot may not know its location. In
                    // this case, we just warn the driver and quit.
                    if (taskParams.inAuto)
                    {
                        aprilTagPose = RobotParams.APRILTAG_POSES[taskParams.aprilTagId - 1];
                        robot.globalTracer.traceInfo(
                            funcName, "Drive to AprilTag using absolute odometry (pose=%s).", aprilTagPose);
                    }
                    else if (robot.blinkin != null)
                    {
                        // Tell the drivers vision doesn't see anything so they can score manually.
                        robot.blinkin.setDetectedPattern(BlinkinLEDs.DETECTED_NOTHING);
                    }
                }

                if (aprilTagPose != null)
                {
                    // We are right in front of the backdrop, so we don't need full power to approach it.
                    robot.robotDrive.purePursuitDrive.getYPosPidCtrl().setOutputLimit(0.5);
                    // Account for end-effector offset from the camera.
                    aprilTagPose.x -= 2.0;
                    aprilTagPose.angle = -90.0;
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false, aprilTagPose);
                    sm.addEvent(event);
                    if (robot.elevatorArm != null)
                    {
                        // Set ElevatorArm to scoring position level 1.
                        robot.elevatorArm.setScoringPosition(
                            currOwner, 0.0, RobotParams.ELEVATOR_LEVEL1_POS, elevatorArmEvent, 0.0);
                        sm.addEvent(elevatorArmEvent);
                    }
                    sm.waitForEvents(State.LOWER_ELEVATOR, true);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case LOWER_ELEVATOR:
                robot.robotDrive.purePursuitDrive.getYPosPidCtrl().setOutputLimit(1.0);
                if (robot.elevatorArm != null)
                {
                    // Lower elevator to the lowest height to minimize pixel bouncing off.
                    robot.elevatorArm.elevatorSetPosition(
                        currOwner, 0.0, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ELEVATOR_POWER_LIMIT,
                        elevatorArmEvent, 0.0);
                    sm.waitForSingleEvent(elevatorArmEvent, State.PLACE_PIXEL);
                }
                else
                {
                    sm.setState(State.PLACE_PIXEL);
                }
                break;

            case PLACE_PIXEL:
                // Place pixel at the appropriate location on the backdrop.
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
                        currOwner, 0.0, RobotParams.ELEVATOR_LEVEL2_POS, RobotParams.ELEVATOR_POWER_LIMIT,
                        elevatorArmEvent, 0.0);
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
                    sm.waitForSingleEvent(elevatorArmEvent, State.DONE);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPlacePixel
