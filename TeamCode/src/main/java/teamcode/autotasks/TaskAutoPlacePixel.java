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

import java.util.Locale;

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
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.autocommands.CmdAuto;

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
        PLACE_PIXEL,
        DONE
    }   //enum State

//    private static class TaskParams
//    {
//        Vision.PixelType pixelType;
//        TaskParams(Vision.PixelType pixelType)
//        {
//            this.pixelType = pixelType;
//        }   //TaskParams
//    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;

    private TrcPose2D aprilTagPose = null;
    private Double visionExpiredTime = null;
    private Integer aprilTagId = null;
    private String currOwner = null;

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
        event = new TrcEvent(moduleName);
    }   //TaskAuto

    /**
     * This method starts the auto-assist place operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPlace(TrcEvent completionEvent, FtcAuto.Alliance alliance)
    {
        final String funcName = "autoAssistPlace";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: event=%s", moduleName, completionEvent);
        }
        aprilTagId = (alliance == FtcAuto.Alliance.BLUE_ALLIANCE ? RobotParams.BLUE_BACKDROP_APRILTAGS[1] : RobotParams.RED_BACKDROP_APRILTAGS[1]);
        startAutoTask(State.START, null, completionEvent);
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
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

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
        switch (state)
        {
            case START:
                sm.setState(State.FIND_APRILTAG);
                break;
            case FIND_APRILTAG:
                // Set up elevator and arm for placing pixel on the Backdrop.
//                if (robot.elevatorArm != null)
//                {
//                    robot.elevatorArm.setupPositions(
//                            null, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ARM_SCORE_BACKDROP_POS);
//                }
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
//                        msg = String.format(
//                                Locale.US, "AprilTag %d found at x %.1f, y %.1f",
//                                aprilTagId, aprilTagInfo.objPose.x, aprilTagInfo.objPose.y);
//                        robot.dashboard.displayPrintf(1, "%s", msg);
//                        robot.speak(msg);
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
                aprilTagPose.x -= 6.0;
                aprilTagPose.angle = -90.0;
                robot.robotDrive.purePursuitDrive.start(
                        event, 3.0,  robot.robotDrive.driveBase.getFieldPosition(), false, aprilTagPose);
//                robot.elevatorArm.setupPositions(null, RobotParams.ELEVATOR_LEVEL1_POS, RobotParams.ARM_SCORE_BACKDROP_POS);
                sm.waitForSingleEvent(event, State.PLACE_PIXEL);
                break;
            case PLACE_PIXEL:
                // Place yellow pixel at the appropriate location on the backdrop.
                if (robot.pixelTray != null)
                {
                    robot.pixelTray.setLowerGateOpened(true, event);
                    robot.pixelTray.setUpperGateOpened(true, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
