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
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import teamcode.Robot;
import teamcode.vision.Vision;

/**
 * This class implements auto-assist pickup pixel task.
 */
public class TaskAutoPickupPixel extends TrcAutoTask<TaskAutoPickupPixel.State>
{
    private static final String moduleName = "TaskAutoPickupPixel";

    public enum State
    {
        START,
        FIND_PIXEL,
        ALIGN_TO_PIXEL,
        PICK_UP_PIXEL,
        DONE
    }   //enum State

    private static class TaskParams
    {
        Vision.PixelType pixelType;
        TaskParams(Vision.PixelType pixelType)
        {
            this.pixelType = pixelType;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;

    private String currOwner = null;
    private TrcPose2D pixelPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskAutoPickupPixel(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskAutoPickupPixel

    /**
     * This method starts the auto-assist pickup operation.
     *
     * @param pixelType specifies the pixel type to look for and pick up.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(Vision.PixelType pixelType, TrcEvent completionEvent)
    {
        final String funcName = "autoAssistPickup";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: pixelType=%s, event=%s", moduleName, pixelType, completionEvent);
        }

        startAutoTask(State.START, new TaskParams(pixelType), completionEvent);
    }   //autoAssistPickup

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

        if (robot.vision != null)
        {
            robot.vision.setPixelVisionEnabled(Vision.PixelType.AnyPixel, false);
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
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                // Set up robot.
                if (robot.vision != null)
                {
                    robot.vision.setPixelVisionEnabled(Vision.PixelType.AnyPixel, true);
                }
                break;

            case FIND_PIXEL:
                // Use vision to locate pixel.
                if (robot.vision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo =
                        robot.vision.getDetectedPixel(taskParams.pixelType, -1);
                    if (pixelInfo != null)
                    {
                        pixelPose = new TrcPose2D(
                            pixelInfo.objPose.x, pixelInfo.objPose.y - 6.0, pixelInfo.objPose.yaw);
                        String msg = String.format(
                            Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f",
                            taskParams.pixelType, pixelInfo.objPose.x, pixelInfo.objPose.y, pixelInfo.objPose.yaw);
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.dashboard.displayPrintf(3, "%s", msg);
                        robot.speak(msg);
                        sm.setState(State.ALIGN_TO_PIXEL);
                    }
                    else if (visionExpiredTime == null)
                    {
                        // Can't find any pixel, set a timeout and try again.
                        visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                    }
                    else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                    {
                        // Timing out, moving on.
                        sm.setState(State.DONE);
                    }
                }
                else
                {
                    // Vision is not enable, move on.
                    sm.setState(State.DONE);
                }
                break;

            case ALIGN_TO_PIXEL:
                // Navigate robot to the pixel.
                if (pixelPose != null)
                {
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true, pixelPose);
                    sm.waitForSingleEvent(event, State.PICK_UP_PIXEL);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case PICK_UP_PIXEL:
                // Pick up pixel.
                robot.intake.pickUp(0.0, 6.0 , event);

                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                robot.pixelTray.setUpperGateOpened(false, null);
                if (robot.vision != null)
                {
                    robot.vision.setPixelVisionEnabled(Vision.PixelType.AnyPixel, false);
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupPixel
