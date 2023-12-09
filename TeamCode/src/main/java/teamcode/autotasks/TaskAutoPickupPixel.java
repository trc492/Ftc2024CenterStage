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
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import teamcode.Robot;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.vision.Vision;

/**
 * This class implements auto-assist pickup pixel task.
 */
public class TaskAutoPickupPixel extends TrcAutoTask<TaskAutoPickupPixel.State>
{
    private static final String moduleName = TaskAutoPickupPixel.class.getSimpleName();

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
    private final TrcEvent event;

    private String currOwner = null;
    private Vision.PixelType pixelType = null;
    private TrcPose2D pixelPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupPixel(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
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
        tracer.traceInfo(moduleName, "pixelType=%s, event=%s", pixelType, completionEvent);
        this.pixelType = pixelType;
        startAutoTask(State.START, new TaskParams(pixelType), completionEvent);
    }   //autoAssistPickup

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        tracer.traceInfo(moduleName, "Canceling auto-assist.");
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
        boolean success = ownerName == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

        if (success)
        {
            currOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName, "Failed to acquire subsystem ownership (currOwner=%s, robotDrive=%s).",
                currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase));
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
        if (ownerName != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName, "Releasing subsystem ownership (currOwner=%s, robotDrive=%s).",
                currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase));
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
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.robotDrive.cancel(currOwner);
        robot.intake.stop();

        if (robot.vision != null && pixelType != null)
        {
            robot.vision.setPixelVisionEnabled(pixelType, false);
            pixelType = null;
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
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                // Set up robot.
                pixelPose = null;
                if (robot.vision != null)
                {
                    // Set up vision: turn on front camera and enable pixel detection pipeline.
                    robot.vision.setActiveWebcam(robot.vision.getFrontWebcam());
                    robot.vision.setPixelVisionEnabled(taskParams.pixelType, true);
                    if (robot.elevatorArm != null)
                    {
                        // Make sure the ElevatorArm is at loading position.
                        robot.elevatorArm.setLoadingPosition(currOwner, 0.0, event, 0.0);
                        sm.waitForSingleEvent(event, State.FIND_PIXEL);
                    }
                    else
                    {
                        sm.setState(State.FIND_PIXEL);
                    }
                }
                else
                {
                    // Vision is not enabled, there is nothing to do.
                    sm.setState(State.DONE);
                }
                break;

            case FIND_PIXEL:
                // Use vision to locate pixel.
                if (robot.vision.isPixelVisionEnabled(taskParams.pixelType))
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
                        tracer.traceInfo(moduleName, msg);
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
                        tracer.traceInfo(moduleName, "%s not found.", taskParams.pixelType);
                        if (robot.blinkin != null)
                        {
                            // Tell the drivers vision doesn't see anything so they can score manually.
                            robot.blinkin.setDetectedPattern(BlinkinLEDs.DETECTED_NOTHING);
                        }

                        sm.setState(State.DONE);
                    }
                }
                else
                {
                    // Vision is not enable, moving on.
                    tracer.traceInfo(moduleName, "Vision not enabled.");
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
                robot.intake.setOn(0.0, 6.0 , event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                robot.pixelTray.setUpperGateOpened(false, null);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupPixel
