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

import ftclib.vision.FtcVisionAprilTag;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.BlinkinLEDs;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;
import trclib.dataprocessor.TrcTrigger;
import trclib.dataprocessor.TrcTriggerThresholdZones;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist place pixel task.
 */
public class TaskAutoPlacePixel extends TrcAutoTask<TaskAutoPlacePixel.State>
{
    private static final String moduleName = TaskAutoPlacePixel.class.getSimpleName();

    public enum State
    {
        START,
        FIND_APRILTAG,
        SET_SCORING_POS,
        DRIVE_TO_APRILTAG,
        LOWER_ELEVATOR,
        PLACE_PIXEL_1,
        DRIVE_TO_POS_2,
        PLACE_PIXEL_2,
        RAISE_ELEVATOR,
        RETRACT_ALL,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;
        int aprilTagIndex;
        boolean hasSecondPixel;
        double scoreLevel;
        boolean inAuto;

        TaskParams(boolean useVision, int aprilTagIndex, boolean hasSecondPixel, double scoreLevel, boolean inAuto)
        {
            this.useVision = useVision;
            this.aprilTagIndex = aprilTagIndex;
            this.hasSecondPixel = hasSecondPixel;
            this.scoreLevel = scoreLevel;
            this.inAuto = inAuto;
        }   //TaskParams

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent elevatorArmEvent;

    private String currOwner = null;
    private TrcPose2D aprilTagPose = null;
    private TrcPose2D adjAprilTagPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPlacePixel(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName + ".event");
        elevatorArmEvent = new TrcEvent(moduleName + ".elevatorArmEvent");
    }   //TaskAutoPlacePixel

    /**
     * This method starts the auto-assist place operation.
     *
     * @param useVision specifies true to use vision, false otherwise.
     * @param aprilTagIndex specifies the AprilTag index to approach for placing pixel, 0 for left, 1 for center and
     *        2 for right.
     * @param hasSecondPixel specifies true if scoring 2 pixels, false otherwise.
     * @param scoreLevel specifies the score level.
     * @param inAuto specifies true if called by Autonomous, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPlace(
        boolean useVision, int aprilTagIndex, boolean hasSecondPixel, double scoreLevel, boolean inAuto,
        TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName, "useVision=%s, aprilTagIndex=%d, has2Pixels=%s, scoreLevel=%f, inAuto=%s, event=%s",
            useVision, aprilTagIndex, hasSecondPixel, scoreLevel, inAuto, completionEvent);
        startAutoTask(
            State.START, new TaskParams(useVision, aprilTagIndex, hasSecondPixel, scoreLevel, inAuto), completionEvent);
    }   //autoAssistPlace

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
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                           robot.elevatorArm.acquireExclusiveAccess(ownerName));

        if (success)
        {
            currOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceWarn(
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
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
        if (robot.elevatorArm != null && robot.elevatorArm.wristTrigger != null)
        {
            robot.elevatorArm.wristTrigger.disableTrigger();
        }

        robot.robotDrive.cancel(currOwner);
        robot.elevatorArm.cancel(currOwner);

        if (robot.vision != null && robot.vision.aprilTagVision != null)
        {
            robot.vision.setAprilTagVisionEnabled(false);
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
                aprilTagPose = null;
                if (robot.pixelTray != null)
                {
                    robot.pixelTray.setUpperGateOpened(false, null);
                    robot.pixelTray.setLowerGateOpened(false, null);
                }

                if (taskParams.useVision && robot.vision != null && robot.vision.aprilTagVision != null)
                {
                    // Set up vision: switch to rear camera and enable AprilTagVision.
                    robot.vision.setActiveWebcam(robot.vision.getRearWebcam());
                    robot.vision.setAprilTagVisionEnabled(true);
                    tracer.traceInfo(moduleName, "Enabling AprilTagVision.");
                    sm.setState(State.FIND_APRILTAG);
                }
                else
                {
                    // Not using vision or AprilTag Vision is not enabled, skip vision.
                    tracer.traceInfo(moduleName, "AprilTag Vision not enabled.");
                    sm.setState(State.SET_SCORING_POS);
                }
                break;

            case FIND_APRILTAG:
                // Use vision to determine the appropriate AprilTag location.
                // As an optimization for time, we could skip using vision to look for AprilTag. We have absolute
                // locations of all the AprilTag and we have absolute odometry, so we could navigate the robot
                // there with just odometry.
                // Look for any AprilTag in front of us.
                TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
                    robot.vision.getDetectedAprilTag(null, -1);
                if (aprilTagInfo != null && aprilTagInfo.detectedObj.aprilTagDetection.id < 7)
                {
                    // If we see the AprilTag, we can use its location to re-localize the robot. It's especially
                    // useful if we started on the audience side where we traveled a great distance to the backdrop
                    // and odometry may cumulate some amount of error.
                    TrcPose2D robotFieldPose = robot.vision.getRobotFieldPose(aprilTagInfo);
                    robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                    tracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + robotFieldPose);
                    // Determine the absolute field location of the AprilTag.
                    FtcAuto.Alliance alliance =
                        aprilTagInfo.detectedObj.aprilTagDetection.id < 4 ?
                            FtcAuto.Alliance.BLUE_ALLIANCE : FtcAuto.Alliance.RED_ALLIANCE;
                    int targetAprilTagId =
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE ?
                            RobotParams.BLUE_BACKDROP_APRILTAGS[taskParams.aprilTagIndex] :
                            RobotParams.RED_BACKDROP_APRILTAGS[taskParams.aprilTagIndex];
                    aprilTagPose = RobotParams.APRILTAG_POSES[targetAprilTagId - 1];
                    tracer.traceInfo(
                        moduleName,
                        "Vision found AprilTag %d at %s from camera.\n" +
                        "\tTargeting AprilTag %d at %s from robotPose %s.",
                        aprilTagInfo.detectedObj.aprilTagDetection.id, aprilTagInfo.objPose,
                        targetAprilTagId, aprilTagPose, robot.robotDrive.driveBase.getFieldPosition());
                    sm.setState(State.SET_SCORING_POS);
                }
                else
                {
                    if (aprilTagInfo != null)
                    {
                        // For some reason, we found an AprilTag but not in the range of 1-6. Ignore it as if we did
                        // not see it.
                        tracer.traceWarn(moduleName, "Not seeing backdrop!!! (obj=%s)", aprilTagInfo.detectedObj);
                    }

                    if (visionExpiredTime == null)
                    {
                        // Can't find AprilTag, set a timeout and try again.
                        visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                    }
                    else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                    {
                        // Timed out, moving on.
                        tracer.traceInfo(moduleName, "No AprilTag found.");
                        sm.setState(State.SET_SCORING_POS);
                    }
                }
                break;

            case SET_SCORING_POS:
                if (robot.elevatorArm != null)
                {
                    // Set ElevatorArm to scoring position.
                    double elevatorHeight = Math.max(taskParams.scoreLevel, RobotParams.ELEVATOR_LEVEL1_POS);
                    robot.elevatorArm.setScoringPosition(currOwner, 0.0, elevatorHeight, elevatorArmEvent, 4.0);
                    sm.waitForSingleEvent(elevatorArmEvent, State.DRIVE_TO_APRILTAG);
                }
                else
                {
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case DRIVE_TO_APRILTAG:
                // Navigate robot to Apriltag.
                tracer.traceInfo(
                    moduleName, state + ": RobotFieldPose=" + robot.robotDrive.driveBase.getFieldPosition());
                if (aprilTagPose == null)
                {
                    // Not using vision or vision did not see AprilTag, just go to it blindly using odometry and
                    // its known location. If we are not in Autonomous, the robot may not know its location. In
                    // this case, we just warn the driver and quit.
                    if (taskParams.inAuto)
                    {
                        int targetAprilTagId =
                            FtcAuto.autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE ?
                                RobotParams.BLUE_BACKDROP_APRILTAGS[taskParams.aprilTagIndex] :
                                RobotParams.RED_BACKDROP_APRILTAGS[taskParams.aprilTagIndex];
                        aprilTagPose = RobotParams.APRILTAG_POSES[targetAprilTagId - 1];
                        tracer.traceInfo(
                            moduleName, "Drive to AprilTag %d using absolute odometry (pose=%s).",
                            targetAprilTagId, aprilTagPose);
                    }
                    else if (robot.blinkin != null)
                    {
                        // Tell the drivers vision doesn't see anything so they can score manually.
                        robot.blinkin.setDetectedPattern(BlinkinLEDs.DETECTED_NOTHING);
                    }
                }

                if (aprilTagPose != null)
                {
                    if (robot.elevatorArm != null && robot.elevatorArm.wristTrigger != null)
                    {
                        robot.elevatorArm.wristTrigger.enableTrigger(
                            TrcTrigger.TriggerMode.OnBoth, this::wristSensorTriggered);
                    }
                    // Account for end-effector offset from the camera.
                    // Clone aprilTagPose before changing it, or we will corrupt the AprilTag location array.
                    adjAprilTagPose = aprilTagPose.clone();
                    adjAprilTagPose.x -= 5.85;
                    if (taskParams.aprilTagIndex == 0)
                    {
                        adjAprilTagPose.y -= 0.65;
                    }
                    else if (taskParams.aprilTagIndex == 1)
                    {
                        adjAprilTagPose.y -= 0.75;
                    }
                    else
                    {
                        adjAprilTagPose.y += 0.65;
                    }
                    // Maintain heading to be squared to the backdrop.
                    adjAprilTagPose.angle = -90.0;
                    TrcPose2D targetPose = adjAprilTagPose.clone();
                    if (taskParams.hasSecondPixel)
                    {
                        // We have a second pixel, score it first. So, adjust the Y position for the second pixel.
                        double yDeltaUnit = taskParams.aprilTagIndex == 0 ? -2.0 : taskParams.aprilTagIndex;
                        targetPose.y += RobotParams.BACKDROP_APRILTAG_DELTA_Y * yDeltaUnit;
                    }
                    // We are right in front of the backdrop, so we don't need full power to approach it.
                    // Go sideways first so we can approach the backdrop straight forward at the end. Or we could
                    // stop the sideways movement prematurely if the distance sensor said it's close enough.
                    TrcPose2D intermediate = targetPose.clone();
                    intermediate.x = robot.robotDrive.driveBase.getXPosition();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.35);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 10.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        intermediate, targetPose);
                    sm.waitForSingleEvent(event, State.LOWER_ELEVATOR);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case LOWER_ELEVATOR:
                if (robot.elevatorArm != null && robot.elevatorArm.wristTrigger != null)
                {
                    robot.elevatorArm.wristTrigger.disableTrigger();
                }

                if (robot.elevatorArm != null)
                {
                    // Lower elevator to the lowest height to minimize pixel bouncing off.
                    robot.elevatorArm.elevatorSetPosition(
                        currOwner, 0.0, taskParams.scoreLevel, RobotParams.ELEVATOR_POWER_LIMIT,
                        elevatorArmEvent, 0.0);
                    sm.waitForSingleEvent(elevatorArmEvent, State.PLACE_PIXEL_1);
                }
                else
                {
                    sm.setState(State.PLACE_PIXEL_1);
                }
                break;

            case PLACE_PIXEL_1:
                // Place pixel at the appropriate location on the backdrop.
                if (robot.pixelTray != null)
                {
                    robot.pixelTray.setUpperGateOpened(true, event);
                    if (!taskParams.hasSecondPixel)
                    {
                        // We are scoring lower pixel, open the lower gate too.
                        robot.pixelTray.setLowerGateOpened(true, null);
                    }
                    sm.waitForSingleEvent(
                        event, taskParams.hasSecondPixel ? State.DRIVE_TO_POS_2 : State.RAISE_ELEVATOR);
                }
                else
                {
                    // PixelTray does not exist, moving on.
                    sm.setState(State.RAISE_ELEVATOR);
                }
                break;

            case DRIVE_TO_POS_2:
                // We just scored the first pixel, we are moving to the position to score the second pixel.
                TrcPose2D intermediate1 = robot.robotDrive.driveBase.getFieldPosition();
                TrcPose2D intermediate2 = adjAprilTagPose.clone();
                intermediate1.x -= 10.0;
                intermediate2.x -= 10.0;
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 10.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    intermediate1, intermediate2, adjAprilTagPose);
                sm.waitForSingleEvent(event, State.PLACE_PIXEL_2);
                break;

            case PLACE_PIXEL_2:
                // Place pixel at the appropriate location on the backdrop.
                if (robot.pixelTray != null)
                {
                    robot.pixelTray.setLowerGateOpened(true, event);
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

    /**
     * This method is called when the wrist sensor trigger event occurred.
     *
     * @param context specifies the
     */
    private void wristSensorTriggered(Object context)
    {
        TrcTriggerThresholdZones.CallbackContext callbackContext = (TrcTriggerThresholdZones.CallbackContext) context;

        if (callbackContext.prevZone == 1 && callbackContext.currZone == 0)
        {
            robot.robotDrive.purePursuitDrive.cancel(currOwner);
            tracer.traceInfo(
                moduleName, "Drive to AprilTag canceled by wristSensor, triggerState=%s, robotPose=%s",
                callbackContext, robot.robotDrive.driveBase.getFieldPosition());
        }
    }   //wristSensorTriggered

}   //class TaskAutoPlacePixel
