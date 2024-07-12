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

package teamcode.subsystems;

import java.util.ArrayList;
import java.util.Locale;

import ftclib.sensor.FtcDistanceSensor;
import ftclib.subsystem.FtcMotorActuator;
import ftclib.motor.FtcServo;
import ftclib.subsystem.FtcServoActuator;
import teamcode.RobotParams;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcSensor;
import trclib.timer.TrcTimer;
import trclib.dataprocessor.TrcTriggerThresholdZones;

public class ElevatorArm implements TrcExclusiveSubsystem
{
    private final String moduleName = ElevatorArm.class.getSimpleName();

    private enum ActionType
    {
        ElevatorSetPosition,
        ElevatorSetPower,
        ElevatorSetPidPower,
        ArmSetPosition,
        ArmSetPower,
        ArmSetPidPower
    }   //enum ActionType

    private static class ActionParams
    {
        ActionType actionType = null;
        TrcEvent actionEvent = null;
        double delay = 0.0;
        double pos = 0.0;
        double powerLimit = 1.0;
        double power = 0.0;
        double duration = 0.0;
        double minPos = 0.0;
        double maxPos = 0.0;
        TrcEvent completionEvent = null;
        double expiredTime = 0.0;

        void setPositionParams(
            ActionType actionType, TrcEvent actionEvent, double delay, double pos, double powerLimit,
            TrcEvent completionEvent, double expiredTime)
        {
            this.actionType = actionType;
            this.actionEvent = actionEvent;
            this.delay = delay;
            this.pos = pos;
            this.powerLimit = powerLimit;
            this.completionEvent = completionEvent;
            this.expiredTime = expiredTime;
        }   //setPositionParams

        void setPowerParams(
            ActionType actionType, TrcEvent actionEvent, double delay, double power, double duration,
            TrcEvent completionEvent)
        {
            this.actionType = actionType;
            this.actionEvent = actionEvent;
            this.delay = delay;
            this.power = power;
            this.duration = duration;
            this.completionEvent = completionEvent;
        }   //setPowerParams

        void setPidPowerParams(
            ActionType actionType, TrcEvent actionEvent, double power, double minPos, double maxPos)
        {
            this.actionType = actionType;
            this.actionEvent = actionEvent;
            this.power = power;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }   //setPidPowerParams

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "\n(\tactionType=%s\n" +
                "\tactionEvent=%s\n" +
                "\tdelay=%.3f\n" +
                "\tpos=%.1f\n" +
                "\tpowerLimit=%.1f\n" +
                "\tpower=%.1f\n" +
                "\tduration=%.1f\n" +
                "\tposRange=%.1f/%.1f\n" +
                "\tevent=%s\n" +
                "\texpiredTime=%.3f\n)",
                actionType, actionEvent, delay, pos, powerLimit, power, duration, minPos, maxPos, completionEvent,
                expiredTime);
        }   //toString

    }   //class ActionParams

    private final ArrayList<TrcEvent> pendingEventCallbacks = new ArrayList<>();
    private final TrcDbgTrace tracer;
    // Elevator subsystem.
    public final TrcMotor elevator;
    private double elevatorPrevPrintedPower = 0.0;
    // Arm subsystem.
    public final TrcMotor arm;
    private double armPrevPrintedPower = 0.0;
    // Wrist subsystem.
    public final FtcServo wrist;
    public final FtcDistanceSensor wristSensor;
    public final TrcTriggerThresholdZones wristTrigger;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ElevatorArm()
    {
        tracer = new TrcDbgTrace();
        // Elevator subsystem.
        if (RobotParams.Preferences.useElevator)
        {
            FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
                .setMotorInverted(RobotParams.ELEVATOR_MOTOR_INVERTED)
                .setLowerLimitSwitch(
                    RobotParams.ELEVATOR_HAS_LOWER_LIMIT_SWITCH,
                    RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED)
                .setUpperLimitSwitch(
                    RobotParams.ELEVATOR_HAS_UPPER_LIMIT_SWITCH,
                    RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED)
                .setVoltageCompensationEnabled(RobotParams.ELEVATOR_VOLTAGE_COMP_ENABLED)
                .setPositionScaleAndOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
                .setPositionPresets(RobotParams.ELEVATOR_PRESET_TOLERANCE, RobotParams.ELEVATOR_PRESETS);
            elevator =
                new FtcMotorActuator(RobotParams.HWNAME_ELEVATOR, elevatorParams).getActuator();
            elevator.setSoftwarePidEnabled(true);
            elevator.setPositionPidParameters(
                RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD, RobotParams.ELEVATOR_KF,
                RobotParams.ELEVATOR_IZONE, RobotParams.ELEVATOR_TOLERANCE);
            elevator.setPidStallDetectionEnabled(
                RobotParams.ELEVATOR_STALL_DETECTION_DELAY, RobotParams.ELEVATOR_STALL_DETECTION_TIMEOUT,
                RobotParams.ELEVATOR_STALL_ERR_RATE_THRESHOLD);
            elevator.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
//            elevator.resetPositionOnLowerLimitSwitch();
        }
        else
        {
            elevator = null;
        }
        // Arm subsystem.
        if (RobotParams.Preferences.useArm)
        {
            FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setMotorInverted(RobotParams.ARM_MOTOR_INVERTED)
                .setFollowerMotor(RobotParams.ARM_HAS_FOLLOWER_MOTOR, RobotParams.ARM_FOLLOWER_MOTOR_INVERTED)
                .setLowerLimitSwitch(RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED)
                .setUpperLimitSwitch(RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED)
                .setExternalEncoder(
                    RobotParams.ARM_HAS_EXTERNAL_ENCODER, RobotParams.ARM_ENCODER_INVERTED,
                    RobotParams.ARM_ENCODER_ABSOLUTE)
                .setVoltageCompensationEnabled(RobotParams.ARM_VOLTAGE_COMP_ENABLED)
                .setPositionScaleAndOffset(RobotParams.ARM_DEG_SCALE, RobotParams.ARM_OFFSET,
                                           RobotParams.ARM_ZERO_OFFSET)
                .setPositionPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.ARM_PRESETS);
            arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, true, armParams).getActuator();
            arm.setPositionPidParameters(
                RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_KF,
                RobotParams.ARM_IZONE, RobotParams.ARM_TOLERANCE);
            arm.setPositionPidTolerance(RobotParams.ARM_TOLERANCE);
            arm.setPositionPidPowerComp(this::armGetPowerComp);
            arm.setPidStallDetectionEnabled(
                RobotParams.ARM_STALL_DETECTION_DELAY, RobotParams.ARM_STALL_DETECTION_TIMEOUT,
                RobotParams.ARM_STALL_ERR_RATE_THRESHOLD);
            arm.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
        }
        else
        {
            arm = null;
        }
        // Wrist subsystem.
        if (RobotParams.Preferences.useWrist)
        {
            FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
                .setServoInverted(RobotParams.WRIST_SERVO_INVERTED)
                .setHasFollowerServo(RobotParams.WRIST_HAS_FOLLOWER_SERVO, RobotParams.WRIST_FOLLOWER_SERVO_INVERTED);

            wrist = new FtcServoActuator(RobotParams.HWNAME_WRIST, wristParams).getActuator();
            if (RobotParams.Preferences.hasWristSensor)
            {
                wristSensor = new FtcDistanceSensor(RobotParams.HWNAME_WRIST + ".sensor");
                wristTrigger = new TrcTriggerThresholdZones(
                    RobotParams.HWNAME_WRIST + ".analogTrigger", this::wristGetDistance,
                    RobotParams.WRIST_SENSOR_THRESHOLDS, false);
            }
            else
            {
                wristSensor = null;
                wristTrigger = null;
            }
            wrist.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.INFO);
        }
        else
        {
            wrist = null;
            wristSensor = null;
            wristTrigger = null;
        }
    }   //ElevatorArm

    /**
     * This method returns the module name.
     *
     * @return module name.
     */
    @Override
    public String toString()
    {
        return moduleName;
    }   //toString

    /**
     * This method cancels the elevator and arm operation if there is any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     */
    public void cancel(String owner)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        if (hasOwnership(owner))
        {
            elevator.cancel();
            arm.cancel();
            // Check if there are any pending event callbacks and remove them.
            synchronized (pendingEventCallbacks)
            {
                if (!pendingEventCallbacks.isEmpty())
                {
                    for (TrcEvent event: pendingEventCallbacks)
                    {
                        event.setCallback(null, null);
                    }
                    pendingEventCallbacks.clear();
                }
            }
            releaseExclusiveAccess(owner);
        }
    }   //cancel

    /**
     * This method cancels the elevator and arm operation if there is any.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method removes the event from the Pending Event Callback list if it is in there.
     *
     * @param event specifies the event to be removed from the list.
     * @return true if the event is removed, false otherwise.
     */
    private boolean removePendingEventCallback(TrcEvent event)
    {
        boolean removed = false;

        if (event != null)
        {
            synchronized (pendingEventCallbacks)
            {
                removed = pendingEventCallbacks.remove(event);
            }
        }

        return removed;
    }   //removePendingEventCallback

    /**
     * This method sets up the action callback to finish the action when it's safe to do so.
     *
     * @param actionParams specifies the action parameters to perform.
     * @param actionEvent specifies the event that signals the callback for completion.
     */
    private void setActionCallback(ActionParams actionParams, TrcEvent actionEvent)
    {
        actionEvent.setCallback(this::performAction, actionParams);
        pendingEventCallbacks.add(actionEvent);
    }   //setActionCallback

    /**
     * This method zero calibrates the elevator. Arm has absolute encoder and doesn't need zero calibration.
     * Note: This assumes the arm is at safe position or elevator zero calibration won't happen.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        cancel(owner);
        TrcEvent completionEvent = new TrcEvent(moduleName + ".zeroCalComplete");
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);

        if (validateOwnership(owner))
        {
            // Do zero calibration only if arm is tucked in.
            if (elevator != null && arm != null &&
                arm.getPosition() - RobotParams.ARM_TOLERANCE <= RobotParams.ARM_LOAD_POS)
            {
                if (wrist != null)
                {
                    // Holding wrist in loading position while zero calibrating.
                    wrist.setPosition(RobotParams.WRIST_DOWN_POS);
                }
                // Holding arm in loading position while zero calibrating.
                arm.setPosition(null, 0.0, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, null, 0.0);
                // Enable stall protection in case the limit switch is malfunctioning so we can still zero calibrate
                // on stall.
                elevator.setStallProtection(RobotParams.ELEVATOR_CAL_POWER, 0.1, 0.2, 0.5);
                completionEvent.setCallback(this::zeroCalCompleted, null);
                elevator.zeroCalibrate(
                    RobotParams.ELEVATOR_CAL_POWER,
                    releaseOwnershipEvent != null? releaseOwnershipEvent: completionEvent);
            }
        }
    }   //zeroCalibrate

    /**
     * This method is called after zero calibration is done so we can turn off stall protection.
     *
     * @param context not used.
     */
    private void zeroCalCompleted(Object context)
    {
        elevator.setStallProtection(0.0, 0.0, 0.0, 0.0);
        tracer.traceInfo(
            moduleName, "Zero Calibrate completed (lowerLimitSw=" + elevator.isLowerLimitSwitchActive() + ").");
    }   //zeroCalCompleted

    /**
     * This method is a callback to perform the specified action when it is safe to do so.
     *
     * @param context specifies the action parameters.
     */
    private void performAction(Object context)
    {
        ActionParams actionParams = (ActionParams) context;
        double delay, timeout;

        switch (actionParams.actionType)
        {
            case ElevatorSetPosition:
                tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                // Move the elevator only if the arm is already at or has reached safe position.
                // actionParams.actionEvent is null if arm is already at safe position.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
                    timeout =
                        actionParams.expiredTime == 0.0 ? 0.0 : actionParams.expiredTime - TrcTimer.getCurrentTime();
                    // Move elevator to desired position.
                    elevator.setPosition(
                        null, delay, actionParams.pos, true, actionParams.powerLimit, actionParams.completionEvent,
                        timeout);
                }
                break;

            case ElevatorSetPower:
                // Trace only if power has changed.
                if (actionParams.power != elevatorPrevPrintedPower)
                {
                    tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                    elevatorPrevPrintedPower = actionParams.power;
                }
                // Move the elevator only if the arm is already at or has reached safe position.
                // actionParams.actionEvent is null if arm is already at safe position.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
                    // Move elevator with desired power.
                    elevator.setPower(
                        null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
                }
                break;

            case ElevatorSetPidPower:
                // Trace only if power has changed.
                if (actionParams.power != elevatorPrevPrintedPower)
                {
                    tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                    elevatorPrevPrintedPower = actionParams.power;
                }
                // Move the elevator only if the arm is already at or has reached safe position.
                // actionParams.actionEvent is null if arm is already at safe position.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Move elevator with desired power with PID control.
                    elevator.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;

            case ArmSetPosition:
                tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                // Move the arm only if the elevator is already at or has reached safe height.
                // actionParams.actionEvent is null if elevator is already at safe height.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If elevator has moved, it already applied delay.
                    delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
                    timeout =
                        actionParams.expiredTime == 0.0 ? 0.0 : actionParams.expiredTime - TrcTimer.getCurrentTime();
                    // Make sure wrist is at appropriate position.
                    if (actionParams.pos == RobotParams.ARM_SCORE_BACKDROP_POS)
                    {
                        wrist.setPosition(RobotParams.WRIST_UP_POS);
                    }
                    else if (actionParams.pos == RobotParams.ARM_LOAD_POS)
                    {
                        wrist.setPosition(RobotParams.WRIST_DOWN_POS);
                    }
                    // Move arm to desired position.
                    arm.setPosition(
                        null, delay, actionParams.pos, true, actionParams.powerLimit, actionParams.completionEvent,
                        timeout);
                }
                break;

            case ArmSetPower:
                // Trace only if power has changed.
                if (actionParams.power != armPrevPrintedPower)
                {
                    tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                    armPrevPrintedPower = actionParams.power;
                }
                // Move the arm only if the elevator is already at or has reached safe height.
                // actionParams.actionEvent is null if elevator is already at safe height.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If elevator has moved, it already applied delay.
                    delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
                    // Move arm with desired power.
                    arm.setPower(null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
                }
                break;

            case ArmSetPidPower:
                // Trace only if power has changed.
                if (actionParams.power != armPrevPrintedPower)
                {
                    tracer.traceInfo(moduleName, "actionParams=" + actionParams);
                    armPrevPrintedPower = actionParams.power;
                }
                // Move the arm only if the elevator is already at or has reached safe height.
                // actionParams.actionEvent is null if elevator is already at safe height.
                if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
                {
                    // Move arm with desired power with PID control.
                    arm.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;
        }
        removePendingEventCallback(actionParams.actionEvent);
    }   //performAction

    /**
     * This method sets the elevator, arm, wrist to the loading position and makes sure it doesn't hit the intake on
     * its way.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setLoadingPosition(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        tracer.traceInfo(
            moduleName, "owner=%s, delay=%.3f, event=%s, timeout=%.3f (elevatorPos=%.2f, armPos=%.2f)",
            owner, delay, completionEvent, timeout, elevator.getPosition(), arm.getPosition());
        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // Move the wrist in loading position before lowering the elevator.
            wrist.setPosition(RobotParams.WRIST_DOWN_POS);
            // Before moving the arm, make sure the arm is safe to move, or we have to bring the elevator up to safe
            // height first before moving the arm back in then we will lower the elevator.
            if (!armIsSafeToMove(RobotParams.ARM_LOAD_POS))
            {
                // Move elevator to safe position first. Elevator is fast and arm is slow, so we don't have to wait for
                // the elevator to complete its movement.
                elevator.setPosition(
                    null, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, null, 0.0);
            }
            // Move the arm to loading position before lowering the elevator.
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = new TrcEvent("setLoadingPosition.actionEvent");
            // Setting up the elevator operation after the arm is in loading position.
            actionParams.setPositionParams(
                ActionType.ElevatorSetPosition, actionEvent, 0.0, RobotParams.ELEVATOR_LOAD_POS,
                RobotParams.ELEVATOR_POWER_LIMIT, completionEvent, expiredTime);
            setActionCallback(actionParams, actionEvent);
            armSetPosition(
                owner, false, delay, RobotParams.ARM_LOAD_POS, RobotParams.ARM_POWER_LIMIT, actionEvent, 3.0);
        }
    }   //setLoadingPosition

    /**
     * This method sets the elevator, arm, wrist to the scoring position and makes sure it doesn't hit the intake on
     * its way.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param elevatorPos specifies the elevator scoring level.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setScoringPosition(
        String owner, double delay, double elevatorPos, TrcEvent completionEvent, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        tracer.traceInfo(
            moduleName,
            "owner=%s, delay=%.3f, elevatorPos=%.1f, event=%s, timeout=%.3f (elevatorPos=%.2f, armPos=%.2f)",
            owner, delay, elevatorPos, completionEvent, timeout, elevator.getPosition(), arm.getPosition());
        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = new TrcEvent("setScoringPosition.actionEvent");
            // Setting up the arm operation after the elevator is in scoring height.
            actionParams.setPositionParams(
                ActionType.ArmSetPosition, actionEvent, 0.0, RobotParams.ARM_SCORE_BACKDROP_POS,
                RobotParams.ARM_POWER_LIMIT, completionEvent, expiredTime);
            // Wrist position will be set in the arm operation.
            // Move the elevator to scoring height before moving the arm to scoring position.
            setActionCallback(actionParams, actionEvent);
            elevatorSetPosition(owner, false, delay, elevatorPos, RobotParams.ELEVATOR_POWER_LIMIT, actionEvent, 1.0);
        }
    }   //setScoringPosition

    /**
     * This method sets the elevator and arm to the hanging position and makes sure it doesn't hit the intake on its
     * way.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setHangingPosition(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        tracer.traceInfo(
            moduleName, "owner=%s, delay=%.3f, event=%s, timeout=%.3f", owner, delay, completionEvent, timeout);
        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = new TrcEvent("setHangingPosition.actionEvent");
            // Setting up the arm operation after the elevator is in max height.
            actionParams.setPositionParams(
                ActionType.ArmSetPosition, actionEvent, 0.0, RobotParams.ARM_HANG_POS, 0.1,
                completionEvent, expiredTime);
            // Move the elevator to max height before moving the arm to hanging position.
            setActionCallback(actionParams, actionEvent);
            elevatorSetPosition(
                owner, false, delay, RobotParams.ELEVATOR_MAX_POS - 0.5, RobotParams.ELEVATOR_POWER_LIMIT, actionEvent,
                1.0);
        }
    }   //setHangingPosition

    //
    // Elevator subsystem methods.
    //

    /**
     * This method checks if the elevator is safe to move to the target position so it doesn't hit the intake.
     *
     * @param elevatorTargetPos specifies the elevator target position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean elevatorIsSafeToMove(double elevatorTargetPos)
    {
        double currArmPos = arm.getPosition();

        return currArmPos - RobotParams.ARM_TOLERANCE <= RobotParams.ARM_LOAD_POS ||
               // ^^^ Arm is already in load position.
               currArmPos + RobotParams.ARM_TOLERANCE >= RobotParams.ARM_FREE_TO_MOVE_POS ||
               // ^^^ Arm is already beyond FREE_TO_MOVE position.
               elevatorTargetPos >= RobotParams.ELEVATOR_SAFE_POS;
               // ^^^ We are moving up beyond ELEVATOR_SAFE_POS.
    }   //elevatorIsSafeToMove

    /**
     * This method sets the elevator position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param cancelPrev specifies true to cancel previous operation, false otherwise.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    private void elevatorSetPosition(
        String owner, boolean cancelPrev, double delay, double pos, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        tracer.traceInfo(
            moduleName,
            "owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f " +
            "(elevatorPos=%.2f, armPos=%.2f)",
            owner, delay, pos, powerLimit, completionEvent, timeout, elevator.getPosition(), arm.getPosition());
        if (cancelPrev)
        {
            cancel(owner);
        }
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            boolean safeToMove = elevatorIsSafeToMove(pos);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPosition.actionEvent");
            // Setting up the elevator operation after the arm is at safe position.
            actionParams.setPositionParams(
                ActionType.ElevatorSetPosition, actionEvent, delay, pos, powerLimit, completionEvent, expiredTime);
            if (safeToMove)
            {
                // Arm is already at safe position.
                performAction(actionParams);
            }
            else
            {
                // Move the arm to safe position and set up a callback to finish the elevator operation.
                setActionCallback(actionParams, actionEvent);
                arm.setPosition(
                    null, delay, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, actionEvent, timeout);
            }
        }
    }   //elevatorSetPosition

    /**
     * This method sets the elevator position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    public void elevatorSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
    {
        elevatorSetPosition(owner, true, delay, pos, powerLimit, completionEvent, timeout);
    }   //elevatorSetPosition

    /**
     * This method sets the elevator power.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param power specifies the percentage power (range -1.0 to 1.0).
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void elevatorSetPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        // Trace only if power has changed.
        if (power != elevatorPrevPrintedPower)
        {
            tracer.traceInfo(
                moduleName, "owner=%s, delay=%.1f, power=%.3f, duration=%.3f, pos=%.3f, event=%s, safe=%s",
                owner, delay, power, duration, elevator.getPosition(), event,
                elevatorIsSafeToMove(power > 0.0 ? RobotParams.ELEVATOR_MAX_POS : RobotParams.ELEVATOR_MIN_POS));
            elevatorPrevPrintedPower = power;
        }

        if (validateOwnership(owner))
        {
            boolean safeToMove =
                power == 0.0 ||
                elevatorIsSafeToMove(power > 0.0 ? RobotParams.ELEVATOR_MAX_POS : RobotParams.ELEVATOR_MIN_POS);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPower.actionEvent");
            // Setting up the elevator operation after the arm is at safe position.
            actionParams.setPowerParams(ActionType.ElevatorSetPower, actionEvent, delay, power, duration, event);
            if (safeToMove)
            {
                // Arm is already at safe position.
                performAction(actionParams);
            }
            else
            {
                // Move the arm to safe position and set up a callback to finish the elevator operation.
                setActionCallback(actionParams, actionEvent);
                arm.setPosition(
                    null, delay, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, actionEvent, 0.0);
            }
        }
    }   //elevatorSetPower

    /**
     * This method sets the elevator power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     */
    public void elevatorSetPidPower(String owner, double power, double minPos, double maxPos)
    {
        // Trace only if power has changed.
        if (power != elevatorPrevPrintedPower)
        {
            tracer.traceInfo(
                moduleName, "owner=%s, power=%.3f, pos=%.3f, minPos=%.1f, maxPos=%.1f, safe=%s",
                owner, power, elevator.getPosition(), minPos, maxPos,
                elevatorIsSafeToMove(power > 0.0 ? maxPos : minPos));
            elevatorPrevPrintedPower = power;
        }

        if (validateOwnership(owner))
        {
            boolean safeToMove = power == 0.0 || elevatorIsSafeToMove(power > 0.0 ? maxPos : minPos);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPidPower.actionEvent");
            // Setting up the elevator operation after the arm is at safe position.
            actionParams.setPidPowerParams(ActionType.ElevatorSetPidPower, actionEvent, power, minPos, maxPos);
            if (safeToMove)
            {
                // Arm is already at safe position.
                performAction(actionParams);
            }
            else
            {
                // Move the arm to safe position and set up a callback to finish the elevator operation.
                setActionCallback(actionParams, actionEvent);
                arm.setPosition(
                    null, 0.0, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, actionEvent, 0.0);
            }
        }
    }   //elevatorSetPidPower

    /**
     * This method sets the elevator to the specified preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param powerLimit specifies the power limit applied to the elevator.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
     *        should be set to zero.
     */
    public void elevatorSetPresetPosition(
        String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
    {
        if (elevator.validatePresetIndex(presetIndex))
        {
            elevatorSetPosition(owner, delay, elevator.getPresetValue(presetIndex), powerLimit, event, timeout);
        }
    }   //elevatorSetPresetPosition

    /**
     * This method sets the elevator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is completed, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void elevatorPresetPositionUp(String owner, double powerLimit)
    {
        int index = elevator.nextPresetIndexUp();

        if (index != -1)
        {
            elevatorSetPosition(owner, 0.0, elevator.getPresetValue(index), powerLimit, null, 1.0);
        }
    }   //elevatorPresetPositionUp

    /**
     * This method sets the elevator to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is completed, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void elevatorPresetPositionDown(String owner, double powerLimit)
    {
        int index = elevator.nextPresetIndexDown();

        if (index != -1)
        {
            elevatorSetPosition(owner, 0.0, elevator.getPresetValue(index), powerLimit, null, 1.0);
        }
    }   //elevatorPresetPositionDown

    //
    // Arm subsystem methods.
    //

    /**
     * This method is called to compute the power compensation to counteract gravity on the Arm.
     *
     * @param currPower specifies the current motor power (not used).
     * @return gravity compensation for the arm.
     */
    private double armGetPowerComp(double currPower)
    {
        return RobotParams.ARM_MAX_GRAVITY_COMP_POWER * Math.sin(Math.toRadians(arm.getPosition()));
    }   //armGetPowerComp

    /**
     * This method checks if the arm is safe to move so it doesn't hit the intake.
     *
     * @param armTargetPos specifies the arm target position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean armIsSafeToMove(double armTargetPos)
    {
        double armPos = arm.getPosition();

        return elevator.getPosition() + RobotParams.ELEVATOR_TOLERANCE >= RobotParams.ELEVATOR_SAFE_POS ||
               // ^^^ Elevator above safe height.
               armPos <= RobotParams.ARM_LOAD_POS && armTargetPos <= RobotParams.ARM_LOAD_POS ||
               // ^^^ Arm already in LOAD_POS.
               armPos + RobotParams.ARM_TOLERANCE >= RobotParams.ARM_FREE_TO_MOVE_POS &&
               armTargetPos >= RobotParams.ARM_FREE_TO_MOVE_POS;
               // ^^^ Elevator is down but arm is already out and we are lifting arm.
    }   //armIsSafeToMove

    /**
     * This method sets the arm position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param cancelPrev specifies true to cancel previous operation, false otherwise.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    private void armSetPosition(
        String owner, boolean cancelPrev, double delay, double pos, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        tracer.traceInfo(
            moduleName,
            "owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f " +
            "(elevatorPos=%.2f, armPos=%.2f)",
            owner, delay, pos, powerLimit, completionEvent, timeout, elevator.getPosition(), arm.getPosition());
        if (cancelPrev)
        {
            cancel(owner);
        }
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            boolean safeToMove = armIsSafeToMove(pos);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPosition.actionEvent");
            // Setting up the arm operation after the elevator is at safe height.
            actionParams.setPositionParams(
                ActionType.ArmSetPosition, actionEvent, delay, pos, powerLimit, completionEvent, expiredTime);
            if (safeToMove)
            {
                // Elevator is already at safe height.
                performAction(actionParams);
            }
            else
            {
                // Move the elevator to safe height and set up a callback to finish the arm operation.
                setActionCallback(actionParams, actionEvent);
                elevator.setPosition(
                    null, delay, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, actionEvent,
                    timeout);
            }
        }
    }   //armSetPosition

    /**
     * This method sets the arm position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    public void armSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
    {
        armSetPosition(owner, true, delay, pos, powerLimit, completionEvent, timeout);
    }   //armSetPosition

    /**
     * This method sets the arm power.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param power specifies the percentage power (range -1.0 to 1.0).
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void armSetPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        // Trace only if power has changed.
        if (power != armPrevPrintedPower)
        {
            tracer.traceInfo(
                moduleName, "owner=%s, delay=%.1f, power=%.3f, duration=%.3f, pos=%.3f, event=%s, safe=%s",
                owner, delay, power, duration, arm.getPosition(), event,
                armIsSafeToMove(power > 0.0 ? RobotParams.ARM_MAX_POS : RobotParams.ARM_MIN_POS));
            armPrevPrintedPower = power;
        }

        if (validateOwnership(owner))
        {
            boolean safeToMove =
                power == 0.0 || armIsSafeToMove(power > 0.0 ? RobotParams.ARM_MAX_POS : RobotParams.ARM_MIN_POS);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPower.actionEvent");
            // Setting up the arm operation after the elevator is at safe height.
            actionParams.setPowerParams(ActionType.ArmSetPower, actionEvent, delay, power, duration, event);
            if (safeToMove)
            {
                // Elevator is already at safe height.
                performAction(actionParams);
            }
            else
            {
                // Move the elevator to safe height and set up a callback to finish the arm operation.
                setActionCallback(actionParams, actionEvent);
                elevator.setPosition(
                    null, delay, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, actionEvent,
                    0.0);
            }
        }
    }   //armSetPower

    /**
     * This method sets the arm power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     */
    public void armSetPidPower(String owner, double power, double minPos, double maxPos)
    {
        // Trace only if power has changed.
        if (power != armPrevPrintedPower)
        {
            tracer.traceInfo(
                moduleName, "owner=%s, power=%.3f, pos=%.3f, minPos=%.1f, maxPos=%.1f, safe=%s",
                owner, power, arm.getPosition(), minPos, maxPos, armIsSafeToMove(power > 0.0 ? maxPos : minPos));
            armPrevPrintedPower = power;
        }

        if (validateOwnership(owner))
        {
            boolean safeToMove =
                power == 0.0 || armIsSafeToMove(power > 0.0 ? maxPos : minPos);
            ActionParams actionParams = new ActionParams();
            TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPidPower.actionEvent");
            // Setting up the arm operation after the elevator is at safe height.
            actionParams.setPidPowerParams(ActionType.ArmSetPidPower, actionEvent, power, minPos, maxPos);
            if (safeToMove)
            {
                // Elevator is already at safe height.
                performAction(actionParams);
            }
            else
            {
                // Move the elevator to safe height and set up a callback to finish the arm operation.
                setActionCallback(actionParams, actionEvent);
                elevator.setPosition(
                    null, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, actionEvent, 0.0);
            }
        }
    }   //armSetPidPower

    /**
     * This method sets the arm to the specified preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param powerLimit specifies the power limit applied to the elevator.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
     *        should be set to zero.
     */
    public void armSetPresetPosition(
        String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
    {
        if (arm.validatePresetIndex(presetIndex))
        {
            armSetPosition(owner, delay, arm.getPresetValue(presetIndex), powerLimit, event, timeout);
        }
    }   //armSetPresetPosition

    /**
     * This method sets the arm to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is completed, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void armPresetPositionUp(String owner, double powerLimit)
    {
        int index = arm.nextPresetIndexUp();

        if (index != -1)
        {
            armSetPosition(owner, 0.0, arm.getPresetValue(index), powerLimit, null, 5.0);
        }
    }   //armPresetPositionUp

    /**
     * This method sets the arm to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is completed, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void armPresetPositionDown(String owner, double powerLimit)
    {
        int index = arm.nextPresetIndexDown();

        if (index != -1)
        {
            armSetPosition(owner, 0.0, arm.getPresetValue(index), powerLimit, null, 5.0);
        }
    }   //armPresetPositionDown

    /**
     * This method sets the wrist position while observing whether it's safe to do so.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void wristSetPosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {
        if (elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_POS ||
            arm.getPosition() >= RobotParams.ARM_FREE_TO_MOVE_POS)
        {
            wrist.setPosition(owner, delay, position, completionEvent, timeout);
        }
    }   //wristSetPosition

    /**
     * This method sets the wrist position while observing whether it's safe to do so.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void wristSetPosition(double delay, double position, TrcEvent completionEvent, double timeout)
    {
        wristSetPosition(null, delay, position, completionEvent, timeout);
    }   //wristSetPosition

    /**
     * This method sets the wrist position while observing whether it's safe to do so.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void wristSetPosition(double delay, double position)
    {
        wristSetPosition(null, delay, position, null, 0.0);
    }   //wristSetPosition

    /**
     * This method sets the wrist position while observing whether it's safe to do so.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void wristSetPosition(double position)
    {
        wristSetPosition(null, 0.0, position, null, 0.0);
    }   //wristSetPosition

    /**
     * This method is called the TrcTriggerThresholdZones to get the sensor data.
     *
     * @return distance to detected object in inches.
     */
    public double wristGetDistance()
    {
        TrcSensor.SensorData<Double> data =
            wristSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH);
        return data != null && data.value != null ? data.value : 0.0;
    }   //wristGetDistance

}   //class ElevatorArm
