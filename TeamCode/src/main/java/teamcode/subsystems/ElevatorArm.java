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

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcTriggerThresholdZones;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;
import TrcFtcLib.ftclib.FtcServoActuator;
import teamcode.RobotParams;

public class ElevatorArm implements TrcExclusiveSubsystem
{
    private static final String moduleName = "ElevatorArm";

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
        boolean safeToMove = false;
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
            ActionType actionType, double delay, double pos, double powerLimit, TrcEvent event, double expiredTime)
        {
            this.actionType = actionType;
            this.safeToMove = false;
            this.delay = delay;
            this.pos = pos;
            this.powerLimit = powerLimit;
            this.completionEvent = event;
            this.expiredTime = expiredTime;
        }   //setPositionParams

        void setPowerParams(ActionType actionType, double delay, double power, double duration, TrcEvent event)
        {
            this.actionType = actionType;
            this.safeToMove = false;
            this.delay = delay;
            this.power = power;
            this.duration = duration;
            this.completionEvent = event;
        }   //setPowerParams

        void setPidPowerParams(ActionType actionType, double power, double minPos, double maxPos)
        {
            this.actionType = actionType;
            this.safeToMove = false;
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
                "\tsafeToMove=%s\n" +
                "\tdelay=%.3f\n" +
                "\tpos=%.1f\n" +
                "\tpowerLimit=%.1f\n" +
                "\tpower=%.1f\n" +
                "\tduration=%.1f\n" +
                "\tposRange=%.1f/%.1f\n" +
                "\tevent=%s\n" +
                "\texpiredTime=%.3f\n)",
                actionType, safeToMove, delay, pos, powerLimit, power, duration, minPos, maxPos, completionEvent,
                expiredTime);
        }   //toString

    }   //class ActionParams

    private final ActionParams elevatorActionParams = new ActionParams();
    private final ActionParams armActionParams = new ActionParams();
    private final TrcDbgTrace msgTracer;
    // Elevator subsystem.
    public final TrcMotor elevator;
    private final TrcEvent elevatorActionEvent;
    // Arm subsystem.
    public final TrcMotor arm;
    private final TrcEvent armActionEvent;
    // Wrist subsystem.
    public final FtcServo wrist;
    public final FtcDistanceSensor wristSensor;
    public final TrcTriggerThresholdZones wristTrigger;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     * @param tracePidInfo specifies true to enable tracing PID info, false to disable.
     */
    public ElevatorArm(TrcDbgTrace msgTracer, boolean tracePidInfo)
    {
        this.msgTracer = msgTracer;
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
                new FtcMotorActuator(
                    RobotParams.HWNAME_ELEVATOR, elevatorParams, msgTracer, tracePidInfo).getActuator();
            elevator.setSoftwarePidEnabled(true);
            elevator.setPositionPidCoefficients(
                RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD, RobotParams.ELEVATOR_KF,
                RobotParams.ELEVATOR_IZONE);
            elevator.setPositionPidTolerance(RobotParams.ELEVATOR_TOLERANCE);
            elevator.setStallDetectionEnabled(
                RobotParams.ARM_STALL_DETECTION_DELAY, RobotParams.ARM_STALL_DETECTION_TIMEOUT,
                RobotParams.ARM_STALL_ERR_RATE_THRESHOLD);
            elevator.getPositionPidController().setTraceEnabled(true, false, false);
            elevatorActionEvent = new TrcEvent(RobotParams.HWNAME_ELEVATOR + ".actionEvent");
        }
        else
        {
            elevator = null;
            elevatorActionEvent = null;
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
            arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, true, armParams, msgTracer, tracePidInfo).getActuator();
            arm.setPositionPidCoefficients(
                RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_KF, RobotParams.ARM_IZONE);
            arm.setPositionPidTolerance(RobotParams.ARM_TOLERANCE);
            arm.setPositionPidPowerComp(this::armGetPowerComp);
            arm.setStallDetectionEnabled(true);
            arm.getPositionPidController().setTraceEnabled(true, false, false);
            armActionEvent = new TrcEvent(RobotParams.HWNAME_ARM + ".actionEvent");
        }
        else
        {
            arm = null;
            armActionEvent = null;
        }
        // Wrist subsystem.
        if (RobotParams.Preferences.useWrist)
        {
            FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
                .setServoInverted(RobotParams.WRIST_SERVO_INVERTED)
                .setHasFollowerServo(RobotParams.WRIST_HAS_FOLLOWER_SERVO, RobotParams.WRIST_FOLLOWER_SERVO_INVERTED);

            wrist = new FtcServoActuator(RobotParams.HWNAME_WRIST, wristParams, msgTracer).getActuator();
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
        final String funcName = "cancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcTimer.getModeElapsedTime(), owner);
        }

        if (hasOwnership(owner))
        {
            elevator.stop();
            arm.stop();
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
     * This method zero calibrates the elevator. Arm has absolute encoder and doesn't need zero calibration.
     * Note: This assumes the arm is at safe position or elevator zero calibration won't happen.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        final String funcName = "zeroCalibrate";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcTimer.getModeElapsedTime(), owner);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, null, msgTracer);
        if (validateOwnership(owner))
        {
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
                elevator.zeroCalibrate(RobotParams.ELEVATOR_CAL_POWER, releaseOwnershipEvent);
            }
        }
    }   //zeroCalibrate

    /**
     * This method is a callback to perform the specified action when it is safe to do so.
     *
     * @param context specifies the action parameters.
     */
    private void performAction(Object context)
    {
        final String funcName = "performAction";
        ActionParams actionParams = (ActionParams) context;

        switch (actionParams.actionType)
        {
            case ElevatorSetPosition:
                if (msgTracer != null)
                {
                    msgTracer.traceInfo(
                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
                }
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || elevatorActionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();
                    // Move elevator to desired position.
                    elevator.setPosition(
                        null, delay, actionParams.pos, true, actionParams.powerLimit, actionParams.completionEvent,
                        timeout);
                }
                break;

            case ElevatorSetPower:
//                if (msgTracer != null && actionParams.power != 0.0)
//                {
//                    msgTracer.traceInfo(
//                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
//                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
//                }
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || elevatorActionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    // Move elevator with desired power.
                    elevator.setPower(
                        null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
                }
                break;

            case ElevatorSetPidPower:
//                if (msgTracer != null && actionParams.power != 0.0)
//                {
//                    msgTracer.traceInfo(
//                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
//                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
//                }
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || elevatorActionEvent.isSignaled())
                {
                    // Move elevator with desired power with PID control.
                    elevator.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;

            case ArmSetPosition:
                if (msgTracer != null)
                {
                    msgTracer.traceInfo(
                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
                }
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || armActionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If elevator has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();
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
//                if (msgTracer != null && actionParams.power != 0.0)
//                {
//                    msgTracer.traceInfo(
//                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
//                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
//                }
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || armActionEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If elevator has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    // Move arm with desired power.
                    arm.setPower(null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
                }
                break;

            case ArmSetPidPower:
//                if (msgTracer != null && actionParams.power != 0.0)
//                {
//                    msgTracer.traceInfo(
//                        funcName, "[%.3f] elevatorEvent=%s, armEvent=%s, actionParams=%s",
//                        TrcTimer.getModeElapsedTime(), elevatorActionEvent, armActionEvent, actionParams);
//                }
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || armActionEvent.isSignaled())
                {
                    // Move arm with desired power with PID control.
                    arm.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;
        }
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
        final String funcName = "setLoadingPosition";
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.3f, event=%s, timeout=%.3f",
                TrcTimer.getModeElapsedTime(), owner, delay, completionEvent, timeout);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, msgTracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // Move the wrist in loading position before lowering the elevator.
            wrist.setPosition(RobotParams.WRIST_DOWN_POS);
            // Setting up the elevator operation after the arm is in loading position.
            elevatorActionParams.setPositionParams(
                ActionType.ElevatorSetPosition, 0.0, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ELEVATOR_POWER_LIMIT,
                completionEvent, expiredTime);
            // Before moving the arm, make sure the arm is safe to move, or we have to bring the elevator up to safe
            // height first before moving the arm back in then we will lower the elevator.
            // Even if the elevator is already at safe height, we may want to set it to safe height just to hold its
            // position so it doesn't drop.
            if (!armIsSafeToMove(RobotParams.ARM_LOAD_POS) ||
                elevator.getPosition() + RobotParams.ELEVATOR_TOLERANCE >= RobotParams.ELEVATOR_SAFE_POS)
            {
                // Move elevator to safe position first. Elevator is fast and arm is slow, so we don't have to wait for
                // the elevator to complete its movement.
                elevator.setPosition(
                    null, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, null, 0.0);
            }
            // Move the arm to loading position before lowering the elevator.
            elevatorActionEvent.setCallback(this::performAction, elevatorActionParams);
            armSetPosition(
                owner, delay, RobotParams.ARM_LOAD_POS, RobotParams.ARM_POWER_LIMIT, elevatorActionEvent, 3.0);
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
        final String funcName = "setScoringPosition";
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.3f, elevatorPos=%.1f, event=%s, timeout=%.3f",
                TrcTimer.getModeElapsedTime(), owner, delay, elevatorPos, completionEvent, timeout);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, msgTracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // Setting up the arm operation after the elevator is in scoring height.
            // Wrist position will be set in the arm operation.
            armActionParams.setPositionParams(
                ActionType.ArmSetPosition, 0.0, RobotParams.ARM_SCORE_BACKDROP_POS, RobotParams.ARM_POWER_LIMIT,
                completionEvent, expiredTime);
            // Move the elevator to scoring height before moving the arm to scoring position.
            armActionEvent.setCallback(this::performAction, armActionParams);
            elevatorSetPosition(owner, delay, elevatorPos, RobotParams.ELEVATOR_POWER_LIMIT, armActionEvent, 1.0);
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
        final String funcName = "setHangingPosition";
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.3f, event=%s, timeout=%.3f",
                TrcTimer.getModeElapsedTime(), owner, delay, completionEvent, timeout);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, msgTracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // Setting up the arm operation after the elevator is in max height.
            armActionParams.setPositionParams(
                ActionType.ArmSetPosition, 0.0, RobotParams.ARM_HANG_POS, RobotParams.ARM_POWER_LIMIT, completionEvent,
                expiredTime);
            // Move the elevator to max height before moving the arm to hanging position.
            armActionEvent.setCallback(this::performAction, armActionParams);
            elevatorSetPosition(
                owner, delay, RobotParams.ELEVATOR_MAX_POS - 0.5, RobotParams.ELEVATOR_POWER_LIMIT, armActionEvent, 1.0);
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
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    public void elevatorSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
    {
        final String funcName = "elevatorSetPosition";
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f",
                TrcTimer.getModeElapsedTime(), owner, delay, pos, powerLimit, completionEvent, timeout);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, msgTracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            elevatorActionParams.setPositionParams(
                ActionType.ElevatorSetPosition, delay, pos, powerLimit, completionEvent, expiredTime);
            if (elevatorIsSafeToMove(pos))
            {
                elevatorActionParams.safeToMove = true;
                performAction(elevatorActionParams);
            }
            else
            {
                elevatorActionEvent.clear();
                elevatorActionEvent.setCallback(this::performAction, elevatorActionParams);
                arm.setPosition(
                    null, delay, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, elevatorActionEvent,
                    timeout);
            }
        }
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
        if (validateOwnership(owner))
        {
//            if (msgTracer != null && power != 0.0)
//            {
//                msgTracer.traceInfo(
//                    "elevatorSetPower",
//                    "[%.3f] owner=%s, delay=%.1f, power=%.1f, duration=%.1f, event=%s, safe=%s",
//                    TrcTimer.getModeElapsedTime(), owner, delay, power, duration, event,
//                    elevatorIsSafeToMove(power > 0.0? RobotParams.ELEVATOR_MAX_POS: RobotParams.ELEVATOR_MIN_POS));
//            }
            elevatorActionParams.setPowerParams(ActionType.ElevatorSetPower, delay, power, duration, event);
            if (power == 0.0 ||
                elevatorIsSafeToMove(power > 0.0 ? RobotParams.ELEVATOR_MAX_POS : RobotParams.ELEVATOR_MIN_POS))
            {
                elevatorActionParams.safeToMove = true;
                performAction(elevatorActionParams);
            }
            else
            {
                elevatorActionEvent.clear();
                elevatorActionEvent.setCallback(this::performAction, elevatorActionParams);
                arm.setPosition(
                    null, delay, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, elevatorActionEvent, 0.0);
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
        if (validateOwnership(owner))
        {
//            if (msgTracer != null && power != 0.0)
//            {
//                msgTracer.traceInfo(
//                    "elevatorSetPidPower", "[%.3f] owner=%s, power=%.1f, minPos=%.1f, maxPos=%.1f, safe=%s",
//                    TrcTimer.getModeElapsedTime(), owner, power, minPos, maxPos, elevatorIsSafeToMove(power > 0.0?
//                                                                                                         maxPos: minPos));
//            }
            elevatorActionParams.setPidPowerParams(ActionType.ElevatorSetPidPower, power, minPos, maxPos);
            if (power == 0.0 || elevatorIsSafeToMove(power > 0.0 ? maxPos : minPos))
            {
                elevatorActionParams.safeToMove = true;
                performAction(elevatorActionParams);
            }
            else
            {
                elevatorActionEvent.clear();
                elevatorActionEvent.setCallback(this::performAction, elevatorActionParams);
                arm.setPosition(
                    null, 0.0, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, elevatorActionEvent, 0.0);
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
            elevatorSetPosition(owner, delay, elevator.getPresetPosition(presetIndex), powerLimit, event, timeout);
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
            elevatorSetPosition(owner, 0.0, elevator.getPresetPosition(index), powerLimit, null, 1.0);
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
            elevatorSetPosition(owner, 0.0, elevator.getPresetPosition(index), powerLimit, null, 1.0);
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
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
     * @param timeout specifies timeout in seconds.
     */
    public void armSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
    {
        final String funcName = "armSetPosition";
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f",
                TrcTimer.getModeElapsedTime(), owner, delay, pos, powerLimit, completionEvent, timeout);
        }

        cancel(owner);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, msgTracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            armActionParams.setPositionParams(
                ActionType.ArmSetPosition, delay, pos, powerLimit, completionEvent, expiredTime);
            if (armIsSafeToMove(pos))
            {
                armActionParams.safeToMove = true;
                performAction(armActionParams);
            }
            else
            {
                armActionEvent.clear();
                armActionEvent.setCallback(this::performAction, armActionParams);
                elevator.setPosition(
                    null, delay, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, armActionEvent,
                    timeout);
            }
        }
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
        if (validateOwnership(owner))
        {
//            if (msgTracer != null && power != 0.0)
//            {
//                msgTracer.traceInfo(
//                    "armSetPower", "[%.3f] owner=%s, delay=%.1f, power=%.1f, duration=%.1f, event=%s, safe=%s",
//                    TrcTimer.getModeElapsedTime(), owner, delay, power, duration, event,
//                    armIsSafeToMove(power > 0.0? RobotParams.ARM_MAX_POS: RobotParams.ARM_MIN_POS));
//            }
            armActionParams.setPowerParams(ActionType.ArmSetPower, delay, power, duration, event);
            if (power == 0.0 ||
                armIsSafeToMove(power > 0.0 ? RobotParams.ARM_MAX_POS : RobotParams.ARM_MIN_POS))
            {
                armActionParams.safeToMove = true;
                performAction(armActionParams);
            }
            else
            {
                armActionEvent.clear();
                armActionEvent.setCallback(this::performAction, armActionParams);
                elevator.setPosition(
                    null, delay, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, armActionEvent,
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
        if (validateOwnership(owner))
        {
//            if (msgTracer != null && power != 0.0)
//            {
//                msgTracer.traceInfo(
//                    "armSetPidPower", "[%.3f] owner=%s, power=%.1f, minPos=%.1f, maxPos=%.1f, safe=%s",
//                    TrcTimer.getModeElapsedTime(), owner, power, minPos, maxPos,
//                    armIsSafeToMove(power > 0.0? maxPos: minPos));
//            }
            armActionParams.setPidPowerParams(ActionType.ArmSetPidPower, power, minPos, maxPos);
            if (power == 0.0 || armIsSafeToMove(power > 0.0 ? maxPos : minPos))
            {
                armActionParams.safeToMove = true;
                performAction(armActionParams);
            }
            else
            {
                msgTracer.traceInfo("armSetPidPower", "Not safe to move arm yet, move elevator to safe height.");
                armActionEvent.clear();
                armActionEvent.setCallback(this::performAction, armActionParams);
                elevator.setPosition(
                    null, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, armActionEvent,
                    0.0);
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
            armSetPosition(owner, delay, arm.getPresetPosition(presetIndex), powerLimit, event, timeout);
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
            armSetPosition(owner, 0.0, arm.getPresetPosition(index), powerLimit, null, 3.0);
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
            armSetPosition(owner, 0.0, arm.getPresetPosition(index), powerLimit, null, 3.0);
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
        return data != null && data.value != null? data.value: 0.0;
    }   //wristGetDistance

}   //class ElevatorArm
