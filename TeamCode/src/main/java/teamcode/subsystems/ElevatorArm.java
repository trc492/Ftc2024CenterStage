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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;
import TrcFtcLib.ftclib.FtcServoActuator;
import teamcode.RobotParams;

public class ElevatorArm
{
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
        String owner = null;
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
            ActionType actionType, String owner, double delay, double pos, double powerLimit, TrcEvent event,
            double expiredTime)
        {
            this.actionType = actionType;
            this.safeToMove = false;
            this.owner = owner;
            this.delay = delay;
            this.pos = pos;
            this.powerLimit = powerLimit;
            this.completionEvent = event;
            this.expiredTime = expiredTime;
        }   //setPositionParams

        void setPowerParams(
            ActionType actionType, String owner, double delay, double power, double duration, TrcEvent event)
        {
            this.actionType = actionType;
            this.safeToMove = false;
            this.owner = owner;
            this.delay = delay;
            this.power = power;
            this.duration = duration;
            this.completionEvent = event;
        }   //setPowerParams

        void setPidPowerParams(
            ActionType actionType, String owner, double power, double minPos, double maxPos)
        {
            this.actionType = actionType;
            this.safeToMove = false;
            this.owner = owner;
            this.power = power;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }   //setPidPowerParams

    }   //class ActionParams

    private final ActionParams elevatorActionParams = new ActionParams();
    private final ActionParams armActionParams = new ActionParams();
    // Elevator subsystem.
    public final TrcMotor elevator;
    public final TrcEvent elevatorEvent;
    // Arm subsystem.
    public final TrcMotor arm;
    public final TrcEvent armEvent;
    // Wrist subsystem.
    public final FtcServo wrist;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     * @param tracePidInfo specifies true to enable tracing PID info, false to disable.
     */
    public ElevatorArm(TrcDbgTrace msgTracer, boolean tracePidInfo)
    {
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
            elevatorEvent = new TrcEvent(RobotParams.HWNAME_ELEVATOR + ".event");
            elevatorEvent.setCallback(this::performAction, elevatorActionParams);
        }
        else
        {
            elevator = null;
            elevatorEvent = null;
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
            armEvent = new TrcEvent(RobotParams.HWNAME_ARM + ".event");
            armEvent.setCallback(this::performAction, armActionParams);
        }
        else
        {
            arm = null;
            armEvent = null;
        }
        // Wrist subsystem.
        if (RobotParams.Preferences.useWrist)
        {
            FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
                .setServoInverted(RobotParams.WRIST_SERVO_INVERTED)
                .setHasFollowerServo(RobotParams.WRIST_HAS_FOLLOWER_SERVO, RobotParams.WRIST_FOLLOWER_SERVO_INVERTED);
            wrist = new FtcServoActuator(RobotParams.HWNAME_WRIST, wristParams, msgTracer).getActuator();
        }
        else
        {
            wrist = null;
        }
    }   //ElevatorArm

    /**
     * This method cancels the elevator and arm operation if there is any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     */
    public void cancel(String owner)
    {
        elevator.stop(owner);
        arm.stop(owner);
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
        if (elevator != null && arm != null  && arm.getPosition() <= RobotParams.ARM_SAFE_POS)
        {
            if (wrist != null)
            {
                // Holding wrist in loading position while zero calibrating.
                wrist.setPosition(RobotParams.WRIST_DOWN_POS);
            }
            // Holding arm in loading position while zero calibrating.
            arm.setPosition(owner, 0.0, RobotParams.ARM_LOAD_POS, true, RobotParams.ARM_POWER_LIMIT, null, 0.0);
            elevator.zeroCalibrate(owner, RobotParams.ELEVATOR_CAL_POWER);
        }
    }   //zeroCalibrate

    /**
     * This method is a callback to perform the specified action when it is safe to do so.
     *
     * @param context specifies the action parameters.
     */
    private void performAction(Object context)
    {
        ActionParams actionParams = (ActionParams) context;

        switch (actionParams.actionType)
        {
            case ElevatorSetPosition:
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();
                    // Move elevator to desired position.
                    elevator.setPosition(
                        actionParams.owner, delay, actionParams.pos, true, actionParams.powerLimit,
                        actionParams.completionEvent, timeout);
                }
                break;

            case ElevatorSetPower:
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If arm has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    // Move elevator with desired power.
                    elevator.setPower(
                        actionParams.owner, delay, actionParams.power, actionParams.duration,
                        actionParams.completionEvent);
                }
                break;

            case ElevatorSetPidPower:
                // Perform action only if already in safe position or arm has moved to safe position.
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    // Move elevator with desired power with PID control.
                    elevator.setPidPower(
                        actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;

            case ArmSetPosition:
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
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
                        actionParams.owner, delay, actionParams.pos, true, actionParams.powerLimit,
                        actionParams.completionEvent, timeout);
                }
                break;

            case ArmSetPower:
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
                {
                    // Delay is only applicable for first movement. If elevator has moved, it already applied delay.
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    // Move arm with desired power.
                    arm.setPower(
                        actionParams.owner, delay, actionParams.power, actionParams.duration,
                        actionParams.completionEvent);
                }
                break;

            case ArmSetPidPower:
                // Perform action only if already in safe position or elevator has moved to safe height.
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
                {
                    // Move arm with desired power with PID control.
                    arm.setPidPower(
                        actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
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
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setLoadingPosition(String owner, double delay, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;
        // Move the wrist in loading position before lowering the elevator.
        wrist.setPosition(owner, 0.0, RobotParams.WRIST_DOWN_POS, null, 0.0);
        // Setting up the elevator operation after the arm is in loading position.
        armActionParams.setPositionParams(
            ActionType.ElevatorSetPosition, owner, 0.0, RobotParams.ELEVATOR_LOAD_POS, RobotParams.ELEVATOR_POWER_LIMIT,
            event, expiredTime);
        // Move the arm to loading position before lowering the elevator.
        if (!armIsSafeToMove(RobotParams.ARM_LOAD_POS))
        {
            // Move elevator to safe position first. Elevator is fast and arm is slow, so we don't have to wait for
            // the elevator to complete its movement.
            elevator.setPosition(
                owner, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, RobotParams.ELEVATOR_POWER_LIMIT, null, 0.0);
        }
        armSetPosition(owner, delay, RobotParams.ARM_LOAD_POS, RobotParams.ARM_POWER_LIMIT, armEvent, 3.0);
    }   //setLoadingPosition

    /**
     * This method sets the elevator, arm, wrist to the scoring position and makes sure it doesn't hit the intake on
     * its way.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param elevatorPos specifies the elevator scoring level.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setScoringPosition(String owner, double delay, double elevatorPos, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;
        // Setting up the arm operation after the elevator is in scoring height.
        // Wrist position will be set in the arm operation.
        elevatorActionParams.setPositionParams(
            ActionType.ArmSetPosition, owner, 0.0, RobotParams.ARM_SCORE_BACKDROP_POS, RobotParams.ARM_POWER_LIMIT,
            event, expiredTime);
        // Move the elevator to scoring height before moving the arm to scoring position.
        elevatorSetPosition(owner, delay, elevatorPos, RobotParams.ELEVATOR_POWER_LIMIT, elevatorEvent, 1.0);
    }   //setScoringPosition

    /**
     * This method sets the elevator and arm the climbing position and makes sure it doesn't hit the intake on
     * its way.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setClimbingPosition(String owner, double delay, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;
        // Setting up the arm operation after the elevator is in scoring height.
        // Wrist position will be set in the arm operation.
        elevatorActionParams.setPositionParams(
                ActionType.ArmSetPosition, owner, 0.0, RobotParams.ARM_CLIMB_POS, RobotParams.ARM_POWER_LIMIT,
                event, expiredTime);
        // Move the elevator to scoring height before moving the arm to scoring position.
        elevatorSetPosition(owner, delay, RobotParams.ELEVATOR_MAX_POS, RobotParams.ELEVATOR_POWER_LIMIT, elevatorEvent, 1.0);
    }   //setClimbingPosition

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

        return currArmPos <= RobotParams.ARM_SAFE_POS ||
               currArmPos >= RobotParams.ARM_FREE_TO_MOVE_POS ||
               elevatorTargetPos >= RobotParams.ELEVATOR_SAFE_POS;
    }   //elevatorIsSafeToMove

    /**
     * This method sets the elevator position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param event specifies the event to signal when the motor operation is completed.
     * @param timeout specifies timeout in seconds.
     */
    public void elevatorSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        armActionParams.setPositionParams(
            ActionType.ElevatorSetPosition, owner, delay, pos, powerLimit, event, expiredTime);
        if (elevatorIsSafeToMove(pos))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, delay, RobotParams.ARM_MIN_POS, true, powerLimit, armEvent, timeout);
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
        armActionParams.setPowerParams(ActionType.ElevatorSetPower, owner, delay, power, duration, event);
        if (power == 0.0 ||
            elevatorIsSafeToMove(power > 0.0? RobotParams.ELEVATOR_MAX_POS: RobotParams.ELEVATOR_MIN_POS))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, delay, RobotParams.ARM_MIN_POS, true, 1.0, armEvent, 0.0);
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
        armActionParams.setPidPowerParams(ActionType.ElevatorSetPidPower, owner, power, minPos, maxPos);
        if (power == 0.0 || elevatorIsSafeToMove(power > 0.0? maxPos: minPos))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, 0.0, RobotParams.ARM_MIN_POS, true, 1.0, armEvent, 0.0);
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
        return elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_POS ||
               arm.getPosition() >= RobotParams.ARM_FREE_TO_MOVE_POS &&
               armTargetPos >= RobotParams.ARM_FREE_TO_MOVE_POS;
    }   //armIsSafeToMove

    /**
     * This method sets the arm position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param pos specifies the position in scaled units to be set.
     * @param powerLimit specifies the maximum power output limits.
     * @param event specifies the event to signal when the motor operation is completed.
     * @param timeout specifies timeout in seconds.
     */
    public void armSetPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        elevatorActionParams.setPositionParams(
            ActionType.ArmSetPosition, owner, delay, pos, powerLimit, event, expiredTime);
        if (armIsSafeToMove(pos))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, delay, RobotParams.ELEVATOR_SAFE_POS, true, powerLimit, elevatorEvent, timeout);
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
        elevatorActionParams.setPowerParams(ActionType.ArmSetPower, owner, delay, power, duration, event);
        if (power == 0.0 ||
            armIsSafeToMove(power > 0.0? RobotParams.ARM_MAX_POS: RobotParams.ARM_MIN_POS))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, delay, RobotParams.ELEVATOR_SAFE_POS, true, 1.0, elevatorEvent, 0.0);
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
        elevatorActionParams.setPidPowerParams(ActionType.ArmSetPidPower, owner, power, minPos, maxPos);
        if (power == 0.0 || armIsSafeToMove(power > 0.0? maxPos: minPos))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, 1.0, elevatorEvent, 0.0);
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

}   //class ElevatorArm
