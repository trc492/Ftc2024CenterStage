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
        SetupPositions,
        SetupCompleted,
        SetElevatorPosition,
        SetElevatorPower,
        SetElevatorPidPower,
        SetArmPosition,
        SetArmPower,
        SetArmPidPower
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

    // Elevator subsystem.
    private final ActionParams elevatorActionParams = new ActionParams();
    public final TrcMotor elevator;
    private final TrcEvent elevatorEvent;
    // Arm subsystem.
    private final ActionParams armActionParams = new ActionParams();
    public final TrcMotor arm;
    private final TrcEvent armEvent;
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
        FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
            .setMotorInverted(RobotParams.ELEVATOR_MOTOR_INVERTED)
            .setLowerLimitSwitch(RobotParams.ELEVATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED)
            .setUpperLimitSwitch(RobotParams.ELEVATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
            .setPositionPresets(RobotParams.ELEVATOR_PRESET_TOLERANCE, RobotParams.ELEVATOR_PRESETS);
        elevator =
            new FtcMotorActuator(RobotParams.HWNAME_ELEVATOR, elevatorParams, msgTracer, tracePidInfo).getActuator();
        elevatorEvent = new TrcEvent(RobotParams.HWNAME_ELEVATOR + ".event");
        elevatorEvent.setCallback(this::performAction, elevatorActionParams);
        // Arm subsystem.
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
            .setMotorInverted(RobotParams.ARM_MOTOR_INVERTED)
            .setSlaveMotor(RobotParams.ARM_HAS_SLAVE_MOTOR, RobotParams.ARM_SLAVE_MOTOR_INVERTED)
            .setLowerLimitSwitch(RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED)
            .setUpperLimitSwitch(RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED)
            .setExternalEncoder(
                RobotParams.ARM_HAS_EXTERNAL_ENCODER, RobotParams.ARM_ENCODER_INVERTED,
                RobotParams.ARM_ENCODER_ABSOLUTE)
            .setPositionScaleAndOffset(RobotParams.ARM_DEG_SCALE, RobotParams.ARM_OFFSET)
            .setPositionPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.ARM_PRESETS);
        arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, true, armParams, msgTracer, tracePidInfo).getActuator();
        armEvent = new TrcEvent(RobotParams.HWNAME_ARM + ".event");
        armEvent.setCallback(this::performAction, armActionParams);
        // Wrist subsystem.
        FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
            .setServoInverted(RobotParams.WRIST_SERVO_INVERTED)
            .setHasServo2(RobotParams.WRIST_HAS_SLAVE_SERVO, RobotParams.WRIST_SLAVE_SERVO_INVERTED)
            .setPhysicalPosRange(RobotParams.WRIST_MIN_POS, RobotParams.WRIST_MAX_POS)
            .setPositionPresets(RobotParams.WRIST_PRESET_TOLERANCE, RobotParams.WRIST_PRESETS);
        wrist = new FtcServoActuator(RobotParams.HWNAME_WRIST, wristParams, msgTracer).getActuator();
    }   //ElevatorArm

    /**
     * This method zero calibrates the elevator. Arm has absolute encoder and doesn't need zero calibration.
     * Note: This assumes the arm is at safe position or elevator zero calibration won't happen.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (arm.getPosition() <= RobotParams.ARM_SAFE_POS)
        {
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
            case SetupPositions:
                if (actionParams.safeToMove || elevatorEvent.isSignaled() && armEvent.isSignaled())
                {
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();

                    actionParams.actionType = ActionType.SetupCompleted;
                    elevatorEvent.clear();
                    armEvent.clear();
                    elevator.setPosition(
                        elevatorActionParams.owner, delay, elevatorActionParams.pos, true,
                        elevatorActionParams.powerLimit, elevatorEvent, timeout);
                    arm.setPosition(
                        armActionParams.owner, delay, armActionParams.pos, true, armActionParams.powerLimit, armEvent,
                        timeout);
                }
                break;

            case SetupCompleted:
                if (elevatorEvent.isSignaled() && armEvent.isSignaled() && actionParams.completionEvent != null)
                {
                    actionParams.completionEvent.signal();
                }
                break;

            case SetElevatorPosition:
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();

                    elevator.setPosition(
                        actionParams.owner, delay, actionParams.pos, true, actionParams.powerLimit,
                        actionParams.completionEvent, timeout);
                }
                break;

            case SetElevatorPower:
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;

                    elevator.setPower(
                        actionParams.owner, delay, actionParams.power, actionParams.duration,
                        actionParams.completionEvent);
                }
                break;

            case SetElevatorPidPower:
                if (actionParams.safeToMove || armEvent.isSignaled())
                {
                    elevator.setPidPower(
                        actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;

            case SetArmPosition:
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
                {
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;
                    double timeout =
                        actionParams.expiredTime == 0.0? 0.0: actionParams.expiredTime - TrcTimer.getCurrentTime();

                    arm.setPosition(
                        actionParams.owner, delay, actionParams.pos, true, actionParams.powerLimit,
                        actionParams.completionEvent, timeout);
                }
                break;

            case SetArmPower:
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
                {
                    double delay = actionParams.safeToMove? actionParams.delay: 0.0;

                    arm.setPower(
                        actionParams.owner, delay, actionParams.power, actionParams.duration,
                        actionParams.completionEvent);
                }
                break;

            case SetArmPidPower:
                if (actionParams.safeToMove || elevatorEvent.isSignaled())
                {
                    arm.setPidPower(
                        actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
                }
                break;
        }
    }   //performAction

    /**
     * This method checks if the specified elevator and arm positions are at home positions.
     *
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @return true if both positions are at home.
     */
    private boolean isHomePosition(double elevatorPos, double armPos)
    {
        return elevatorPos < RobotParams.ELEVATOR_SAFE_POS && armPos < RobotParams.ARM_SAFE_POS;
    }   //isHomePosition

    /**
     * This method set up the elevator and arm to the given positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param delay specifies the delay before moving in seconds.
     * @param elevatorPos specifies the elevator position.
     * @param elevatorPowerLimit specifies the elevator power limit.
     * @param armPos specifies the arm position.
     * @param armPowerLimit specifies the arm power limit.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setupPositions(
        String owner, double delay, double elevatorPos, double elevatorPowerLimit, double armPos, double armPowerLimit,
        TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;
        boolean currPosIsHome = isHomePosition(elevator.getPosition(), arm.getPosition());
        boolean targetIsHome = isHomePosition(elevatorPos, armPos);

        elevatorActionParams.setPositionParams(
            ActionType.SetupPositions, owner, delay, elevatorPos, elevatorPowerLimit, event, expiredTime);
        armActionParams.setPositionParams(
            ActionType.SetupPositions, owner, delay, armPos, armPowerLimit, event, expiredTime);
        if (currPosIsHome ^ targetIsHome)
        {
            // Either current position is home and target is not home or current position is not home and target is
            // home. In either of these scenarios, we must move the elevator and arm to safe positions first which is
            // ELEVATOR_SAFE_POS and ARM_MIN_POS.
            elevatorEvent.clear();
            armEvent.clear();
            elevator.setPosition(
                owner, delay, RobotParams.ELEVATOR_SAFE_POS, true, elevatorPowerLimit, elevatorEvent, timeout);
            arm.setPosition(owner, delay, RobotParams.ARM_MIN_POS, true, armPowerLimit, armEvent, timeout);
        }
        else
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
    }   //setupPositions

    /**
     * This method set up the elevator and arm to the given positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param elevatorPowerLimit specifies the elevator power limit.
     * @param armPos specifies the arm position.
     * @param armPowerLimit specifies the arm power limit.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setupPositions(
        String owner, double elevatorPos, double elevatorPowerLimit, double armPos, double armPowerLimit,
        TrcEvent event, double timeout)
    {
        setupPositions(owner, 0.0, elevatorPos, elevatorPowerLimit, armPos, armPowerLimit, event, timeout);
    }   //setupPositions

    /**
     * This method set up the elevator and arm to the given positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param elevatorPowerLimit specifies the elevator power limit.
     * @param armPos specifies the arm position.
     * @param armPowerLimit specifies the arm power limit.
     */
    public void setupPositions(
        String owner, double elevatorPos, double elevatorPowerLimit, double armPos, double armPowerLimit)
    {
        setupPositions(owner, 0.0, elevatorPos, elevatorPowerLimit, armPos, armPowerLimit, null, 0.0);
    }   //setupPositions

    /**
     * This method set up the elevator and arm to the given positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
     */
    public void setupPositions(
        String owner, double elevatorPos, double armPos, TrcEvent event, double timeout)
    {
        setupPositions(owner, 0.0, elevatorPos, 1.0, armPos, 1.0, event, timeout);
    }   //setupPositions

    /**
     * This method set up the elevator and arm to the given positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     */
    public void setupPositions(String owner, double elevatorPos, double armPos)
    {
        setupPositions(owner, 0.0, elevatorPos, 1.0, armPos, 1.0, null, 0.0);
    }   //setupPositions

    //
    // Elevator subsystem methods.
    //

    /**
     * This method checks if the elevator is safe to move to the target position so it doesn't hit the intake.
     *
     * @param elevatorTargetPos specifies the elevator target position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean isElevatorSafeToMove(double elevatorTargetPos)
    {
        double currArmPos = arm.getPosition();

        return currArmPos <= RobotParams.ARM_SAFE_POS ||
               currArmPos >= RobotParams.ARM_FREE_TO_MOVE_POS ||
               elevatorTargetPos >= RobotParams.ELEVATOR_SAFE_POS;
    }   //isElevatorSafeToMove

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
    public void setElevatorPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        elevatorActionParams.setPositionParams(
            ActionType.SetElevatorPosition, owner, delay, pos, powerLimit, event, expiredTime);
        if (isElevatorSafeToMove(pos))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, delay, RobotParams.ARM_MIN_POS, true, powerLimit, armEvent, timeout);
        }
    }   //setElevatorPosition

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
    public void setElevatorPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        elevatorActionParams.setPowerParams(ActionType.SetElevatorPower, owner, delay, power, duration, event);
        if (power == 0.0 ||
            isElevatorSafeToMove(power > 0.0? RobotParams.ELEVATOR_MAX_POS: RobotParams.ELEVATOR_MIN_POS))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, delay, RobotParams.ARM_MIN_POS, true, 1.0, armEvent, 0.0);
        }
    }   //setElevatorPower

    /**
     * This method sets the elevator power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     */
    public void setElevatorPidPower(String owner, double power, double minPos, double maxPos)
    {
        elevatorActionParams.setPidPowerParams(ActionType.SetElevatorPidPower, owner, power, minPos, maxPos);
        if (power == 0.0 || isElevatorSafeToMove(power > 0.0? maxPos: minPos))
        {
            elevatorActionParams.safeToMove = true;
            performAction(elevatorActionParams);
        }
        else
        {
            armEvent.clear();
            arm.setPosition(owner, 0.0, RobotParams.ARM_MIN_POS, true, 1.0, armEvent, 0.0);
        }
    }   //setElevatorPidPower

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
    public void setElevatorPresetPosition(
        String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
    {
        if (elevator.validatePresetIndex(presetIndex))
        {
            setElevatorPosition(
                owner, delay, elevator.getPresetPosition(presetIndex), powerLimit, event, timeout);
        }
    }   //setElevatorPresetPosition

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
            setElevatorPosition(owner, 0.0, elevator.getPresetPosition(index), powerLimit, null, 0.0);
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
            setElevatorPosition(owner, 0.0, elevator.getPresetPosition(index), powerLimit, null, 0.0);
        }
    }   //elevatorPresetPositionDown

    //
    // Arm subsystem methods.
    //

    /**
     * This method checks if the arm is safe to move so it doesn't hit the intake.
     *
     * @param armTargetPos specifies the arm target position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean isArmSafeToMove(double armTargetPos)
    {
        return elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_POS ||
               arm.getPosition() >= RobotParams.ARM_FREE_TO_MOVE_POS &&
               armTargetPos >= RobotParams.ARM_FREE_TO_MOVE_POS;
    }   //isArmSafeToMove

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
    public void setArmPosition(
        String owner, double delay, double pos, double powerLimit, TrcEvent event, double timeout)
    {
        double expiredTime = timeout == 0.0? 0.0: TrcTimer.getCurrentTime() + timeout;

        armActionParams.setPositionParams(
            ActionType.SetArmPosition, owner, delay, pos, powerLimit, event, expiredTime);
        if (isArmSafeToMove(pos))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, delay, RobotParams.ELEVATOR_SAFE_POS, true, powerLimit, elevatorEvent, timeout);
        }
    }   //setArmPosition

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
    public void setArmPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        armActionParams.setPowerParams(ActionType.SetArmPower, owner, delay, power, duration, event);
        if (power == 0.0 ||
            isArmSafeToMove(power > 0.0? RobotParams.ARM_MAX_POS: RobotParams.ARM_MIN_POS))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, delay, RobotParams.ELEVATOR_SAFE_POS, true, 1.0, elevatorEvent, 0.0);
        }
    }   //setArmPower

    /**
     * This method sets the arm power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     */
    public void setArmPidPower(String owner, double power, double minPos, double maxPos)
    {
        armActionParams.setPidPowerParams(ActionType.SetArmPidPower, owner, power, minPos, maxPos);
        if (power == 0.0 || isArmSafeToMove(power > 0.0? maxPos: minPos))
        {
            armActionParams.safeToMove = true;
            performAction(armActionParams);
        }
        else
        {
            elevatorEvent.clear();
            elevator.setPosition(owner, 0.0, RobotParams.ELEVATOR_SAFE_POS, true, 1.0, elevatorEvent, 0.0);
        }
    }   //setArmPidPower

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
    public void setArmPresetPosition(
        String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
    {
        if (arm.validatePresetIndex(presetIndex))
        {
            setArmPosition(owner, delay, arm.getPresetPosition(presetIndex), powerLimit, event, timeout);
        }
    }   //setArmPresetPosition

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
            setArmPosition(owner, 0.0, arm.getPresetPosition(index), powerLimit, null, 0.0);
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
            setArmPosition(owner, 0.0, arm.getPresetPosition(index), powerLimit, null, 0.0);
        }
    }   //armPresetPositionDown

}   //class ElevatorArm
