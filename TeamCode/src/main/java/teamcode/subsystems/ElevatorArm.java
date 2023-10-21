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
import TrcFtcLib.ftclib.FtcMotorActuator;
import teamcode.RobotParams;

public class ElevatorArm
{
    private enum ActionType
    {
        ZeroCalibrate,
        SetElevatorPosition,
        SetElevatorPower,
        SetElevatorPidPower,
        SetArmPosition,
        SetArmPower,
        SetArmPidPower
    }   //enum ActionType

    private static class ActionParams
    {
        ActionType actionType;
        String owner;
        double delay;
        double position;
        double minPos;
        double maxPos;
        boolean holdTarget;
        double powerLimit;
        double power;
        double duration;
        TrcEvent completionEvent;
        double timeout;

        void setParams(
            ActionType actionType, String owner, double delay, double position, double minPos, double maxPos,
            boolean holdTarget, double powerLimit, double power, double duration, TrcEvent completionEvent,
            double timeout)
        {
            this.actionType = actionType;
            this.owner = owner;
            this.delay = delay;
            this.position = position;
            this.minPos = minPos;
            this.maxPos = maxPos;
            this.holdTarget = holdTarget;
            this.powerLimit = powerLimit;
            this.power = power;
            this.duration = duration;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
        }   //setParams
    }   //class ActionParams

    private final ActionParams elevatorActionParams = new ActionParams();
    public final TrcMotor elevator;
    private final TrcEvent elevatorEvent;

    private final ActionParams armActionParams = new ActionParams();
    public final TrcMotor arm;
    private final TrcEvent armEvent;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     * @param tracePidInfo specifies true to enable tracing PID info, false to disable.
     */
    public ElevatorArm(TrcDbgTrace msgTracer, boolean tracePidInfo)
    {
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

        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
            .setMotorInverted(RobotParams.ARM_MOTOR_INVERTED)
            .setLowerLimitSwitch(RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED)
            .setUpperLimitSwitch(RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(RobotParams.ARM_DEG_SCALE, RobotParams.ARM_OFFSET)
            .setPositionPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.ARM_PRESETS);
        arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, armParams, msgTracer, tracePidInfo).getActuator();
        armEvent = new TrcEvent(RobotParams.HWNAME_ARM + ".event");
        armEvent.setCallback(this::performAction, armActionParams);
    }   //ElevatorArm

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
            case ZeroCalibrate:
                // Arm has finished zero calibration, let's do elevator zero calibration.
                elevator.zeroCalibrate(actionParams.owner, actionParams.power);
                break;

            case SetElevatorPosition:
                elevator.setPosition(
                    actionParams.owner, actionParams.delay, actionParams.position, actionParams.holdTarget,
                    actionParams.powerLimit, actionParams.completionEvent, actionParams.timeout);
                break;

            case SetElevatorPower:
                elevator.setPower(
                    actionParams.owner, actionParams.delay, actionParams.power, actionParams.duration,
                    actionParams.completionEvent);
                break;

            case SetElevatorPidPower:
                elevator.setPidPower(
                    actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos,
                    actionParams.holdTarget);
                break;

            case SetArmPosition:
                arm.setPosition(
                    actionParams.owner, actionParams.delay, actionParams.position, actionParams.holdTarget,
                    actionParams.powerLimit, actionParams.completionEvent, actionParams.timeout);
                break;

            case SetArmPower:
                arm.setPower(
                    actionParams.owner, actionParams.delay, actionParams.power, actionParams.duration,
                    actionParams.completionEvent);
                break;

            case SetArmPidPower:
                arm.setPidPower(
                    actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos,
                    actionParams.holdTarget);
                break;
        }
    }   //performAction

    /**
     * This method zero calibrates the elevator and arm.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        armActionParams.setParams(
            ActionType.ZeroCalibrate, owner, 0.0, 0.0, 0.0, 0.0, false, 0.0, RobotParams.ELEVATOR_CAL_POWER, 0.0, null,
            0.0);
        arm.zeroCalibrate(owner, RobotParams.ARM_CAL_POWER, armEvent);
    }   //zeroCalibrate

    //
    // Elevator subsystem methods.
    //

    /**
     * This method sets the elevator position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param position specifies the position in scaled units to be set.
     * @param holdTarget specifies true to hold position target, false otherwise.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when the motor operation is completed.
     * @param timeout specifies timeout in seconds.
     */
    public void setElevatorPosition(
        String owner, double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        elevatorActionParams.setParams(
            ActionType.SetElevatorPosition, owner, delay, position, 0.0, 0.0, holdTarget, powerLimit, 0.0, 0.0,
            completionEvent, timeout);
        if (isElevatorSafeToMove(position))
        {
            performAction(elevatorActionParams);
        }
        else
        {
            lowerArmToMinPos(owner);
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
     * @param completionEvent specifies the event to signal when the motor operation is completed.
     */
    public void setElevatorPower(String owner, double delay, double power, double duration, TrcEvent completionEvent)
    {
        elevatorActionParams.setParams(
            ActionType.SetElevatorPower, owner, delay, 0.0, 0.0, 0.0, false, 0.0, power, duration, completionEvent,
            0.0);
        if (power == 0.0 ||
            isElevatorSafeToMove(power > 0.0? RobotParams.ELEVATOR_MAX_POS: RobotParams.ELEVATOR_MIN_POS))
        {
            performAction(elevatorActionParams);
        }
        else
        {
            lowerArmToMinPos(owner);
        }
    }   //setElevatorPower

    /**
     * This method sets the elevator power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setElevatorPidPower(String owner, double power, double minPos, double maxPos, boolean holdTarget)
    {
        elevatorActionParams.setParams(
            ActionType.SetElevatorPidPower, owner, 0.0, 0.0, minPos, maxPos, holdTarget, 0.0, power, 0.0, null, 0.0);
        if (power == 0.0 || isElevatorSafeToMove(power > 0.0? maxPos: minPos))
        {
            performAction(elevatorActionParams);
        }
        else
        {
            lowerArmToMinPos(owner);
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
                owner, delay, elevator.getPresetPosition(presetIndex), true, powerLimit, event, timeout);
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
            setElevatorPosition(owner, 0.0, elevator.getPresetPosition(index), true, powerLimit, null, 0.0);
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
            setElevatorPosition(owner, 0.0, elevator.getPresetPosition(index), true, powerLimit, null, 0.0);
        }
    }   //elevatorPresetPositionDown

    /**
     * This method raises the elevator to safe height before moving the arm.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    private void raiseElevatorToSafeHeight(String owner)
    {
        elevator.setPosition(owner, 0.0, RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, elevatorEvent, 0.0);
    }   //raiseElevatorToSafeHeight

    /**
     * This method checks if the elevator is safe to move to the target position so it doesn't hit the intake.
     *
     * @param targetElevatorPos specifies the target elevator position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean isElevatorSafeToMove(double targetElevatorPos)
    {
        double currArmPos = arm.getPosition();
        double currElevatorPos = elevator.getPosition();

        return currArmPos >= RobotParams.ARM_FREE_TO_MOVE_POSITION ||
               currArmPos <= RobotParams.ARM_TUGGEDIN_THRESHOLD ||
               currElevatorPos >= RobotParams.ELEVATOR_SAFE_HEIGHT &&
               targetElevatorPos >= RobotParams.ELEVATOR_SAFE_HEIGHT;
    }   //isElevatorSafeToMove

    //
    // Arm subsystem methods.
    //

    /**
     * This method sets the arm position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param position specifies the position in scaled units to be set.
     * @param holdTarget specifies true to hold position target, false otherwise.
     * @param powerLimit specifies the maximum power output limits.
     * @param completionEvent specifies the event to signal when the motor operation is completed.
     * @param timeout specifies timeout in seconds.
     */
    public void setArmPosition(
        String owner, double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        armActionParams.setParams(
            ActionType.SetArmPosition, owner, delay, position, 0.0, 0.0, holdTarget, powerLimit, 0.0, 0.0,
            completionEvent, timeout);
        if (isArmSafeToMove(position))
        {
            performAction(armActionParams);
        }
        else
        {
            raiseElevatorToSafeHeight(owner);
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
     * @param completionEvent specifies the event to signal when the motor operation is completed.
     */
    public void setArmPower(String owner, double delay, double power, double duration, TrcEvent completionEvent)
    {
        armActionParams.setParams(
            ActionType.SetArmPower, owner, delay, 0.0, 0.0, 0.0, false, 0.0, power, duration, completionEvent, 0.0);
        if (power == 0.0 ||
            isArmSafeToMove(power > 0.0? RobotParams.ARM_MAX_POS: RobotParams.ARM_MIN_POS))
        {
            performAction(armActionParams);
        }
        else
        {
            raiseElevatorToSafeHeight(owner);
        }
    }   //setArmPower

    /**
     * This method sets the arm power with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setArmPidPower(String owner, double power, double minPos, double maxPos, boolean holdTarget)
    {
        armActionParams.setParams(
            ActionType.SetArmPidPower, owner, 0.0, 0.0, minPos, maxPos, holdTarget, 0.0, power, 0.0, null, 0.0);
        if (power == 0.0 || isArmSafeToMove(power > 0.0? maxPos: minPos))
        {
            performAction(armActionParams);
        }
        else
        {
            raiseElevatorToSafeHeight(owner);
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
            setArmPosition(owner, delay, arm.getPresetPosition(presetIndex), true, powerLimit, event, timeout);
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
            setArmPosition(owner, 0.0, arm.getPresetPosition(index), true, powerLimit, null, 0.0);
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
            setArmPosition(owner, 0.0, arm.getPresetPosition(index), true, powerLimit, null, 0.0);
        }
    }   //armPresetPositionDown

    /**
     * This method lowers the arm to tugged-in position before moving the elevator.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    private void lowerArmToMinPos(String owner)
    {
        arm.setPosition(owner, 0.0, RobotParams.ARM_MIN_POS, true, 1.0, armEvent, 0.0);
    }   //lowerArmToMinPos

    /**
     * This method checks if the arm is safe to move so it doesn't hit the intake.
     *
     * @param targetArmPos specifies the target arm position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean isArmSafeToMove(double targetArmPos)
    {
        double currArmPos = arm.getPosition();

        return elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_HEIGHT ||
               currArmPos >= RobotParams.ARM_FREE_TO_MOVE_POSITION &&
               targetArmPos >= RobotParams.ARM_FREE_TO_MOVE_POSITION;
    }   //isArmSafeToMove

}   //class ElevatorArm
