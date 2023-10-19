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
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;
import teamcode.RobotParams;

public class Elevator extends FtcMotorActuator
{
    private enum ActionType
    {
        SetPosition,
        SetPower,
        SetPidPower,
        ZeroCalibrate
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
            ActionType actionType, String owner, double delay, double position, double minPos,
            double maxPos, boolean holdTarget, double powerLimit, double power, double duration,
            TrcEvent completionEvent, double timeout)
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

    private final ActionParams actionParams = new ActionParams();
    private final TrcEvent armEvent;
    private FtcServo armActuator;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param motorParams specifies motor params for the FtcServoActuator.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     * @param tracePidInfo specifies true to enable tracing PID info, false to disable.
     */
    public Elevator(FtcMotorActuator.MotorParams motorParams, TrcDbgTrace msgTracer, boolean tracePidInfo)
    {
        super(RobotParams.HWNAME_ELEVATOR, motorParams, msgTracer, tracePidInfo);
        armEvent = new TrcEvent(instanceName + ".elevatorEvent");
        armEvent.setCallback(this::performAction, null);
    }   //Elevator

    /**
     * This method allows the caller to provide the arm actuator so that the elevator can move the arm out of the way
     * to avoid hitting the intake.
     *
     * @param armActuator specifies the arm actuator object.
     */
    public void setArmActuator(FtcServo armActuator)
    {
        this.armActuator = armActuator;
    }   //setArmActuator

    /**
     * This method is a callback to perform the specified action when it is safe to do so.
     *
     * @param context not used.
     */
    private void performAction(Object context)
    {
        switch (actionParams.actionType)
        {
            case SetPosition:
                actuator.setPosition(
                    actionParams.owner, actionParams.delay, actionParams.position, actionParams.holdTarget,
                    actionParams.powerLimit, actionParams.completionEvent, actionParams.timeout);
                break;

            case SetPower:
                actuator.setPower(
                    actionParams.owner, actionParams.delay, actionParams.power, actionParams.duration,
                    actionParams.completionEvent);
                break;

            case SetPidPower:
                actuator.setPidPower(
                    actionParams.owner, actionParams.power, actionParams.minPos, actionParams.maxPos,
                    actionParams.holdTarget);
                break;

            case ZeroCalibrate:
                actuator.zeroCalibrate(actionParams.owner, actionParams.power);
                break;
        }
    }   //performSetPosition

    /**
     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
     * and set it to power control mode.
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
    public void setPosition(
        String owner, double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        actionParams.setParams(
            ActionType.SetPosition, owner, delay, position, 0.0, 0.0, holdTarget, powerLimit, 0.0, 0.0, completionEvent,
            timeout);
        if (safeToMove(position))
        {
            performAction(null);
        }
        else
        {
            lowerArmToMinPos(owner);
        }
    }   //setPosition

    /**
     * This method sets the motor power. If the motor is not in the correct control mode, it will stop the motor
     * and set it to power control mode. Optionally, you can specify a delay before running the motor and a duration
     * for which the motor will be turned off afterwards.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
     * @param power specifies the percentage power (range -1.0 to 1.0).
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param completionEvent specifies the event to signal when the motor operation is completed.
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent completionEvent)
    {
        actionParams.setParams(
            ActionType.SetPower, owner, delay, 0.0, 0.0, 0.0, false, 0.0, power, duration, completionEvent, 0.0);
        if (power == 0.0 || safeToMove(power > 0.0? RobotParams.ELEVATOR_MAX_POS: RobotParams.ELEVATOR_MIN_POS))
        {
            performAction(null);
        }
        else
        {
            lowerArmToMinPos(owner);
        }
    }   //setPower

    /**
     * This method sets the motor power with PID control. This is basically the same as setPosition but with
     * dynamically changing powerLimit. The motor will be under position PID control and the power specifies the
     * maximum limit of how fast the motor can go. The actual motor power is controlled by a PID controller with
     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
     * value will specify the maximum limit of the elevator power. So if the joystick is only pushed half way, the
     * elevator will only go half power even though it is far away from the target.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPidPower(String owner, double power, double minPos, double maxPos, boolean holdTarget)
    {
        actionParams.setParams(
            ActionType.SetPidPower, owner, 0.0, 0.0, minPos, maxPos, holdTarget, 0.0, power, 0.0, null, 0.0);
        if (power == 0.0 || safeToMove(power > 0.0? maxPos: minPos))
        {
            performAction(null);
        }
        else
        {
            lowerArmToMinPos(owner);
        }
    }   //setPidPower

    /**
     * This method zero calibrates the motor with the specified calibration power.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
     *        the desire direction of movement.
     */
    public void zeroCalibrate(String owner, double calPower)
    {
        actionParams.setParams(
            ActionType.ZeroCalibrate, owner, 0.0, 0.0, 0.0, 0.0, false, 0.0, calPower, 0.0, null, 0.0);
        if (safeToMove(RobotParams.ELEVATOR_MIN_POS))
        {
            performAction(null);
        }
        else
        {
            lowerArmToMinPos(owner);
        }
    }   //zeroCalibrate

    /**
     * This method sets the servo to the specified preset position.
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
    public void setPresetPosition(
        String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
    {
        if (actuator.validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, actuator.getPresetPosition(presetIndex), true, powerLimit, event, timeout);
        }
    }   //setPresetPosition

    /**
     * This method sets the actuator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void presetPositionUp(String owner, double powerLimit)
    {
        int index = actuator.nextPresetIndexUp();

        if (index != -1)
        {
            setPosition(owner, 0.0, actuator.getPresetPosition(index), true, powerLimit, null, 0.0);
        }
    }   //presetPositionUp

    /**
     * This method sets the actuator to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the power limit applied to the elevator.
     */
    public void presetPositionDown(String owner, double powerLimit)
    {
        int index = actuator.nextPresetIndexDown();

        if (index != -1)
        {
            setPosition(owner, 0.0, actuator.getPresetPosition(index), true, powerLimit, null, 0.0);
        }
    }   //presetPositionDown

    /**
     * This method checks if the elevator is safe to move to the target position so it doesn't hit the intake.
     *
     * @param targetElevatorPos specifies the target elevator position.
     * @return true if it is safe to move, false otherwise.
     */
    private boolean safeToMove(double targetElevatorPos)
    {
        double currArmPos = armActuator.getPosition();
        double currElevatorPos = actuator.getPosition();
        return currArmPos >= RobotParams.ARM_FREE_TO_MOVE_POSITION ||
               currArmPos <= RobotParams.ARM_TUGIN_THRESHOLD ||
               currElevatorPos >= RobotParams.ELEVATOR_SAFE_HEIGHT &&
               targetElevatorPos >= RobotParams.ELEVATOR_SAFE_HEIGHT;
    }   //safeToMove

    /**
     * This method lowers the arm to tugged-in position before moving the elevator.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    private void lowerArmToMinPos(String owner)
    {
        armActuator.setPosition(owner, 0.0, RobotParams.ARM_PHYSICAL_MIN_POS, armEvent, 0.0);
    }   //lowerArmToMinPos

}   //class Elevator
