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
    private static class SetPositionParams
    {
        String owner;
        double delay;
        double position;
        boolean holdTarget;
        double powerLimit;
        TrcEvent completionEvent;
        double timeout;

        void setParams(
            String owner, double delay, double position, boolean holdTarget, double powerLimit,
            TrcEvent completionEvent, double timeout)
        {
            this.owner = owner;
            this.delay = delay;
            this.position = position;
            this.holdTarget = holdTarget;
            this.powerLimit = powerLimit;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
        }   //setParams

    }   //class SetPositionParams

    private final SetPositionParams setPositionParams = new SetPositionParams();
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
        armEvent.setCallback(this::performSetPosition, setPositionParams);
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
     * This method is a callback to perform the setPosition operation when it is safe to do so.
     *
     * @param context specifies the setPosition parameters.
     */
    private void performSetPosition(Object context)
    {
        SetPositionParams setPositionParams = (SetPositionParams) context;

        actuator.setPosition(
            setPositionParams.owner, setPositionParams.delay, setPositionParams.position, setPositionParams.holdTarget,
            setPositionParams.powerLimit, setPositionParams.completionEvent, setPositionParams.timeout);
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
        setPositionParams.setParams(owner, delay, position, holdTarget, powerLimit, completionEvent, timeout);
        if (armActuator.getPosition() > RobotParams.ELEVATOR_SAFE_HEIGHT &&
            actuator.getPosition() > RobotParams.ARM_FREE_TO_MOVE_POSITION)
        {
            // Raise elevator before moving the arm.
            armActuator.setPosition(owner, 0.0, RobotParams.ARM_FREE_TO_MOVE_POSITION, armEvent, 0.0);
        }
        else
        {
            performSetPosition(setPositionParams);
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
     * @param event specifies the event to signal when the motor operation is completed
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        actuator.setPower(owner, delay, power, duration, event);
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
        actuator.setPidPower(owner, power, minPos, maxPos, holdTarget);
    }   //setPidPower

    /**
     * This method sets the motor to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the maximum power limit.
     */
    public void presetPositionUp(String owner, double powerLimit)
    {
        actuator.presetPositionUp(owner, powerLimit);
    }   //presetPositionUp

    /**
     * This method sets the motor to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the maximum power limit.
     */
    public void presetPositionDown(String owner, double powerLimit)
    {
        actuator.presetPositionDown(owner, powerLimit);
    }   //presetPositionDown

    /**
     * This method zero calibrates the motor with the specified calibration power.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
     *        the desire direction of movement.
     */
    public void zeroCalibrate(String owner, double calPower)
    {
        actuator.zeroCalibrate(owner, calPower);
    }   //zeroCalibrate

}   //class Elevator
