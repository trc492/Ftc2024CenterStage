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
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcServoActuator;
import teamcode.RobotParams;

public class Arm extends FtcServoActuator
{
    private static class SetPositionParams
    {
        String owner;
        double delay;
        double position;
        TrcEvent completionEvent;
        double timeout;

        void setParams(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
        {
            this.owner = owner;
            this.delay = delay;
            this.position = position;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
        }   //setParams

    }   //class SetPositionParams

    private final SetPositionParams setPositionParams = new SetPositionParams();
    private final TrcEvent elevatorEvent;
    private FtcDcMotor elevatorActuator;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param servoParams specifies servo params for the FtcServoActuator.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     */
    public Arm(FtcServoActuator.ServoParams servoParams, TrcDbgTrace msgTracer)
    {
        super(RobotParams.HWNAME_ARM, servoParams, msgTracer);
        elevatorEvent = new TrcEvent(instanceName + ".elevatorEvent");
        elevatorEvent.setCallback(this::performSetPosition, setPositionParams);
    }   //Arm

    /**
     * This method allows the caller to provide the elevator actuator so that the arm can move the elevator out of the
     * way to avoid hitting the intake.
     *
     * @param elevatorActuator specifies the elevator actuator object.
     */
    public void setElevatorActuator(FtcDcMotor elevatorActuator)
    {
        this.elevatorActuator = elevatorActuator;
    }   //setElevatorActuator

    /**
     * This method is a callback to perform the setPosition operation when it is safe to do so.
     *
     * @param context specifies the setPosition parameters.
     */
    private void performSetPosition(Object context)
    {
        SetPositionParams setPositionParams = (SetPositionParams) context;

        actuator.setPosition(
            setPositionParams.owner, setPositionParams.delay, setPositionParams.position,
            setPositionParams.completionEvent,
            setPositionParams.timeout);
    }   //performSetPosition

    /**
     * This method sets the servo position. By default, the servo maps its physical position the same as its logical
     * position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world physical range
     * (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets event after
     * the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {
        setPositionParams.setParams(owner, delay, position, completionEvent, timeout);
        if (elevatorActuator.getPosition() < RobotParams.ELEVATOR_SAFE_HEIGHT &&
            actuator.getPosition() < RobotParams.ARM_FREE_TO_MOVE_POSITION)
        {
            // Raise elevator before moving the arm.
            elevatorActuator.setPosition(owner, 0.0, RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, elevatorEvent, 0.0);
        }
        else
        {
            performSetPosition(setPositionParams);
        }
    }   //setPosition

    /**
     * This method sets power of the continuous servo.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay speciifes the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(String owner, double delay, double power)
    {
        actuator.setPower(owner, delay, power);
    }   //setPower

    /**
     * This method sets the actuator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     */
    public void presetPositionUp(String owner)
    {
        actuator.presetPositionUp(owner);
    }   //presetPositionUp

    /**
     * This method sets the actuator to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     */
    public void presetPositionDown(String owner)
    {
        actuator.presetPositionDown(owner);
    }   //presetPositionDown

}   //class Arm
