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
import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcTriggerThresholdZones;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import teamcode.Robot;
import teamcode.RobotParams;

public class Intake
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final Robot robot;
    private final FtcDcMotor intakeMotor;
    private final FtcDistanceSensor intakeSensor;
    private final TrcTriggerThresholdZones analogTrigger;
    private TrcEvent completionEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param robot specifies the robot object.
     */
    public Intake(String instanceName, Robot robot)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.robot = robot;
        intakeMotor = new FtcDcMotor(instanceName + ".motor");
        intakeMotor.setMotorInverted(RobotParams.INTAKE_MOTOR_INVERTED);
        if (RobotParams.Preferences.hasIntakeSensor)
        {
            intakeSensor = new FtcDistanceSensor(instanceName + ".sensor");
            analogTrigger = new TrcTriggerThresholdZones(
                instanceName + ".analogTrigger", this::getDistance, RobotParams.INTAKE_SENSOR_THRESHOLDS, false);
        }
        else
        {
            intakeSensor = null;
            analogTrigger = null;
        }
    }   //Intake

    /**
     * This method returns the intake motor object.
     *
     * @return intake motor object.
     */
    public FtcDcMotor getIntakeMotor()
    {
        return intakeMotor;
    }   //getIntakeMotor

    public void stop()
    {
        intakeMotor.stop();
    }   //stop

    public void setOn(double delay, double power, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, power, duration, event);
    }   //setOn

    public void setOn(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_FORWARD_POWER, duration, event);
    }   //setOn

    public void setOn(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_FORWARD_POWER: 0.0);
    }   //setOn

    public void setReverse(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_REVERSE_POWER, duration, event);
    }   //setReverse

    public void setReverse(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_REVERSE_POWER: 0.0);
        analogTrigger.disableTrigger();
    }   //setReverse

    public void pickupPixel(boolean on, TrcEvent event)
    {
        setOn(on);
        if (analogTrigger != null)
        {
            if (on)
            {
                completionEvent = event;
                analogTrigger.enableTrigger(this::analogTriggerCallback);
            }
            else
            {
                completionEvent = null;
                analogTrigger.disableTrigger();
            }
        }
    }   //pickupPixel

    public boolean hasTwoPixels()
    {
        return getDistance() < RobotParams.INTAKE_SENSOR_THRESHOLDS[0];
    }   //hasTwoPixels

    /**
     * This method is called when an analog sensor threshold has been crossed.
     *
     * @param context specifies the callback context.
     */
    private void analogTriggerCallback(Object context)
    {
        TrcTriggerThresholdZones.CallbackContext callbackContext = (TrcTriggerThresholdZones.CallbackContext) context;

        tracer.traceDebug(
            instanceName, "Zone=%d->%d, value=%.3f",
            callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
        if (hasTwoPixels())
        {
            analogTrigger.disableTrigger();
            intakeMotor.setPower(0.0, RobotParams.INTAKE_REVERSE_POWER, 0.5);

            if (completionEvent != null)
            {
                completionEvent.signal();
                completionEvent = null;
            }

            if (robot.blinkin != null)
            {
                robot.blinkin.setDetectedPattern(BlinkinLEDs.INTAKE_PIXEL);
            }
        }
    }   //analogTriggerCallback

    /**
     * This method is called the TrcTriggerThresholdZones to get the sensor data.
     *
     * @return distance to detected object in inches.
     */
    public double getDistance()
    {
        TrcSensor.SensorData<Double> data =
            intakeSensor != null? intakeSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH): null;
        return data != null && data.value != null? data.value: 500.0;
    }   //getDistance

}   //class Intake
