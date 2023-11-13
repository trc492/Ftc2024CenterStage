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
import teamcode.RobotParams;

public class Intake
{
    private final TrcDbgTrace msgTracer;
    private final FtcDcMotor intakeMotor;
//    private final FtcDistanceSensor intakeSensor;
//    private int pixelCount = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     */
    public Intake(String instanceName, TrcDbgTrace msgTracer)
    {
        this.msgTracer = msgTracer;
        intakeMotor = new FtcDcMotor(instanceName + ".motor");
        intakeMotor.setMotorInverted(RobotParams.INTAKE_MOTOR_INVERTED);
//        if (RobotParams.Preferences.hasIntakeSensor)
//        {
//            intakeSensor = new FtcDistanceSensor(instanceName + ".sensor");
//            TrcTriggerThresholdZones analogTrigger = new TrcTriggerThresholdZones(
//                instanceName + ".analogTrigger", this::getDistance, new double[]{RobotParams.INTAKE_SENSOR_THRESHOLD},
//                false);
//        }
//        else
//        {
//            intakeSensor = null;
//        }
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
    }   //setReverse

//    /**
//     * This method is called when an analog sensor threshold has been crossed.
//     *
//     * @param context specifies the callback context.
//     */
//    private void analogTriggerCallback(Object context)
//    {
//        final String funcName = "analogTriggerEvent";
//        TrcTriggerThresholdZones.CallbackContext callbackContext = (TrcTriggerThresholdZones.CallbackContext) context;
//
//        if (msgTracer != null)
//        {
//            msgTracer.traceInfo(
//                funcName, "Zone=%d->%d, value=%.3f",
//                callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
//        }
//
//        if (callbackContext.prevZone == 1 && callbackContext.currZone == 0)
//        {
//            pixelCount++;
//            if (pixelCount >= 2)
//            {
//                intakeMotor.setPower(0.0, RobotParams.INTAKE_REVERSE_POWER, 0.5);
//            }
//        }
//    }   //analogTriggerCallback
//
//    /**
//     * This method is called the TrcTriggerThresholdZones to get the sensor data.
//     *
//     * @return distance to detected object in inches.
//     */
//    private double getDistance()
//    {
//        TrcSensor.SensorData<Double> data =
//            intakeSensor != null? intakeSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH): null;
//        return data != null && data.value != null? data.value: 0.0;
//    }   //getDistance

}   //class Intake
