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
import TrcFtcLib.ftclib.FtcServo;
import teamcode.RobotParams;

public class PixelTray
{
    private final String instanceName;
    private final TrcDbgTrace msgTracer;
    private final FtcServo servo1, servo2;
    private boolean gate1Opened, gate2Opened;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     */
    public PixelTray(String instanceName, TrcDbgTrace msgTracer)
    {
        this.instanceName = instanceName;
        this.msgTracer = msgTracer;
        servo1 = new FtcServo(instanceName + ".servo1");
        servo2 = new FtcServo(instanceName + ".servo2");
        setGate1Opened(false, null);
        setGate2Opened(false, null);
    }   //PixelTray

    /**
     * This method returns the state info of the Pixel Tray in string format.
     *
     * @return state info of the Pixel Tray.
     */
    @Override
    public String toString()
    {
        return instanceName + ": gate1=" + gate1Opened + ", gate2=" + gate2Opened;
    }   //toString

    /**
     * This method opens and closes the pixel gate 1.
     *
     * @param opened specifies true to open the gate, false to close.
     * @param event specifies the event to signal after open/close wait time expired.
     */
    public void setGate1Opened(boolean opened, TrcEvent event)
    {
        final String funcName = "setGate1Opened";

        gate1Opened = opened;
        servo1.setPosition(
            opened? RobotParams.PIXELTRAY_GATE1_OPEN: RobotParams.PIXELTRAY_GATE1_CLOSE,
            event, RobotParams.PIXELTRAY_OPEN_CLOSE_TIME);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "Gate 1 set to opened = %s", opened);
        }
    }   //setGate1Opened

    /**
     * This method returns the state of gate 1.
     *
     * @return true if gate is opened, false if closed.
     */
    public boolean isGate1Opened()
    {
        return gate1Opened;
    }   //isGate1Opened

    /**
     * This method opens and closes the pixel gate 2.
     *
     * @param opened specifies true to open the gate, false to close.
     * @param event specifies the event to signal after open/close wait time expired.
     */
    public void setGate2Opened(boolean opened, TrcEvent event)
    {
        final String funcName = "setGate2Opened";

        gate2Opened = opened;
        servo2.setPosition(
            opened? RobotParams.PIXELTRAY_GATE2_OPEN: RobotParams.PIXELTRAY_GATE2_CLOSE,
            event, RobotParams.PIXELTRAY_OPEN_CLOSE_TIME);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "Gate 2 set to opened = %s", opened);
        }
    }   //setGate2Opened

    /**
     * This method returns the state of gate 2.
     *
     * @return true if gate is opened, false if closed.
     */
    public boolean isGate2Opened()
    {
        return gate2Opened;
    }   //isGate2Opened

}   //class PixelTray
