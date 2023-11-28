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
    private final FtcServo lowerGate, upperGate;
    private boolean lowerGateOpened, upperGateOpened;

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
        lowerGate = new FtcServo(instanceName + ".lowerGate");
        upperGate = new FtcServo(instanceName + ".upperGate");
    }   //PixelTray

    /**
     * This method returns the state info of the Pixel Tray in string format.
     *
     * @return state info of the Pixel Tray.
     */
    @Override
    public String toString()
    {
        return instanceName + ": lowerGate=" + lowerGateOpened + ", upperGate=" + upperGateOpened;
    }   //toString

    /**
     * This method opens and closes the pixel lower gate.
     *
     * @param opened specifies true to open the gate, false to close.
     * @param event specifies the event to signal after open/close wait time expired.
     */
    public void setLowerGateOpened(boolean opened, TrcEvent event)
    {
        lowerGateOpened = opened;
        lowerGate.setPosition(
            opened? RobotParams.PIXELTRAY_LOWER_GATE_OPEN: RobotParams.PIXELTRAY_LOWER_GATE_CLOSE,
            event, RobotParams.PIXELTRAY_OPEN_CLOSE_TIME);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(instanceName, "lowerGateOpened = %s", opened);
        }
    }   //setLowerGateOpened

    /**
     * This method returns the state of the lower gate.
     *
     * @return true if gate is opened, false if closed.
     */
    public boolean isLowerGateOpened()
    {
        return lowerGateOpened;
    }   //isLowerGateOpened

    /**
     * This method opens and closes the pixel upper gate.
     *
     * @param opened specifies true to open the gate, false to close.
     * @param event specifies the event to signal after open/close wait time expired.
     */
    public void setUpperGateOpened(boolean opened, TrcEvent event)
    {
        upperGateOpened = opened;
        upperGate.setPosition(
            opened? RobotParams.PIXELTRAY_UPPER_GATE_OPEN: RobotParams.PIXELTRAY_UPPER_GATE_CLOSE,
            event, RobotParams.PIXELTRAY_OPEN_CLOSE_TIME);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(instanceName, "upperGateOpened = %s", opened);
        }
    }   //setUpperGateOpened

    /**
     * This method returns the state of the upper gate.
     *
     * @return true if gate is opened, false if closed.
     */
    public boolean isUpperGateOpened()
    {
        return upperGateOpened;
    }   //isUpperGateOpened

}   //class PixelTray
