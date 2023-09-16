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

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcFtcLib.ftclib.FtcRevBlinkin;

/**
 * This class encapsulates the REV Blinkin LED controller to provide a priority indicator showing the status of the
 * robot.
 */
public class BlinkinLEDs extends FtcRevBlinkin
{
    // LED pattern names.
    public static final String PURPLE_PIXEL = "PurplePixel";
    public static final String GREEN_PIXEL = "GreenPixel";
    public static final String YELLOW_PIXEL = "YellowPixel";
    public static final String WHITE_PIXEL = "WhitePixel";
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";
    public static final String OFF_PATTERN = "Off";

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name. This is also the REV Blinkin's hardware name.
     */
    public BlinkinLEDs(String instanceName)
    {
        super(instanceName);
        // LED Patterns are sorted in decreasing priority order.
        final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
            // Highest priority.
            new TrcRevBlinkin.Pattern(PURPLE_PIXEL, TrcRevBlinkin.RevLedPattern.SolidViolet),
            new TrcRevBlinkin.Pattern(GREEN_PIXEL, TrcRevBlinkin.RevLedPattern.SolidGreen),
            new TrcRevBlinkin.Pattern(YELLOW_PIXEL, TrcRevBlinkin.RevLedPattern.SolidYellow),
            new TrcRevBlinkin.Pattern(WHITE_PIXEL, TrcRevBlinkin.RevLedPattern.SolidWhite),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidAqua),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidRed),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidBlue),
            new TrcRevBlinkin.Pattern(OFF_PATTERN, RevLedPattern.SolidBlack)
            // Lowest priority.
        };
        setPatternPriorities(ledPatternPriorities);
    }   //BlinkinLEDs

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                setPatternState(DRIVE_ORIENTATION_INVERTED, true);
                setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                setPatternState(DRIVE_ORIENTATION_FIELD, false);
                break;

            case ROBOT:
                setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                setPatternState(DRIVE_ORIENTATION_ROBOT, true);
                setPatternState(DRIVE_ORIENTATION_FIELD, false);
                break;

            case FIELD:
                setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                setPatternState(DRIVE_ORIENTATION_FIELD, true);
                break;
        }
    }   //setDriveOrientation

    public void setVisionDetectedObject(String objLabel)
    {
        setPatternState(PURPLE_PIXEL, false);
        setPatternState(GREEN_PIXEL, false);
        setPatternState(YELLOW_PIXEL, false);
        setPatternState(WHITE_PIXEL, false);
        setPatternState(objLabel, true);
    }   //setVisionDetectedObject

}   //class BlinkinLEDs
