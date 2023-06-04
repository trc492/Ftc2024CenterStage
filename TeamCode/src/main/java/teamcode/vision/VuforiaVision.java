/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcVuforia;
import teamcode.RobotParams;
import teamcode.subsysstems.BlinkinLEDs;

/**
 * This class implements Vuforia Vision that provides the capability of using images to locate the robot position
 * on the field.
 */
public class VuforiaVision
{
    private static final String TRACKABLE_IMAGES_FILE = "PowerPlay";
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical
    // dimension. We will define some constants and conversions here.
    //
    // Height of the center of the target image above the floor.
    private static final float mmTargetHeight = 6.0f * (float)TrcUtil.MM_PER_INCH;
    private static final float halfField = (float)(RobotParams.HALF_FIELD_INCHES*TrcUtil.MM_PER_INCH);
    private static final float fullTile = (float)(RobotParams.FULL_TILE_INCHES*TrcUtil.MM_PER_INCH);
    private static final float oneAndHalfTile = (float)(fullTile*1.5);

    private final FtcVuforia vuforia;
    private final FtcRevBlinkin blinkin;
    private final VuforiaTrackable[] vuforiaImageTargets;
    private String lastVuforiaImageName;

    /**
     * Constructor: Create an instance of the object.
     */
    public VuforiaVision(FtcVuforia vuforia, FtcRevBlinkin blinkin)
    {
        this.vuforia = vuforia;
        this.blinkin = blinkin;
        vuforia.configVideoSource(
            RobotParams.WEBCAM_IMAGE_WIDTH, RobotParams.WEBCAM_IMAGE_HEIGHT, RobotParams.FRAME_QUEUE_CAPACITY);
        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along
         * the Y axis. Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z
         * direction), with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out
         *    the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         */
        final float CAMERA_FORWARD_DISPLACEMENT =
            (float)((RobotParams.ROBOT_LENGTH/2.0 - RobotParams.WEBCAM_FRONT_OFFSET)*TrcUtil.MM_PER_INCH);
        final float CAMERA_VERTICAL_DISPLACEMENT =
            (float)(RobotParams.WEBCAM_HEIGHT_OFFSET*TrcUtil.MM_PER_INCH);
        final float CAMERA_LEFT_DISPLACEMENT =
            (float)(-(RobotParams.ROBOT_WIDTH/2.0 - RobotParams.WEBCAM_LEFT_OFFSET)*TrcUtil.MM_PER_INCH);
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(
                EXTRINSIC, XZY, DEGREES, (float)(90 + RobotParams.WEBCAM_TILT_DOWN), 90, 0));
        /*
         * In order for localization to work, we need to tell the system where each target is on the field,
         * and where the camera resides on the robot.  These specifications are in the form of
         * <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance
         *       station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */
        //
        // Create and initialize all image targets.
        //
        FtcVuforia.TargetInfo[] imageTargetsInfo = {
            new FtcVuforia.TargetInfo(
                0, BlinkinLEDs.IMAGE1_NAME, false,
                createImageLocation(-halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90)),
            new FtcVuforia.TargetInfo(
                1, BlinkinLEDs.IMAGE2_NAME, false,
                createImageLocation(halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90)),
            new FtcVuforia.TargetInfo(
                2, BlinkinLEDs.IMAGE3_NAME, false,
                createImageLocation(-halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90)),
            new FtcVuforia.TargetInfo(
                3, BlinkinLEDs.IMAGE4_NAME, false,
                createImageLocation(halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90))
        };
        vuforia.addTargetList(TRACKABLE_IMAGES_FILE, imageTargetsInfo, cameraLocationOnRobot);

        vuforiaImageTargets = new VuforiaTrackable[imageTargetsInfo.length];
        for (int i = 0; i < vuforiaImageTargets.length; i++)
        {
            vuforiaImageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
        }
    }   //VuforiaVision

    /**
     * This method enables/disables Vuforia Vision.
     *
     * @param enabled specifies true to enable Vuforia Vision, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    /**
     * This method checks if Vuforia is enabled.
     *
     * @return true if Vuforia is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return vuforia.isTrackingEnabled();
    }   //isEnabled

    /**
     * This method creates an OpenGLMatrix as the image location.
     *
     * @param dx specifies translation coordinate in X.
     * @param dy specifies translation coordinate in Y.
     * @param dz specifies translation coordinate in Z.
     * @param rx specifies rotation in the x-axis.
     * @param ry specifies rotation in the y-axis.
     * @param rz specifies rotation in the z-axis.
     * @return generated image location.
     */
    private OpenGLMatrix createImageLocation(float dx, float dy, float dz, float rx, float ry, float rz)
    {
        return OpenGLMatrix.translation(dx, dy, dz)
                           .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz));
    }   //createImageLocation

    /**
     * This method returns the robot location computed with the detected target.
     *
     * @param targetName specifies the detected target name.
     * @return robot location.
     */
    private OpenGLMatrix getRobotLocation(String targetName)
    {
        OpenGLMatrix robotLocation = null;
        VuforiaTrackable target = vuforia.getTarget(targetName);

        if (target != null)
        {
            robotLocation = vuforia.getRobotLocation(target);

            if (blinkin != null)
            {
                if (robotLocation != null)
                {
                    blinkin.setPatternState(targetName, true, 1.0);
                }
            }
        }

        return robotLocation;
    }   //getRobotLocation

    /**
     * This method returns the robot location computed with the detected target.
     *
     * @param targetName specifies the detected target name.
     * @param exclude specifies true to exclude the specified target.
     * @return robot location.
     */
    private OpenGLMatrix getRobotLocation(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = null;

        if (targetName == null || exclude)
        {
            for (VuforiaTrackable target: vuforiaImageTargets)
            {
                String name = target.getName();
                boolean isMatched = targetName == null || !targetName.equals(name);

                if (isMatched && vuforia.isTargetVisible(target))
                {
                    // getRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix location = vuforia.getRobotLocation(target);
                    if (location != null)
                    {
                        robotLocation = location;
                        lastVuforiaImageName = name;
                    }
                    break;
                }
            }
        }
        else
        {
            robotLocation = getRobotLocation(targetName);
            if (robotLocation != null)
            {
                lastVuforiaImageName = targetName;
            }
        }

        if (blinkin != null)
        {
            if (robotLocation != null)
            {
                blinkin.setPatternState(lastVuforiaImageName, true);
            }
            else
            {
                blinkin.reset();
            }
        }

        return robotLocation;
    }   //getRobotLocation

    /**
     * This method returns the robot field position.
     *
     * @param targetName specifies the detected target name.
     * @param exclude specifies true to exclude the specified target.
     * @return robot field position.
     */
    public TrcPose2D getRobotPose(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = getRobotLocation(targetName, exclude);
        VectorF translation = robotLocation == null? null: robotLocation.getTranslation();
        Orientation orientation = robotLocation == null?
            null: Orientation.getOrientation(robotLocation, EXTRINSIC, XYZ, DEGREES);
        //
        // The returned RobotPose have the X axis pointing from the audience side to the back of the field,
        // the Y axis pointing from the red alliance to the blue alliance and the direction of the Y axis
        // is zero degree and increases in the clockwise direction.
        // Vuforia's orientation is such that facing the negative side of the X axis is 0 degree, clockwise gives
        // you negative angle and anti-clockwise gives you positive angle.
        // Robot's orientation is such that facing the positive side of the Y axis is 0 degree, clockwise gives
        // you positive angle and anti-clockwise gives you negative angle.
        // In order to translate Vuforia's orientation to the robot's orientation, we first negate the vuforia
        // angle to make rotation agree with the robot's rotation. Then we subtract 90 degrees from the angle.
        //
        return (translation == null || orientation == null)? null:
            new TrcPose2D(translation.get(0)/TrcUtil.MM_PER_INCH, translation.get(1)/TrcUtil.MM_PER_INCH,
                          -orientation.thirdAngle - 90.0);
    }   //getRobotPose

    /**
     * This method returns the name of the image last seen.
     *
     * @return last seen image name.
     */
    public String getLastSeenImageName()
    {
        return lastVuforiaImageName;
    }   //getLastSeenImageName

}   //class VuforiaVision
