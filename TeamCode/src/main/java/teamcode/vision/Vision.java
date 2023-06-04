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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvAprilTagPipeline;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsysstems.BlinkinLEDs;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";
    public static final String[] TARGET_LABELS = {
        BlinkinLEDs.LABEL_BOLT, BlinkinLEDs.LABEL_BULB, BlinkinLEDs.LABEL_PANEL
    };

    private final Robot robot;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision eocvVision;

    private int lastSignal = 0;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        if (RobotParams.Preferences.useEasyOpenCV)
        {
            OpenCvWebcam webcam;

            if (RobotParams.Preferences.showEasyOpenCvView)
            {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM), cameraViewId);
                webcam.showFpsMeterOnViewport(false);
            }
            else
            {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM));
            }
            // EOCV sometimes timed out on opening the camera. The default timeout was 2 seconds. It seems setting
            // it to 5 seconds would do wonder here.
            webcam.setMillisecondsPermissionTimeout(RobotParams.WEBCAM_PERMISSION_TIMEOUT);

            robot.globalTracer.traceInfo("Vision", "Starting EocvVision...");
            eocvVision = new EocvVision(
                "eocvVision", RobotParams.WEBCAM_IMAGE_WIDTH, RobotParams.WEBCAM_IMAGE_HEIGHT,
                RobotParams.cameraRect, RobotParams.worldRect, webcam, OpenCvCameraRotation.UPRIGHT, null);
        }
        else if (RobotParams.Preferences.useVuforia || RobotParams.Preferences.useTensorFlow)
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        if (tensorFlowVision != null)
        {
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
    }   //tensorFlowShutdown

    /**
     * This method updates the LED state to show the vision detected object.
     *
     * @param label specifies the detected object.
     */
    private void updateVisionLEDs(String label)
    {
        if (label != null && robot.blinkin != null)
        {
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BOLT, false);
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BULB, false);
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_PANEL, false);
            robot.blinkin.setPatternState(label, true, 1.0);
        }
    }   //updateVisionLEDs

    /**
     * This method calls vision to detect the signal and returns the detected info.
     *
     * @return detected signal info, null if none detected.
     */
    public TrcVisionTargetInfo<?> getDetectedSignalInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;
        String label = null;

        if (tensorFlowVision != null && tensorFlowVision.isEnabled())
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                null, null, this::compareConfidence,
                RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.WEBCAM_HEIGHT_OFFSET);
            if (targets != null)
            {
                label = ((TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>) targets[0]).detectedObj.label;
            }
        }
        else if (eocvVision != null && eocvVision.getPipeline() != null &&
                 eocvVision.getDetectObjectType() == EocvVision.ObjectType.APRIL_TAG)
        {
            targets = eocvVision.getDetectedTargetsInfo(
                null, null, RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.WEBCAM_HEIGHT_OFFSET);
            if (targets != null)
            {
                label = TARGET_LABELS[
                    ((FtcEocvAprilTagPipeline.DetectedObject) targets[0].detectedObj).object.id - 1];
            }
        }
        updateVisionLEDs(label);

        return targets != null? targets[0]: null;
    }   //getDetectedSignalInfo

    /**
     * This method determines the signal value from the detected object info.
     *
     * @return signal value of the detected object, 0 if no detected object.
     */
    public int determineDetectedSignal(TrcVisionTargetInfo<?> target)
    {
        int detectedSignal = 0;

        if (target != null)
        {
            if (tensorFlowVision != null)
            {
                FtcTensorFlow.DetectedObject detectedObj = (FtcTensorFlow.DetectedObject) target.detectedObj;

                switch (detectedObj.label)
                {
                    case BlinkinLEDs.LABEL_BOLT:
                        detectedSignal = 1;
                        break;

                    case BlinkinLEDs.LABEL_BULB:
                        detectedSignal = 2;
                        break;

                    case BlinkinLEDs.LABEL_PANEL:
                        detectedSignal = 3;
                        break;
                }
            }
            else if (eocvVision != null && eocvVision.getPipeline() instanceof FtcEocvAprilTagPipeline)
            {
                FtcEocvAprilTagPipeline.DetectedObject detectedObj =
                    (FtcEocvAprilTagPipeline.DetectedObject) target.detectedObj;

                detectedSignal = detectedObj.object.id;
            }
        }

        if (detectedSignal != 0)
        {
            lastSignal = detectedSignal;
            if (robot.blinkin != null)
            {
                // Turn off previous detection indication.
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BOLT, false);
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BULB, false);
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_PANEL, false);

                switch (detectedSignal)
                {
                    case 1:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BOLT, true, 1.0);
                        break;

                    case 2:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BULB, true, 1.0);
                        break;

                    case 3:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_PANEL, true, 1.0);
                        break;
                }
                robot.dashboard.displayPrintf(15, "Found the signal at %d", detectedSignal);
            }
        }

        return detectedSignal;
    }   //determineDetectedSignal

    /**
     * This method calls the appropriate vision detection to detect the signal position.
     *
     * @return detected signal position, 0 if none detected.
     */
    public int getDetectedSignal()
    {
        return determineDetectedSignal(getDetectedSignalInfo());
    }   //getDetectedSignal

    /**
     * This method does not initiate detection. It simply returns the signal detected by the last call to
     * getDetectedSignal. This is typically used to get the last detected signal during the init period.
     *
     * @return last detected signal.
     */
    public int getLastSignal()
    {
        return lastSignal;
    }   //getLastSignal

    /**
     * This method calls vision to detect the cone and returns the detected info.
     *
     * @return detected cone info, null if none detected.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedConeInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (eocvVision != null && eocvVision.getPipeline() != null)
        {
            EocvVision.ObjectType detectObjType = eocvVision.getDetectObjectType();

            if (detectObjType == EocvVision.ObjectType.RED_CONE || detectObjType == EocvVision.ObjectType.BLUE_CONE)
            {
                targets = eocvVision.getDetectedTargetsInfo(null, this::compareBottomY, 0.0, 0.0);

                if (targets != null && robot.blinkin != null)
                {
                    robot.blinkin.setPatternState(
                        detectObjType == EocvVision.ObjectType.RED_CONE?
                            BlinkinLEDs.GOT_RED_CONE: BlinkinLEDs.GOT_BLUE_CONE,
                        true, 1.0);
                }
            }
        }

        return targets != null? (TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>) targets[0]: null;
    }   //getDetectedConeInfo

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing bottom Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller bottom Y than b, 0 if a and b have equal bottom Y,
     *         positive value if a has larger bottom Y than b.
     */
    private int compareBottomY(
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> a,
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> b)
    {
        Rect aRect = a.detectedObj.getRect();
        Rect bRect = b.detectedObj.getRect();

        return (bRect.y + bRect.height) - (aRect.y + aRect.height);
    }   //compareBottomY

}   //class Vision
