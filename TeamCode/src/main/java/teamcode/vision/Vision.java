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

package teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
import java.util.Arrays;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvColorBlobProcessor;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRawEocvColorBlobPipeline;
import TrcFtcLib.ftclib.FtcRawEocvVision;
import TrcFtcLib.ftclib.FtcVision;
import TrcFtcLib.ftclib.FtcVisionAprilTag;
import TrcFtcLib.ftclib.FtcVisionEocvColorBlob;
import TrcFtcLib.ftclib.FtcVisionTensorFlow;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.BlinkinLEDs;

/**
 * This class implements AprilTag/TensorFlow/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    private static final String moduleName = "Vision";

    public enum PixelType
    {
        PurplePixel,
        GreenPixel,
        YellowPixel,
        WhitePixel,
        AnyPixel
    }   //enum PixelType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
    // YCrCb Color Space.
    private static final int colorConversion = Imgproc.COLOR_RGB2YCrCb;
    private static final double[] purplePixelColorThresholds = {60.0, 250.0, 120.0, 150.0, 140.0, 170.0};
    private static final double[] greenPixelColorThresholds = {40.0, 200.0, 60.0, 120.0, 60.0, 120.0};
    private static final double[] yellowPixelColorThresholds = {150.0, 250.0, 110.0, 160.0, 20.0, 100.0};
    private static final double[] whitePixelColorThresholds = {250.0, 255.0, 100.0, 130.0, 120.0, 140.0};
    private static final double[] redConeColorThresholds = {20.0, 120.0, 180.0, 240.0, 90.0, 120.0};
    private static final double[] blueConeColorThresholds = {40.0, 140.0, 100.0, 150.0, 150.0, 200.0};
//    // HSV Color Space.
//    private static final int colorConversion = Imgproc.COLOR_RGB2HSV_FULL;
//    private static final double[] purplePixelColorThresholds = {170.0, 200.0, 40.0, 160.0, 100.0, 255.0};
//    private static final double[] greenPixelColorThresholds = {60.0, 120.0, 60.0, 255.0, 60.0, 255.0};
//    private static final double[] yellowPixelColorThresholds = {30.0, 60.0, 120.0, 225.0, 200.0, 255.0};
//    private static final double[] whitePixelColorThresholds = {70.0, 120.0, 0.0, 255.0, 230.0, 255.0};
//    private static final double[] redConeColorThresholds = {0.0, 10.0, 120.0, 255.0, 100.0, 255.0};
//    private static final double[] blueConeColorThresholds = {160.0, 200.0, 120.0, 255.0, 100.0, 255.0};
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams pixelFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(1000.0)
            .setMinPerimeter(120.0)
            .setWidthRange(50.0, 1000.0)
            .setHeightRange(10.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.2, 5.0);
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams coneFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(5000.0)
            .setMinPerimeter(200.0)
            .setWidthRange(50.0, 1000.0)
            .setHeightRange(80.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.3, 1.0);
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final String TFOD_MODEL_FILENAME = "TrcCenterStage.tflite";
    private static final float TFOD_MIN_CONFIDENCE = 0.90f;
    public static final String TFOD_PIXEL_LABEL = "Pixel";
    public static final String[] TFOD_FIRST_LABELS = {TFOD_PIXEL_LABEL};
    public static final String[] TFOD_TRC_LABELS = {"Yellow Pixel", "Purple Pixel", "White Pixel", "Green Pixel"};

    private final Robot robot;
    private final WebcamName webcam1, webcam2;
    private FtcRawEocvColorBlobPipeline rawColorBlobPipeline;
    public FtcRawEocvVision rawColorBlobVision;
    public FtcVisionAprilTag aprilTagVision;
    private AprilTagProcessor aprilTagProcessor;
    public FtcVisionEocvColorBlob purplePixelVision;
    private FtcEocvColorBlobProcessor purplePixelProcessor;
    public FtcVisionEocvColorBlob greenPixelVision;
    private FtcEocvColorBlobProcessor greenPixelProcessor;
    public FtcVisionEocvColorBlob yellowPixelVision;
    private FtcEocvColorBlobProcessor yellowPixelProcessor;
    public FtcVisionEocvColorBlob whitePixelVision;
    private FtcEocvColorBlobProcessor whitePixelProcessor;
    public FtcVisionEocvColorBlob redConeVision;
    private FtcEocvColorBlobProcessor redConeProcessor;
    public FtcVisionEocvColorBlob blueConeVision;
    private FtcEocvColorBlobProcessor blueConeProcessor;
    public FtcVisionTensorFlow tensorFlowVision;
    private TfodProcessor tensorFlowProcessor;
    public FtcVision vision;
    private int lastTeamPropPos = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public Vision(Robot robot, TrcDbgTrace tracer)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();

        this.robot = robot;
        this.webcam1 = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM1);
        this.webcam2 = RobotParams.Preferences.hasWebCam2?
            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM2): null;
        if (RobotParams.Preferences.tuneColorBlobVision)
        {
            OpenCvCamera openCvCam;

            if (RobotParams.Preferences.showVisionView)
            {
                int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
                openCvCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraViewId);
                openCvCam.showFpsMeterOnViewport(false);
            }
            else
            {
                openCvCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
            }

            robot.globalTracer.traceInfo(moduleName, "Starting RawEocvColorBlobVision...");
            rawColorBlobPipeline = new FtcRawEocvColorBlobPipeline(
                "rawColorBlobPipeline", colorConversion, whitePixelColorThresholds, pixelFilterContourParams, true,
                tracer);
            // By default, display original Mat.
            rawColorBlobPipeline.setVideoOutput(0);
            rawColorBlobPipeline.setAnnotateEnabled(true);
            rawColorBlobVision = new FtcRawEocvVision(
                "rawColorBlobVision", RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT, null, null,
                openCvCam, RobotParams.CAM_ORIENTATION, tracer);
            setRawColorBlobVisionEnabled(false);
        }
        else
        {
            // Creating Vision Processors for VisionPortal.
            ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

            if (RobotParams.Preferences.useAprilTagVision)
            {
                robot.globalTracer.traceInfo(moduleName, "Starting AprilTagVision...");
                FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                    .setDrawTagIdEnabled(true)
                    .setDrawTagOutlineEnabled(true)
                    .setDrawAxesEnabled(false)
                    .setDrawCubeProjectionEnabled(false)
//                    .setLensIntrinsics(
//                        RobotParams.WEBCAM_FX, RobotParams.WEBCAM_FY, RobotParams.WEBCAM_CX, RobotParams.WEBCAM_CY)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
                aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11, tracer);
                aprilTagProcessor = aprilTagVision.getVisionProcessor();
                visionProcessorsList.add(aprilTagProcessor);
            }

            if (RobotParams.Preferences.useColorBlobVision)
            {
                robot.globalTracer.traceInfo(moduleName, "Starting ColorBlobVision...");

                purplePixelVision = new FtcVisionEocvColorBlob(
                    "PurplePixel", colorConversion, purplePixelColorThresholds, pixelFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                purplePixelProcessor = purplePixelVision.getVisionProcessor();
                visionProcessorsList.add(purplePixelProcessor);

                greenPixelVision = new FtcVisionEocvColorBlob(
                    "GreenPixel", colorConversion, greenPixelColorThresholds, pixelFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                greenPixelProcessor = greenPixelVision.getVisionProcessor();
                visionProcessorsList.add(greenPixelProcessor);

                yellowPixelVision = new FtcVisionEocvColorBlob(
                    "YellowPixel", colorConversion, yellowPixelColorThresholds, pixelFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                yellowPixelProcessor = yellowPixelVision.getVisionProcessor();
                visionProcessorsList.add(yellowPixelProcessor);

                whitePixelVision = new FtcVisionEocvColorBlob(
                    "WhitePixel", colorConversion, whitePixelColorThresholds, pixelFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                whitePixelProcessor = whitePixelVision.getVisionProcessor();
                visionProcessorsList.add(whitePixelProcessor);

                redConeVision = new FtcVisionEocvColorBlob(
                    "RedCone", colorConversion, redConeColorThresholds, coneFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                redConeProcessor = redConeVision.getVisionProcessor();
                visionProcessorsList.add(redConeProcessor);

                blueConeVision = new FtcVisionEocvColorBlob(
                    "BlueCone", colorConversion, blueConeColorThresholds, coneFilterContourParams, true,
                    RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
                blueConeProcessor = blueConeVision.getVisionProcessor();
                visionProcessorsList.add(blueConeProcessor);
            }

            if (RobotParams.Preferences.useTensorFlowVision)
            {
                robot.globalTracer.traceInfo(moduleName, "Starting TensorFlowVision...");
                String model = RobotParams.Preferences.useTfodModelAsset? TFOD_MODEL_ASSET: TFOD_MODEL_FILENAME;
                String[] labels = RobotParams.Preferences.useTfodModelAsset? TFOD_FIRST_LABELS: TFOD_TRC_LABELS;
                tensorFlowVision = new FtcVisionTensorFlow(
                    null, RobotParams.Preferences.useTfodModelAsset, model, labels, RobotParams.cameraRect,
                    RobotParams.worldRect, tracer);
                tensorFlowProcessor = tensorFlowVision.getVisionProcessor();
                tensorFlowProcessor.setMinResultConfidence(TFOD_MIN_CONFIDENCE);
                visionProcessorsList.add(tensorFlowProcessor);
            }

            VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
            visionProcessorsList.toArray(visionProcessors);
            if (RobotParams.Preferences.useWebCam)
            {
                // Use USB webcams.
                vision = new FtcVision(
                    webcam1, webcam2, RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                    RobotParams.Preferences.showVisionView, visionProcessors);
            }
            else
            {
                // Use phone camera.
                vision = new FtcVision(
                    RobotParams.Preferences.useBuiltinCamBack?
                        BuiltinCameraDirection.BACK: BuiltinCameraDirection.FRONT,
                    RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                    RobotParams.Preferences.showVisionView, visionProcessors);
            }
            // Disable all vision until they are needed.
            setRawColorBlobVisionEnabled(false);
            for (VisionProcessor processor: visionProcessors)
            {
                vision.setProcessorEnabled(processor, false);
            }
        }
    }   //Vision

    /**
     * This method returns the active camera if we have two webcams.
     *
     * @return active camera.
     */
    public WebcamName getActiveCamera()
    {
        return vision.getActiveCamera();
    }   //getActiveCamera

    /**
     * This method switch the active camera to the other camera if we have two webcams.
     */
    public void switchCamera()
    {
        WebcamName activeCamera = getActiveCamera();

        if (activeCamera == webcam1 && webcam2 != null)
        {
            vision.setActiveCamera(webcam2);
        }
        else if (activeCamera == webcam2 && webcam1 != null)
        {
            vision.setActiveCamera(webcam1);
        }
    }   //switchCamera

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     *
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = vision.getExposureSetting();
        long currExposure = vision.getCurrentExposure();
        int[] gainSetting = vision.getGainSetting();
        int currGain = vision.getCurrentGain();

        robot.dashboard.displayPrintf(
            lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
            currExposure, exposureSetting[0], exposureSetting[1],
            currGain, gainSetting != null? gainSetting[0]: 0, gainSetting != null? gainSetting[1]: 0);
    }   //displayExposureSettings

    /**
     * This method returns the color threshold values of rawColorBlobVision.
     *
     * @return array of color threshold values.
     */
    public double[] getRawColorBlobThresholds()
    {
        return rawColorBlobPipeline != null? rawColorBlobPipeline.getColorThresholds(): null;
    }   //getRawColorBlobThresholds

    /**
     * This method sets the color threshold values of rawColorBlobVision.
     *
     * @param colorThresholds specifies an array of color threshold values.
     */
    public void setRawColorBlobThresholds(double... colorThresholds)
    {
        if (rawColorBlobPipeline != null)
        {
            rawColorBlobPipeline.setColorThresholds(colorThresholds);
        }
    }   //setRawColorBlobThresholds


    /**
     * This method enables/disables raw ColorBlob vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setRawColorBlobVisionEnabled(boolean enabled)
    {
        if (rawColorBlobVision != null)
        {
            rawColorBlobVision.setPipeline(enabled? rawColorBlobPipeline: null);
        }
    }   //setRawColorBlobVisionEnabled

    /**
     * This method checks if raw ColorBlob vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isRawColorBlobVisionEnabled()
    {
        return rawColorBlobVision != null && rawColorBlobVision.getPipeline() != null;
    }   //isRawColorBlobVisionEnabled

    /**
     * This method calls RawColorBlob vision to detect the color blob for color threshold tuning.
     *
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected raw color blob object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getDetectedRawColorBlob(int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> colorBlobInfo =
            rawColorBlobVision != null? rawColorBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0): null;

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(lineNum, "ColorBlob: %s", colorBlobInfo != null? colorBlobInfo: "Not found.");
        }

        return colorBlobInfo;
    }   //getDetectedRawColorBlob

    /**
     * This method enables/disables AprilTag vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setAprilTagVisionEnabled(boolean enabled)
    {
        if (aprilTagProcessor != null)
        {
            vision.setProcessorEnabled(aprilTagProcessor, enabled);
        }
    }   //setAprilTagVisionEnabled

    /**
     * This method checks if AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagVisionEnabled()
    {
        return aprilTagProcessor != null && vision.isVisionProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    /**
     * This method calls AprilTag vision to detect the AprilTag object.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getDetectedAprilTag(Integer id, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
            robot.vision.aprilTagVision.getBestDetectedTargetInfo(id, null);

        if (aprilTagInfo != null && robot.blinkin != null)
        {
            robot.blinkin.setDetectedPattern(BlinkinLEDs.APRIL_TAG);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(lineNum, "AprilTag: %s", aprilTagInfo != null? aprilTagInfo: "Not found.");
        }

        return aprilTagInfo;
    }   //getDetectedAprilTag

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo)
    {
        final String funcName = "getRobotFieldPose";
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null)
        {
            TrcPose2D aprilTagPose = RobotParams.APRILTAG_POSES[aprilTagInfo.detectedObj.aprilTagDetection.id - 1];
            robotPose = aprilTagPose.subtractRelativePose(aprilTagInfo.objPose.toPose2D());
            // TODO: adjust robot pose with respect to the camera location on the robot.
            robot.globalTracer.traceInfo(
                funcName, "AprilTagId=%d, aprilTagFieldPose=%s, aprilTagPoseFromCamera=%s, robotPose=%s",
                aprilTagInfo.detectedObj.aprilTagDetection.id, aprilTagPose, aprilTagInfo.objPose.toPose2D(),
                robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotPose = null;

        if (aprilTagVision != null)
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getDetectedAprilTag(null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo);
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method enables/disables vision for the specified pixel type.
     *
     * @param pixelType specifies the pixel type to be detected.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setPixelVisionEnabled(PixelType pixelType, boolean enabled)
    {
        switch (pixelType)
        {
            case PurplePixel:
                if (purplePixelProcessor != null)
                {
                    vision.setProcessorEnabled(purplePixelProcessor, enabled);
                }
                break;

            case GreenPixel:
                if (greenPixelProcessor != null)
                {
                    vision.setProcessorEnabled(greenPixelProcessor, enabled);
                }
                break;

            case YellowPixel:
                if (yellowPixelProcessor != null)
                {
                    vision.setProcessorEnabled(yellowPixelProcessor, enabled);
                }
                break;

            case WhitePixel:
                if (whitePixelProcessor != null)
                {
                    vision.setProcessorEnabled(whitePixelProcessor, enabled);
                }
                break;

            case AnyPixel:
                if (purplePixelProcessor != null)
                {
                    vision.setProcessorEnabled(purplePixelProcessor, enabled);
                }
                if (greenPixelProcessor != null)
                {
                    vision.setProcessorEnabled(greenPixelProcessor, enabled);
                }
                if (yellowPixelProcessor != null)
                {
                    vision.setProcessorEnabled(yellowPixelProcessor, enabled);
                }
                if (whitePixelProcessor != null)
                {
                    vision.setProcessorEnabled(whitePixelProcessor, enabled);
                }
                break;
        }
    }   //setPixelVisionEnabled

    /**
     * This method checks if vision is enabled for the specified pixel type.
     *
     * @param pixelType specifies the pixel type to be detected.
     * @return true if enabled, false if disabled.
     */
    public boolean isPixelVisionEnabled(PixelType pixelType)
    {
        boolean enabled = false;

        switch (pixelType)
        {
            case PurplePixel:
                enabled = purplePixelProcessor != null && vision.isVisionProcessorEnabled(purplePixelProcessor);
                break;

            case GreenPixel:
                enabled = greenPixelProcessor != null && vision.isVisionProcessorEnabled(greenPixelProcessor);
                break;

            case YellowPixel:
                enabled = yellowPixelProcessor != null && vision.isVisionProcessorEnabled(yellowPixelProcessor);
                break;

            case WhitePixel:
                enabled = whitePixelProcessor != null && vision.isVisionProcessorEnabled(whitePixelProcessor);
                break;

            case AnyPixel:
                enabled = purplePixelProcessor != null && vision.isVisionProcessorEnabled(purplePixelProcessor) ||
                          greenPixelProcessor != null && vision.isVisionProcessorEnabled(greenPixelProcessor) ||
                          yellowPixelProcessor != null && vision.isVisionProcessorEnabled(yellowPixelProcessor) ||
                          whitePixelProcessor != null && vision.isVisionProcessorEnabled(whitePixelProcessor);
                break;
        }

        return enabled;
    }   //isPixelVisionEnabled

    /**
     * This method calls ColorBlob vision to detect the specified Pixel object.
     *
     * @param pixelType specifies the pixel type to be detected.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Purple Pixel object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedPixel(
        PixelType pixelType, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo = null;
        String pixelName = null;

        switch (pixelType)
        {
            case PurplePixel:
                pixelInfo = purplePixelVision != null? purplePixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                pixelName = BlinkinLEDs.PURPLE_PIXEL;
                break;

            case GreenPixel:
                pixelInfo = greenPixelVision != null? greenPixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                pixelName = BlinkinLEDs.GREEN_PIXEL;
                break;

            case YellowPixel:
                pixelInfo = yellowPixelVision != null? yellowPixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                pixelName = BlinkinLEDs.YELLOW_PIXEL;
                break;

            case WhitePixel:
                pixelInfo = whitePixelVision != null? whitePixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                pixelName = BlinkinLEDs.WHITE_PIXEL;
                break;

            case AnyPixel:
                ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> pixelList = new ArrayList<>();

                pixelInfo = purplePixelVision != null? purplePixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                if (pixelInfo != null)
                {
                    pixelList.add(pixelInfo);
                }

                pixelInfo = greenPixelVision != null? greenPixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                if (pixelInfo != null)
                {
                    pixelList.add(pixelInfo);
                }

                pixelInfo = yellowPixelVision != null? yellowPixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                if (pixelInfo != null)
                {
                    pixelList.add(pixelInfo);
                }

                pixelInfo = whitePixelVision != null? whitePixelVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                if (pixelInfo != null)
                {
                    pixelList.add(pixelInfo);
                }

                TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[] pixels =
                    new TrcVisionTargetInfo[pixelList.size()];
                pixelList.toArray(pixels);
                if (pixels.length > 1)
                {
                    Arrays.sort(pixels, this::compareDistance);
                }
                pixelInfo = pixels[0];
                pixelName = pixelInfo.detectedObj.label;
                break;
        }

        if (pixelInfo != null && robot.blinkin != null)
        {
            robot.blinkin.setDetectedPattern(pixelName);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "%s: %s", pixelName, pixelInfo != null? pixelInfo: "Not found.");
        }

        return pixelInfo;
    }   //getDetectedPixel

    /**
     * This method enables/disables RedCone vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setRedConeVisionEnabled(boolean enabled)
    {
        if (redConeProcessor != null)
        {
            vision.setProcessorEnabled(redConeProcessor, enabled);
        }
    }   //setRedConeVisionEnabled

    /**
     * This method checks if RedCone vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isRedConeVisionEnabled()
    {
        return redConeProcessor != null && vision.isVisionProcessorEnabled(redConeProcessor);
    }   //isRedConeVisionEnabled

    /**
     * This method enables/disables BlueCone vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setBlueConeVisionEnabled(boolean enabled)
    {
        if (blueConeProcessor != null)
        {
            vision.setProcessorEnabled(blueConeProcessor, enabled);
        }
    }   //setBlueConeVisionEnabled

    /**
     * This method checks if BlueCone vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isBlueConeVisionEnabled()
    {
        return blueConeProcessor != null && vision.isVisionProcessorEnabled(blueConeProcessor);
    }   //isBlueConeVisionEnabled

    /**
     * This method detects the team prop and determine its position.
     *
     * @param alliance specifies the alliance color to look for team prop.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return team prop position 1, 2, or 3, 0 if not found.
     */
    public int getDetectedTeamPropPosition(FtcAuto.Alliance alliance, int lineNum)
    {
        int pos = 0;
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> teamPropInfo = null;

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            if (redConeVision != null)
            {
                teamPropInfo = redConeVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0);
            }
        }
        else
        {
            if (blueConeVision != null)
            {
                teamPropInfo = blueConeVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0);
            }
        }

        if (teamPropInfo != null)
        {
            double teamPropXPos = teamPropInfo.rect.x + teamPropInfo.rect.width/2.0;
            double oneThirdScreenWidth = RobotParams.CAM_IMAGE_WIDTH/3.0;
            String ledLabel = null;

            if (teamPropXPos < oneThirdScreenWidth)
            {
                pos = 1;
                ledLabel = alliance == FtcAuto.Alliance.RED_ALLIANCE?
                    BlinkinLEDs.RED_CONE_POS_1: BlinkinLEDs.BLUE_CONE_POS_1;
            }
            else if (teamPropXPos < oneThirdScreenWidth*2)
            {
                pos = 2;
                ledLabel = alliance == FtcAuto.Alliance.RED_ALLIANCE?
                    BlinkinLEDs.RED_CONE_POS_2: BlinkinLEDs.BLUE_CONE_POS_2;
            }
            else
            {
                pos = 3;
                ledLabel = alliance == FtcAuto.Alliance.RED_ALLIANCE?
                    BlinkinLEDs.RED_CONE_POS_3: BlinkinLEDs.BLUE_CONE_POS_3;
            }

            if (robot.blinkin != null)
            {
                robot.blinkin.setDetectedPattern(ledLabel);
            }
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "%s: %s (pos=%d)",
                alliance == FtcAuto.Alliance.RED_ALLIANCE? "RedCone": "BlueCone",
                teamPropInfo != null? teamPropInfo: "Not found", pos);
        }

        if (pos != 0)
        {
            lastTeamPropPos = pos;
        }

        return pos;
    }   //getDetectedTeamPropPosition

    /**
     * This method returns the last detected team prop position.
     *
     * @return last team prop position, 0 if never detected it.
     */
    public int getLastDetectedTeamPropPosition()
    {
        return lastTeamPropPos;
    }   //getLastDetectedTeamPropPosition

    /**
     * This method enables/disables TensorFlow vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setTensorFlowVisionEnabled(boolean enabled)
    {
        if (tensorFlowProcessor != null)
        {
            vision.setProcessorEnabled(tensorFlowProcessor, enabled);
        }
    }   //setTensorFlowVisionEnabled

    /**
     * This method checks if TensorFlow vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isTensorFlowVisionEnabled()
    {
        return tensorFlowProcessor != null && vision.isVisionProcessorEnabled(tensorFlowProcessor);
    }   //isTensorFlowVisionEnabled

    /**
     * This method calls TensorFlow vision to detect the Pixel objects.
     *
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Pixel object info.
     */
    public TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> getDetectedTensorFlowPixel(int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> tensorFlowInfo =
            robot.vision.tensorFlowVision.getBestDetectedTargetInfo(
                Vision.TFOD_PIXEL_LABEL, null, this::compareConfidence, 0.0, 0.0);

        if (tensorFlowInfo != null && robot.blinkin != null)
        {
            robot.blinkin.setDetectedPattern(BlinkinLEDs.TENSOR_FLOW);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "TensorFlow: %s", tensorFlowInfo != null? tensorFlowInfo: "Not found.");
        }

        return tensorFlowInfo;
    }   //getDetectedTensorFlowPixel

    /**
     * This method validates the detected pixel is really a pixel by checking its physical width to be about 3 inches.
     *
     * @param pixelInfo specifies the detected pixel info object.
     * @return true if it passes the test, false otherwise.
     */
    public boolean validatePixel(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo)
    {
        // Pixel is 3-inch wide.
        return pixelInfo.objWidth > 2.0 && pixelInfo.objWidth < 4.0;
    }   //validatePixel

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    private int compareDistance(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((b.objPose.y - a.objPose.y)*100);
    }   //compareDistance

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> a,
        TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

}   //class Vision
