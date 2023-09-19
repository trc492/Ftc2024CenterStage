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

import java.util.ArrayList;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvColorBlobProcessor;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcVision;
import TrcFtcLib.ftclib.FtcVisionAprilTag;
import TrcFtcLib.ftclib.FtcVisionEocvColorBlob;
import TrcFtcLib.ftclib.FtcVisionTensorFlow;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class implements AprilTag/TensorFlow/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    private static final String moduleName = "Vision";
    private static final int colorConversion = Imgproc.COLOR_BGRA2BGR;
    private static final double[] purplePixelColorThresholds = {120.0, 255.0, 0.0, 200.0, 200.0, 255.0};
    private static final double[] greenPixelColorThresholds = {0.0, 100.0, 120.0, 255.0, 0.0, 140.0};
    private static final double[] yellowPixelColorThresholds = {120.0, 255.0, 100.0, 225.0, 0.0, 60.0};
    private static final double[] whitePixelColorThresholds = {160.0, 255.0, 175.0, 255.0, 150.0, 225.0};
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams pixelFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(5000.0)
            .setMinPerimeter(200.0)
            .setWidthRange(50.0, 1000.0)
            .setHeightRange(50.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(1.0, 10.0);
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final float TFOD_MIN_CONFIDENCE = 0.90f;
    public static final String[] TFOD_TARGET_LABELS = {"Pixel"};

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
    public FtcVisionTensorFlow tensorFlowVision;
    private TfodProcessor tensorFlowProcessor;
    private final FtcVision vision;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public Vision(Robot robot, TrcDbgTrace tracer)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        ArrayList<VisionProcessor> visionProcessorList = new ArrayList<>();

        if (RobotParams.Preferences.useAprilTagVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting AprilTagVision...");
            FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                .setDrawTagIdEnabled(true)
                .setDrawTagOutlineEnabled(true)
                .setDrawAxesEnabled(false)
                .setDrawCubeProjectionEnabled(false)
                .setLensIntrinsics(
                    RobotParams.WEBCAM_FX, RobotParams.WEBCAM_FY, RobotParams.WEBCAM_CX, RobotParams.WEBCAM_CY)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
            aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11, tracer);
            aprilTagProcessor = aprilTagVision.getVisionProcessor();
            visionProcessorList.add(aprilTagProcessor);
        }

        if (RobotParams.Preferences.useColorBlobVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting ColorBlobVision...");

            purplePixelVision = new FtcVisionEocvColorBlob(
                "PurplePixel", colorConversion, purplePixelColorThresholds, pixelFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            purplePixelProcessor = purplePixelVision.getVisionProcessor();
            visionProcessorList.add(purplePixelProcessor);

            greenPixelVision = new FtcVisionEocvColorBlob(
                "GreenPixel", colorConversion, greenPixelColorThresholds, pixelFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            greenPixelProcessor = greenPixelVision.getVisionProcessor();
            visionProcessorList.add(greenPixelProcessor);

            yellowPixelVision = new FtcVisionEocvColorBlob(
                "YellowPixel", colorConversion, yellowPixelColorThresholds, pixelFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            yellowPixelProcessor = yellowPixelVision.getVisionProcessor();
            visionProcessorList.add(yellowPixelProcessor);

            whitePixelVision = new FtcVisionEocvColorBlob(
                "WhitePixel", colorConversion, whitePixelColorThresholds, pixelFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            whitePixelProcessor = whitePixelVision.getVisionProcessor();
            visionProcessorList.add(whitePixelProcessor);
        }

        if (RobotParams.Preferences.useTensorFlowVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting TensorFlowVision...");
            tensorFlowVision = new FtcVisionTensorFlow(
                null, TFOD_MODEL_ASSET, TFOD_TARGET_LABELS, RobotParams.cameraRect, RobotParams.worldRect, tracer);
            tensorFlowProcessor = tensorFlowVision.getVisionProcessor();
            tensorFlowProcessor.setMinResultConfidence(TFOD_MIN_CONFIDENCE);
            visionProcessorList.add(tensorFlowProcessor);
        }

        VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorList.size()];
        visionProcessorList.toArray(visionProcessors);
        vision = RobotParams.Preferences.useWebCam ?
                    new FtcVision(
                        opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM),
                        RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                        RobotParams.Preferences.showVisionView, visionProcessors) :
                    new FtcVision(
                        RobotParams.Preferences.useBuiltinCamBack ?
                            BuiltinCameraDirection.BACK : BuiltinCameraDirection.FRONT,
                        RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                        RobotParams.Preferences.showVisionView, visionProcessors);
        // Disable all vision processors until they are needed.
        setAprilTagVisionEnabled(false);
        setPurplePixelVisionEnabled(false);
        setGreenPixelVisionEnabled(false);
        setYellowPixelVisionEnabled(false);
        setWhitePixelVisionEnabled(false);
        setTensorFlowVisionEnabled(false);
    }   //Vision

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
     * This method enables/disables PurplePixel vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setPurplePixelVisionEnabled(boolean enabled)
    {
        if (purplePixelProcessor != null)
        {
            vision.setProcessorEnabled(purplePixelProcessor, enabled);
        }
    }   //setPurplePixelVisionEnabled

    /**
     * This method enables/disables GreenPixel vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setGreenPixelVisionEnabled(boolean enabled)
    {
        if (greenPixelProcessor != null)
        {
            vision.setProcessorEnabled(greenPixelProcessor, enabled);
        }
    }   //setGreenPixelVisionEnabled

    /**
     * This method enables/disables YellowPixel vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setYellowPixelVisionEnabled(boolean enabled)
    {
        if (yellowPixelProcessor != null)
        {
            vision.setProcessorEnabled(yellowPixelProcessor, enabled);
        }
    }   //setYellowVisionEnabled

    /**
     * This method enables/disables WhitePixel vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setWhitePixelVisionEnabled(boolean enabled)
    {
        if (whitePixelProcessor != null)
        {
            vision.setProcessorEnabled(whitePixelProcessor, enabled);
        }
    }   //setWhitePixelVisionEnabled

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
     * This method checks if AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagVisionEnabled()
    {
        return aprilTagProcessor != null && vision.isVisionProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    /**
     * This method checks if PurplePixel vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isPurplePixelVisionEnabled()
    {
        return purplePixelProcessor != null && vision.isVisionProcessorEnabled(purplePixelProcessor);
    }   //isPurplePixelVisionEnabled

    /**
     * This method checks if GreenPixel vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isGreenPixelVisionEnabled()
    {
        return greenPixelProcessor != null && vision.isVisionProcessorEnabled(greenPixelProcessor);
    }   //isGreenPixelVisionEnabled

    /**
     * This method checks if YellowPixel vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isYellowPixelVisionEnabled()
    {
        return yellowPixelProcessor != null && vision.isVisionProcessorEnabled(yellowPixelProcessor);
    }   //isYellowPixelVisionEnabled

    /**
     * This method checks if WhitePixel vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isWhitePixelVisionEnabled()
    {
        return whitePixelProcessor != null && vision.isVisionProcessorEnabled(whitePixelProcessor);
    }   //isWhitePixelVisionEnabled

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
