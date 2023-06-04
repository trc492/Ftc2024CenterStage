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

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;

import java.util.Comparator;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;
import teamcode.RobotParams;
import teamcode.vision.Vision;

/**
 * This class implements TensorFlow Vision that provides the capability to detect learned objects and return their
 * location info.
 */
public class TensorFlowVision
{
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final float TFOD_MIN_CONFIDENCE = 0.75f;

    private final TrcDbgTrace tracer;
    private FtcTensorFlow tensorFlow;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param vuforia specifies the FtcVuforia object.
     * @param tracer specifies the tracer to use for logging events.
     */
    public TensorFlowVision(FtcVuforia vuforia, TrcDbgTrace tracer)
    {
        System.loadLibrary(Vision.OPENCV_NATIVE_LIBRARY_NAME);
        FtcOpMode opMode = FtcOpMode.getInstance();
        int tfodMonitorViewId = !RobotParams.Preferences.showTensorFlowView ? -1 :
            opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //
        // If no TFOD monitor view ID, do not activate camera monitor view to save power.
        //
        TFObjectDetector.Parameters tfodParams =
            tfodMonitorViewId == -1?
                new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParams.minResultConfidence = TFOD_MIN_CONFIDENCE;
        tfodParams.isModelTensorFlow2 = true;
        tfodParams.inputSize = 300;

        this.tracer = tracer;
        tensorFlow = new FtcTensorFlow(
            vuforia, tfodParams, TFOD_MODEL_ASSET, Vision.TARGET_LABELS,
            RobotParams.cameraRect, RobotParams.worldRect, tracer);
    }   //TensorFlowVision

    /**
     * This method enables/disables TensorFlow.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        tensorFlow.setEnabled(enabled);
        if (enabled)
        {
            tensorFlow.setZoom(1.0, 16.0/9.0);
        }
    }   //setEnabled

    /**
     * This method checks if TensorFlow is enabled.
     *
     * @return true if TensorFlow is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return tensorFlow.isEnabled();
    }   //isEnabled

    /**
     * This method shuts down TensorFlow.
     */
    public void shutdown()
    {
        setEnabled(false);
        if (tensorFlow != null)
        {
            tensorFlow.shutdown();
            tensorFlow = null;
        }
    }   //shutdown

    /**
     * This method sets the zoom factor and aspect ratio for the camera.
     *
     * @param factor specifies the camera magnification factor.
     * @param aspectRatio specifies the camera aspect ratio (width/height).
     */
    public void setZoom(double factor, double aspectRatio)
    {
        tensorFlow.setZoom(factor, aspectRatio);
    }   //setZoom

    /**
     * This method sets the zoom factor for the camera.
     *
     * @param factor specifies the camera magnification factor.
     */
    public void setZoom(double factor)
    {
        tensorFlow.setZoom(factor, 16.0/9.0);
    }   //setZoom

    /**
     * This method returns an array of detected targets from TensorFlow vision.
     *
     * @param label specifies the label of the targets to detect for, can be null for detecting any target.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] getDetectedTargetsInfo(
        String label, FtcTensorFlow.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        final String funcName = "getDetectedTargetsInfo";
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] targets =
            tensorFlow.getDetectedTargetsInfo(label, filter, comparator, objHeightOffset, cameraHeight);

        if (tracer != null && targets != null)
        {
            for (int i = 0; i < targets.length; i++)
            {
                tracer.traceInfo(funcName, "[%d] Target=%s", i, targets[i]);
            }
        }

        return targets;
    }   //getDetectedTargetsInfo

    /**
     * This method maps a camera screen point to a real-world point.
     *
     * @param point specifies the camera screen point.
     * @return real-world point.
     */
    public Point mapPoint(Point point)
    {
        return tensorFlow != null? tensorFlow.mapPoint(point): null;
    }   //mapPoint

    /**
     * This method maps a camera screen point to a real-world point.
     *
     * @param x specifies the camera screen point x.
     * @param y specifies the camera screen point y.
     * @return real-world point.
     */
    public Point mapPoint(double x, double y)
    {
        return mapPoint(new Point(x, y));
    }   //mapPoint

}   //class TensorFlowVision
