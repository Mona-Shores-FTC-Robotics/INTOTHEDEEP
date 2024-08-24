/*
 * Copyright (c) 2023 Sebastian Erives
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
 *
 */

package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.*;

import static com.example.sharedconstants.FieldConstants.*;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class InitVisionProcessor implements VisionProcessor {

    /** InitVisionProcessor Constants **/
    //This constant defines how much of a spike zone has to be the prop color for it to count as being detected
    private final int TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION = 5;
    private final int DIFFERENCE_THRESHOLD_FOR_RED_BLUE_DETECTION = 2;

    //This constant defines how much of a stage door zone needs to be yellow to count as being detected
    private final int STAGE_DOOR_THRESHOLD = 4;

    /** Scalars for filtering out certain pixel HSV ranges **/
    //Red1
    public Scalar lowerRed1 = new Scalar(0, 100, 20);
    public Scalar upperRed1 = new Scalar(12.8, 255, 255);

    //Red2
    public Scalar lowerRed2 = new Scalar(160, 100, 20);
    public Scalar upperRed2 = new Scalar(179, 255, 255);

    //Blue Filter
    public Scalar lowerBlue = new Scalar(39.7, 65.2, 0);
    public Scalar upperBlue = new Scalar(121.8, 255, 255);

    //Stage Door Filter (yellow)
    public Scalar lowerStageDoor = new Scalar(0, 127.5, 134.6);
    public Scalar upperStageDoor = new Scalar(86.4, 255, 255);

    /** Our Default Values **/
    public AllianceColor allianceColor = AllianceColor.RED;
    public SideOfField sideOfField = SideOfField.BACKSTAGE;
    public TeamPropLocation teamPropLocation = TeamPropLocation.CENTER;

    public AllianceColor allianceColorOverride = AllianceColor.RED;
    public SideOfField sideOfFieldOverride = SideOfField.BACKSTAGE;
    public TeamPropLocation teamPropLocationOverride = TeamPropLocation.CENTER;

    /** Matrices to store the camera images we are changing **/
    private Mat hsvMat       = new Mat();

    private Mat maskedRedMat = new Mat();
    private Mat maskedBlueMat = new Mat();
    private Mat maskedStageDoorMat = new Mat();

    private Mat binaryRedMat1      = new Mat();
    private Mat binaryRedMat2      = new Mat();
    private Mat binaryRedMatFinal      = new Mat();
    private Mat binaryBlueMat = new Mat();
    private Mat binaryStageDoorMat = new Mat();

    // Submatrices to store smaller portions of the screen so we can locate where things are
    private Mat leftZoneRed;
    private Mat centerZoneRed;
    private Mat rightZoneRed;

    private Mat leftZoneBlue;
    private Mat centerZoneBlue;
    private Mat rightZoneBlue;

    private Mat leftStageDoorZone;
    private Mat rightStageDoorZone;

    enum ColorSpace {
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public ColorSpace colorSpace = ColorSpace.HSV;

    /** variables for our five rectangles **/
    private int RectLX, RectLY, RectLwidth, RectLheight;
    private Rect rectL;

    private int RectMX, RectMY, RectMwidth, RectMheight;
    private Rect rectM;

    private int RectRX, RectRY, RectRwidth, RectRheight;
    private Rect rectR;

    private int RectLeftSideOfFieldX, RectLeftSideOfFieldY, RectLeftSideOfFieldWidth, RectLeftSideOfFieldHeight;
    private Rect rectLeftSideOfField;

    private int RectRightSideOfFieldX, RectRightSideOfFieldY, RectRightSideOfFieldWidth, RectRightSideOfFieldHeight;
    private Rect rectRightSideOfField;

    /** Rectangle colors **/
    private Scalar rectangleColorRed = new Scalar(255, 0, 0);
    private Scalar rectangleColorBlue = new Scalar(0, 0, 255);
    private Scalar rectangleColorGreen = new Scalar(0, 255, 0);
    private Scalar rectangleColorWhite = new Scalar(255, 255, 255);

    /** Percentage variables **/
    private double percentLeftZoneRed;
    private double percentCenterZoneRed;
    private double percentRightZoneRed;

    public double percentRedTotal;

    private double percentLeftZoneBlue;
    private double percentCenterZoneBlue;
    private double percentRightZoneBlue;

    public double percentBlueTotal;

    public double percentLeftStageDoorZone;
    public double percentRightStageDoorZone;

    public boolean allianceColorDeterminationProblem;

    public InitVisionProcessor() {
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        //Set parameters for Left Team Prop Location
        RectLX = 0;
        RectLY = (height*2)/3;
        RectLwidth = width/3;
        RectLheight = height-RectLY;
        rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);

        //set parameters for Center Team Prop Location
        RectMX = width/3+2;
        RectMY = (height*2)/3;
        RectMwidth = width/3-2;
        RectMheight = height-RectLY;
        rectM = new Rect(RectMX, RectMY, RectMwidth, RectMheight);

        //set parameters for Right Team Prop Location
        RectRX = (width*2)/3;
        RectRY = (height*2)/3;
        RectRwidth = width/3;
        RectRheight = height-RectLY;
        rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);

        //set parameters for Left Side Stage Door Detection
        RectLeftSideOfFieldX = 0;
        RectLeftSideOfFieldY = (height*1)/8;
        RectLeftSideOfFieldWidth = width/3;
        RectLeftSideOfFieldHeight = height/2;
        rectLeftSideOfField = new Rect(RectLeftSideOfFieldX, RectLeftSideOfFieldY, RectLeftSideOfFieldWidth, RectLeftSideOfFieldHeight);

        //set parameters for Right Side Stage Door Detection
        RectRightSideOfFieldX = (width*2)/3;
        RectRightSideOfFieldY = (height*1)/8;
        RectRightSideOfFieldWidth = width/3;
        RectRightSideOfFieldHeight = height/2;
        rectRightSideOfField = new Rect(RectRightSideOfFieldX, RectRightSideOfFieldY, RectRightSideOfFieldWidth, RectRightSideOfFieldHeight);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, colorSpace.cvtCode);

        /*
         * This is where our thresholding actually happens.
         * Takes our "hsvMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(hsvMat, lowerRed1, upperRed1, binaryRedMat1);
        Core.inRange(hsvMat, lowerRed2, upperRed2, binaryRedMat2);
        Core.bitwise_or(binaryRedMat1, binaryRedMat2, binaryRedMatFinal);

        Core.inRange(hsvMat, lowerBlue, upperBlue, binaryBlueMat);

        Core.inRange(hsvMat, lowerStageDoor, upperStageDoor, binaryStageDoorMat);

        Core.bitwise_and(frame, frame, maskedRedMat, binaryRedMatFinal);
        Core.bitwise_and(frame, frame, maskedBlueMat, binaryBlueMat);
        Core.bitwise_and(frame, frame, maskedStageDoorMat, binaryStageDoorMat);

        //this is where we store certain areas of the picture so we can determine alliance color/side of field/team prop location
        leftZoneRed  = binaryRedMatFinal.submat(rectL);
        centerZoneRed = binaryRedMatFinal.submat(rectM);
        rightZoneRed  = binaryRedMatFinal.submat(rectR);

        leftZoneBlue  = binaryBlueMat.submat(rectL);
        centerZoneBlue = binaryBlueMat.submat(rectM);
        rightZoneBlue  = binaryBlueMat.submat(rectR);

        leftStageDoorZone = binaryStageDoorMat.submat(rectLeftSideOfField);
        rightStageDoorZone = binaryStageDoorMat.submat(rectRightSideOfField);

        /** use gamepad so we can easily see what our camera is filtering for
         * the blue filter / red filter / or yellow(stage door) **/


        if (Robot.getInstance().getActiveOpMode().gamepad1.left_trigger>.1) {
            binaryRedMatFinal.copyTo(frame);
        }
        if (Robot.getInstance().getActiveOpMode().gamepad1.right_trigger>.1) {
            binaryBlueMat.copyTo(frame);
        }
        if (Robot.getInstance().getActiveOpMode().gamepad1.left_bumper) {
            binaryStageDoorMat.copyTo(frame);
        }

        //Calculate percentage values from the binary matrices (number of white pixels vs. black pixels for a particular area)
        percentLeftZoneRed = ((Core.sumElems(leftZoneRed).val[0] /255) / rectL.area()) * 100 ;
        percentCenterZoneRed = ((Core.sumElems(centerZoneRed).val[0]/255) / rectM.area()) * 100 ;
        percentRightZoneRed= ((Core.sumElems(rightZoneRed).val[0]/255) / rectR.area()) * 100 ;

        percentLeftZoneBlue = ((Core.sumElems(leftZoneBlue).val[0]/255) / rectL.area()) * 100 ;
        percentCenterZoneBlue = ((Core.sumElems(centerZoneBlue).val[0]/255) / rectM.area()) * 100 ;
        percentRightZoneBlue= ((Core.sumElems(rightZoneBlue).val[0]/255) / rectR.area()) * 100 ;

        percentLeftStageDoorZone = ((Core.sumElems(leftStageDoorZone).val[0] / 255) / rectLeftSideOfField.area()) * 100;
        percentRightStageDoorZone = ((Core.sumElems(rightStageDoorZone).val[0] / 255) / rectRightSideOfField.area()) * 100;


        // Add up the percent of white pixels in the left, center, and right boxes
        // This is for convenience because while we practice we want to be able to place the prop in any location and have it figure out the alliance color
        // - If we run into trouble, we might want to only look at the center Zone to determine alliance color because we always place our prop in the middle
        percentRedTotal = percentLeftZoneRed + percentCenterZoneRed + percentRightZoneRed;
        percentBlueTotal = percentLeftZoneBlue + percentCenterZoneBlue + percentRightZoneBlue;

        DetermineAllianceColor();
        DetermineSideOfField();
//        DetermineTeamPropLocation();

        /*
         * Different from OpenCvPipeline, you cannot return
         * a Mat from processFrame. Therefore, we will take
         * advantage of the fact that anything drawn onto the
         * passed `frame` object will be displayed on the
         * viewport. We will just return null here.
         */

        //frame should be in a helpful state due to processing above

        //Draw green rectangle to represent where the vision code thinks the prop is located
//        if (teamPropLocation == TeamPropLocation.LEFT)
//        {
//            Imgproc.rectangle(frame, rectL, rectangleColorGreen, 2);
//            Imgproc.rectangle(frame, rectM, rectangleColorWhite, 2);
//            Imgproc.rectangle(frame, rectR, rectangleColorWhite, 2);
//        } else if (teamPropLocation == TeamPropLocation.CENTER)
//
//        {
//            Imgproc.rectangle(frame, rectM, rectangleColorGreen, 2);
//            Imgproc.rectangle(frame, rectL, rectangleColorWhite, 2);
//            Imgproc.rectangle(frame, rectR, rectangleColorWhite, 2);
//        } else if (teamPropLocation == TeamPropLocation.RIGHT)
//        {
//            Imgproc.rectangle(frame, rectR, rectangleColorGreen, 2);
//            Imgproc.rectangle(frame, rectL, rectangleColorWhite, 2);
//            Imgproc.rectangle(frame, rectM, rectangleColorWhite, 2);
//        }

        //Draw colored rectangle where the stage door is to represent
        // we know our alliance color and we know the side of the field we are on
//        if (allianceColor == AllianceColor.RED &&  sideOfField == SideOfField.BACKSTAGE) {
//            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorRed, 2);
//            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorWhite, 2);
//        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.AUDIENCE) {
//            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorWhite, 2);
//            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorRed, 2);
//
//        } else if (allianceColor == AllianceColor.BLUE &&  sideOfField == SideOfField.BACKSTAGE) {
//            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorWhite, 2);
//            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorBlue, 2);
//
//        } else if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.AUDIENCE) {
//            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorBlue, 2);
//            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorWhite, 2);
//
//        }
        //Release all the mats
        hsvMat.release();
        maskedRedMat.release();
        maskedBlueMat.release();
        maskedStageDoorMat.release();
        binaryRedMat1.release();
        binaryRedMat2.release();
        binaryRedMatFinal.release();
        binaryBlueMat.release();
        binaryStageDoorMat.release();
        leftZoneRed.release();
        centerZoneRed.release();
        rightZoneRed.release();
        leftZoneBlue.release();
        centerZoneBlue.release();
        rightZoneBlue.release();
        leftStageDoorZone.release();
        rightStageDoorZone.release();

        return null;
    }

//    private void DetermineTeamPropLocation() {
//        if (allianceColor == AllianceColor.RED) {
//            //Figure out and Store the Team Prop Location
//            if (percentLeftZoneRed > percentCenterZoneRed && percentLeftZoneRed > percentRightZoneRed) {
//                // Red Team Prop is on the Left
//                teamPropLocation = TeamPropLocation.LEFT;
//            } else if (percentCenterZoneRed > percentLeftZoneRed && percentCenterZoneRed > percentRightZoneRed) {
//                // Team Prop is in the Middle
//                teamPropLocation = TeamPropLocation.CENTER;
//            } else if (percentRightZoneRed > percentCenterZoneRed && percentRightZoneRed > percentLeftZoneRed) {
//                // Team Prop is on th Right
//                teamPropLocation = TeamPropLocation.RIGHT;
//            } else {
//                teamPropLocation = TeamPropLocation.CENTER;
//            }
//        }
//
//        if (allianceColor == AllianceColor.BLUE) {
//            if (percentLeftZoneBlue > percentCenterZoneBlue && percentLeftZoneBlue > percentRightZoneBlue) {
//                // Red Team Prop is on the Left
//                teamPropLocation = TeamPropLocation.LEFT;
//            } else if (percentCenterZoneBlue > percentLeftZoneBlue && percentCenterZoneBlue > percentRightZoneBlue) {
//                // Team Prop is in the Middle
//                teamPropLocation = TeamPropLocation.CENTER;
//            } else if (percentRightZoneBlue > percentCenterZoneBlue && percentRightZoneBlue > percentLeftZoneBlue) {
//                // Team Prop is on the Right
//                teamPropLocation = TeamPropLocation.RIGHT;
//            } else {
//                teamPropLocation = TeamPropLocation.CENTER;
//            }
//        }
//    }

    private void DetermineSideOfField() {

        // Here is our steps for Side of Field selection
        // 1. Reset Backstage/Audience determination problem flag
        // 2. check if the total left vs. right box are too close to call - set problem flag if it is but continue
        // 3. Compare each zone to a threshold (4%)
        //      If nether zone is greater than the threshold set default to Backstage, but flag an error
        //      otherwise set an over-threshold true/false variable for backstage/Audience
        // 4. If only the left box or the right box is over the threshold, then set the side of field based on that using the alliance color
        //      if they are both over the threshold set the side of field to default (Backstage) and flag an error

        if (finalAllianceColor == AllianceColor.RED) {
            if (percentLeftStageDoorZone >= percentRightStageDoorZone && percentLeftStageDoorZone > STAGE_DOOR_THRESHOLD) {
                // Stage Door is on the left and we are Red Alliance so we are BACKSTAGE
                sideOfField = SideOfField.BACKSTAGE;
            } else if (percentRightStageDoorZone > percentLeftStageDoorZone && percentRightStageDoorZone > STAGE_DOOR_THRESHOLD)
            {
                // Stage Door is on the right and we are Red Alliance so we are AUDIENCE
                sideOfField = SideOfField.AUDIENCE;
            }
        }

        if (allianceColor == AllianceColor.BLUE) {
            //Figure out where the Stage Door is
            if (percentLeftStageDoorZone >= percentRightStageDoorZone && percentLeftStageDoorZone > STAGE_DOOR_THRESHOLD) {
                // Stage Door is on the left and we are Blue Alliance so we are FRONTSTAGE
                sideOfField = SideOfField.AUDIENCE;
            } else if (percentRightStageDoorZone > percentLeftStageDoorZone && percentRightStageDoorZone > STAGE_DOOR_THRESHOLD)
            {
                // Stage Door is on the right and we are Blue Alliance so we are BACKSTAGE
                sideOfField = SideOfField.BACKSTAGE;
            }
        }
    }

    private void DetermineAllianceColor() {

        // Here are our steps for alliance color selection
        // 1. Reset alliance color determination problem flag
        // 2. check if the total red vs. blue is too close to call - set the problem flag if it is, but continue
        // 3. compare each zone to a threshold (5 percent)
        //      If no zone is greater than the threshold, set the default to Red, but flag an error
        //      Otherwise set an over-threshold true/false variable for blue/red
        // 4. If only blue or only red was over the threshold, then set the alliance color based on that
        //      if blue and red both are over threshold, set the alliance color to red, but flag the problem

        //Reset our variable
        allianceColorDeterminationProblem=false;
        boolean redOverThreshold = false;
        boolean blueOverThreshold = false;

        if (Math.abs(percentRedTotal - percentBlueTotal) < DIFFERENCE_THRESHOLD_FOR_RED_BLUE_DETECTION)
        {
            //set the problem variable to flag this is too close to call
            allianceColorDeterminationProblem=true;
        }

        if (    percentLeftZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentCenterZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentRightZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION)
        {
            redOverThreshold=true;
        }
        if (percentLeftZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentCenterZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentRightZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION)
        {
            blueOverThreshold=true;
        }

        if (!redOverThreshold && !blueOverThreshold)
        {
            //if neither red or blue percents are over the threshold anywhere, then default to red, but tell the driver using the problem boolean
            allianceColor = AllianceColor.RED;
            allianceColorDeterminationProblem=true;
        }

        //if both red and blue are over the threshold, then default to red, but tell the driver using the problem boolean
        if (redOverThreshold && blueOverThreshold)
        {
            allianceColor = AllianceColor.RED;
            allianceColorDeterminationProblem=true;
        } else if (redOverThreshold)
        {
            allianceColor = AllianceColor.RED;
        } else if (blueOverThreshold)
        {
            allianceColor = AllianceColor.BLUE;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    public double getLeftPercent() {
        if (allianceColor.equals(AllianceColor.RED)) {
            return percentLeftZoneRed;
        } else if (allianceColor.equals(AllianceColor.BLUE)) {
            return percentLeftZoneBlue;
        }
        return percentLeftZoneBlue;
    }

    public double getCenterPercent() {
        if (allianceColor == AllianceColor.RED) {
            return percentCenterZoneRed;
        } else if (allianceColor == AllianceColor.BLUE) {
            return percentCenterZoneBlue;
        }
        return percentCenterZoneBlue;
    }

    public double getRightPercent()
    {
        if (allianceColor == AllianceColor.RED) {
            return percentRightZoneRed;
        } else if (allianceColor == AllianceColor.BLUE) {
            return percentRightZoneBlue;
        }
        return percentRightZoneBlue;
    }

    public TeamPropLocation getTeamPropLocation()
    {
        return teamPropLocation;
    }

//    public void setSideOfFieldFinal(SideOfField side) {
//        if (GamepadHandling.LockedInitSettingsFlag != true)
//        {
//            sideOfFieldFinal = side;
//        }
//    }

//    public SideOfField getSideOfFieldFinal()
//    {
//        if (GamepadHandling.ManualOverrideInitSettingsFlag == true) {
//            return sideOfFieldOverride;
//        } else
//        {
//            return sideOfFieldFinal;
//        }
//    }

//    public void setAllianceColorFinal(AllianceColor color) {
//        if (GamepadHandling.LockedInitSettingsFlag != true)
//        {
//            allianceColorFinal = color;
//        }
//    }
//
//    public AllianceColor getAllianceColorFinal()
//    {
//        if (GamepadHandling.ManualOverrideInitSettingsFlag == true) {
//            return allianceColorOverride;
//        } else
//        {
//            return allianceColorFinal;
//        }
//    }
}