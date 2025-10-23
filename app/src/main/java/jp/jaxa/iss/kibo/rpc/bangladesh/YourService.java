package jp.jaxa.iss.kibo.rpc.bangladesh;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.*;

import android.graphics.RectF;
import android.util.Log;
import android.os.SystemClock;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;

import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.dnn.Net;
import org.opencv.dnn.Dnn;

import org.opencv.objdetect.ArucoDetector;
import static org.opencv.objdetect.Objdetect.DICT_5X5_250;
import static org.opencv.objdetect.Objdetect.getPredefinedDictionary;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    private static Net yoloNet;
    private Map<Integer, Mat> areaImage = new HashMap<>();
    private Map<Integer, Mat> postImage = new HashMap<>();
    private Map<Integer, Mat> preImage = new HashMap<>();
    private boolean debugMode = true;

    @Override
    protected void runPlan1(){
        //--- Target Positions
        Map<Integer, gov.nasa.arc.astrobee.types.Point> points = new HashMap<>();
        points.put(1, new gov.nasa.arc.astrobee.types.Point(10.9,-9.55,4.80)); // Target 1
        points.put(2, new gov.nasa.arc.astrobee.types.Point(10.93,-8.28,4.72)); // Target 2 & 3
        points.put(4, new gov.nasa.arc.astrobee.types.Point(11.1,-6.80,4.85)); // Target 4
        points.put(5, new gov.nasa.arc.astrobee.types.Point(11.37d,-6.80d,4.80d)); // Target 5 (Astronaut)
        points.put(6, new gov.nasa.arc.astrobee.types.Point(11.2d,-6.80d,4.80d)); // Oasis

        Map<Integer, Quaternion> quaternions = new HashMap<>();
        quaternions.put(1, new Quaternion(-0.03f, -0.03f, -0.68f,  0.73f)); // Target 1
        quaternions.put(2, new Quaternion(0f, 0.68f, 0f,  0.73f)); // Target 2 & 3
        quaternions.put(4, new Quaternion(-0.0f, -0.0f, -1.0f,  0.07f)); // Target 4
        quaternions.put(5, new Quaternion(0f, 0f, -0.707f, 0.707f)); // Target 5 (Astronaut)

        // The mission starts.
        api.startMission();
        String treasureItem1 = "", treasureItem2 = "", treasureItem3 = "", treasureItem4 = "";

        long delay = 3000;

        // Target-1
        moveAstrobee(points.get(1), quaternions.get(1));

        long startTime = System.currentTimeMillis();
        Log.i("My Debug", "Loading model");
        if (yoloNet == null) {
            yoloNet = Dnn.readNetFromONNX(getModelFile().getAbsolutePath());
        }
        Log.i("My Debug", "Loading model complete");
        long elapsedTime = System.currentTimeMillis() - startTime;

        SystemClock.sleep(delay - elapsedTime);
        captureArea(1);

        // Target-2
        moveAstrobee(points.get(2), quaternions.get(2));

        startTime = System.currentTimeMillis();
        process_AR(1);
        treasureItem1 = detectObject(1);
        elapsedTime = System.currentTimeMillis() - startTime;

        SystemClock.sleep(delay - elapsedTime);
        captureArea(2);

        // Target-3
        captureArea(3);

        // Target-4
        moveAstrobee(points.get(4), quaternions.get(4));

        startTime = System.currentTimeMillis();
        process_AR(2);
        treasureItem2 = detectObject(2);
        process_AR(3);
        treasureItem3 = detectObject(3);
        elapsedTime = System.currentTimeMillis() - startTime;

        SystemClock.sleep(delay - elapsedTime);
        captureArea(4);


        // Detect items from the captured images
        moveAstrobee(points.get(5), quaternions.get(5));

        process_AR(4);
        treasureItem4 = detectObject(4);

        moveAstrobee(points.get(5), quaternions.get(5));

        Log.i("My Debug", "Detection Ends");

        // Rounding Complete
        if(debugMode) {
            Kinematics pos = api.getRobotKinematics();
            Log.i("My Debug", "Confidence: " + pos.getConfidence());
            Log.i("My Debug", "Position: " + pos.getPosition());
        }
        api.reportRoundingCompletion();
        moveAstrobee(points.get(5), quaternions.get(5));

        // Detect Final Target
        captureArea(5);
        boolean ar_target_5 = process_AR(5);

        for(int i = 0 ; i < 40; i++) {
            moveAstrobee(points.get(5), quaternions.get(5));
            if(!ar_target_5) {
                SystemClock.sleep(500);
                captureArea(5);
                ar_target_5 = process_AR(5);
            } else {
                break;
            }
        }

        String treasureItem5 = detectObject(5);
        api.notifyRecognitionItem();

        int area;
        if(treasureItem5.equals(treasureItem1)) {
            area = 1;
        }
        else if(treasureItem5.equals(treasureItem2)) {
            area = 2;
        }
        else if(treasureItem5.equals(treasureItem3)) {
            area = 3;
        }
        else if(treasureItem5.equals(treasureItem4)) {
            area = 4;
        }
        else {
            Random random = new Random();
            area = random.nextInt(4 - 1 + 1) + 1; // Assigning random values in the name of Allah if final target recognition fails.
        }

        Log.i("My Debug", "Area: " + area);

        boolean checkFinal = finalTarget(area);

        if(!checkFinal) {
            moveAstrobee(points.get(6), quaternions.get(5));
            finalTarget(area);
        }

        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // Move Astrobee to desired location
    private boolean moveAstrobee(gov.nasa.arc.astrobee.types.Point point, Quaternion quaternion ) {
        Result result = api.moveTo(point, quaternion, false);


        // Repeat if move fails
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point , quaternion, false);
            loopCounter++;
        }

        return result.hasSucceeded();
    }

    public void captureArea(int target_num) {
        Mat src = new Mat();

        if(target_num == 5) {
            src = api.getMatDockCam();
        } else {
            src = api.getMatNavCam();
        }

        areaImage.put(target_num, src); // Store original image
    }

    public boolean process_AR (int target_num) {
        try {
            Mat src = areaImage.get(target_num);
            if (src == null || src.empty()) {
                Log.i("My Debug", "Original Image not found for target " + target_num);
                return false;
            }

            Mat kernel = new Mat(3, 3, CvType.CV_32F);
            kernel.put(0, 0, 0);
            kernel.put(0, 1, -1);
            kernel.put(0, 2, 0);
            kernel.put(1, 0, -1);
            kernel.put(1, 1, 5);
            kernel.put(1, 2, -1);
            kernel.put(2, 0, 0);
            kernel.put(2, 1, -1);
            kernel.put(2, 2, 0);
            double[][] cameraParam = api.getNavCamIntrinsics();
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
            Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
            cameraMatrix.put(0, 0, cameraParam[0]);
            dstMatrix.put(0, 0, cameraParam[1]);
            Mat sharpened = new Mat();
            Mat undistort = new Mat();

            Calib3d.undistort(src, undistort, cameraMatrix, dstMatrix);

            Imgproc.filter2D(undistort, sharpened, -1, kernel);

            preImage.put(target_num, sharpened.clone());

            if(debugMode) api.saveMatImage(sharpened, "pre_" + target_num + ".png");

            ArucoDetector arucoDetector = new ArucoDetector(getPredefinedDictionary(DICT_5X5_250));
            Mat ids = new Mat();

            List<Mat> corners = new ArrayList<>();

            arucoDetector.detectMarkers(sharpened, corners, ids);

            boolean detected = false;

            if (!corners.isEmpty()) {
                float markerLength = 0.05f;
                // Find the marker with the desired x-coordinate
                int selectedIndex = 0;
                double selectedX = corners.get(0).get(0, 0)[0];  // x-coordinate of the first corner of the first marker

                for (int i = 1; i < corners.size(); i++) {
                    double x = corners.get(i).get(0, 0)[0];
                    if ((target_num == 3 && x > selectedX) || (target_num == 2 && x < selectedX)) {
                        selectedX = x;
                        selectedIndex = i;
                    }
                }

                // Use the marker with the desired y-coordinate for further processing
                Mat selectedCorner = corners.get(selectedIndex);
                int selectedId = (int) ids.get(selectedIndex, 0)[0];
                List<Mat> selectedCorners = new ArrayList<>();
                selectedCorners.add(selectedCorner);
                // Estimate pose of the selected marker

                MatOfPoint2f cornerPoints = new MatOfPoint2f(selectedCorner);
                Point[] cornerArray = cornerPoints.toArray();

                // Calculate the Euclidean distances
                double pixelDistance1 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[1]));
                double pixelDistance2 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[3]));
                double pixelDistance3 = Core.norm(new MatOfPoint2f(cornerArray[1]), new MatOfPoint2f(cornerArray[2]));
                double pixelDistance4 = Core.norm(new MatOfPoint2f(cornerArray[2]), new MatOfPoint2f(cornerArray[3]));
                double pixelDistance = (pixelDistance1 + pixelDistance2 + pixelDistance3 + pixelDistance4) / 4;

                // Calculate the ratio
                double pixelToMRatio = pixelDistance / markerLength;


                // Print the pose information
                double TL = pixelToMRatio * 0.28;
                double TR = pixelToMRatio * 0.015;
                double TH = pixelToMRatio * 0.06;
                double BH = pixelToMRatio * 0.17;
                double angle = Math.atan2(cornerArray[0].y - cornerArray[1].y, cornerArray[0].x - cornerArray[1].x);
                double angleDegrees = Math.toDegrees(angle);
                Point center = new Point((cornerArray[0].x + cornerArray[2].x) / 2, (cornerArray[0].y + cornerArray[2].y) / 2);
                Mat rotationMatrix = Imgproc.getRotationMatrix2D(center, angleDegrees, -1.0);
                Mat rotatedImage = new Mat();
                Imgproc.warpAffine(sharpened, rotatedImage, rotationMatrix, undistort.size());

                // Compute the new corner points after rotation
                MatOfPoint2f newCorners = new MatOfPoint2f();
                Core.perspectiveTransform(cornerPoints, newCorners, rotationMatrix);
                Point[] newCornerArray = newCorners.toArray();
                int centerX = (int) center.x;
                int centerY = (int) center.y;

                int xMin = (int) (centerX - TL);
                int yMin = (int) (centerY - TH);
                int xMax = (int) (centerX + TR);
                int yMax = (int) (centerY + BH);

                // Ensure the coordinates are within image bounds
                xMin = Math.max(0, xMin);
                yMin = Math.max(0, yMin);
                xMax = Math.min(rotatedImage.width(), xMax);
                yMax = Math.min(rotatedImage.height(), yMax);

                // Crop the image
                Rect roi = new Rect(xMin, yMin, xMax - xMin, yMax - yMin);
                Mat croppedImage = new Mat(rotatedImage, roi);
                postImage.put(target_num, croppedImage.clone()); // Store cropped image

                detected=true;
                if(debugMode) api.saveMatImage(croppedImage,"post_" + target_num + ".png");

            } else if(detected==false){
                Log.i("My Debug","AR not found");
            }

            Log.i("My Debug", "AR: " + target_num + ": " + detected);

            return detected;
        } catch(Exception e) {
            Log.i("My Debug", "Something went wrong when processing area: " + e.getMessage());
            return false;
        }
    }

    public boolean finalTarget(int area) {
        try {
            Mat src = preImage.get(area);
            if (src == null || src.empty()) {
                Log.i("My Debug", "Image not found for target " + area);
                return false;
            }

            ArucoDetector arucoDetector = new ArucoDetector(getPredefinedDictionary(DICT_5X5_250));
            Mat ids = new Mat();

            List<Mat> corners = new ArrayList<>();

            arucoDetector.detectMarkers(src, corners, ids);

            if (!corners.isEmpty()) {
                // Find the marker with the desired x-coordinate
                int selectedIndex = 0;
                double selectedX = corners.get(0).get(0, 0)[0];  // x-coordinate of the first corner of the first marker

                for (int i = 1; i < corners.size(); i++) {
                    double x = corners.get(i).get(0, 0)[0];
                    if ((area == 3 && x > selectedX) || (area == 2 && x < selectedX)) {
                        selectedX = x;
                        selectedIndex = i;
                    }
                }

                // Take the correct marker
                Mat selectedCorner = corners.get(selectedIndex);
                MatOfPoint2f cornerPoints = new MatOfPoint2f(selectedCorner);
                Point[] cornerArray = cornerPoints.toArray();

                double centerX = 0, centerY = 0;
                for (Point p : cornerArray) {
                    centerX += p.x;
                    centerY += p.y;
                }
                centerX /= 4.0;
                centerY /= 4.0;

                // Calculate image center
                double imageCenterX = src.cols() / 2.0;
                double imageCenterY = src.rows() / 2.0;

                // Compute differences
                double xDiff = centerX - imageCenterX;
                double yDiff = centerY - imageCenterY;

                // Log differences
                Log.i("My Debug", "X Difference: " + xDiff + ", Y Difference: " + yDiff);

                double prec = 0.0008;

                if(area == 1) prec = 0.0011;
                if(area == 2) prec = 0.0012;
                if(area == 3) prec = 0.0012;
                if(area == 4) prec = 0.0013;
                Log.i("My Debug", "Prec:" + prec);
                Log.i("My Debug", "X Difference: " + xDiff*prec + ", Y Difference: " + yDiff*prec);

                double yPoint = -8.28+xDiff*prec;

                double area4Point = -6.80-xDiff*prec;

                if(area == 2) {
                    if(yPoint > -8.6) yPoint = -8.6;
                } else if(area == 3) {
                    if(yPoint < -8.1) yPoint = -8.1;
                } else if(area == 4) {
                    if(area4Point > -6.465) area4Point = -6.465;
                }

                Map<Integer, gov.nasa.arc.astrobee.types.Point> points = new HashMap<>();
                points.put(1, new gov.nasa.arc.astrobee.types.Point(10.9+xDiff*prec,-9.73,4.80+yDiff*prec)); // Target 1
                points.put(2, new gov.nasa.arc.astrobee.types.Point(10.93+yDiff*prec, yPoint,4.61)); // Target 2
                points.put(3, new gov.nasa.arc.astrobee.types.Point(10.93+yDiff*prec, yPoint,4.61)); // Target 3
                points.put(4, new gov.nasa.arc.astrobee.types.Point(10.71,area4Point,4.85+yDiff*prec)); // Target 4
                points.put(5, new gov.nasa.arc.astrobee.types.Point(11.325d,-6.80d,4.80d)); // Target 5 (Astronaut)


                Map<Integer, Quaternion> quaternions = new HashMap<>();
                quaternions.put(1, new Quaternion(-0.03f, -0.03f, -0.68f,  0.73f)); // Target 1
                quaternions.put(2, new Quaternion(0f, 0.68f, 0f,  0.73f)); // Target 2
                quaternions.put(3, new Quaternion(0f, 0.68f, 0f,  0.73f)); // Target 3
                quaternions.put(4, new Quaternion(-0.0f, -0.0f, -1.0f,  0.07f)); // Target 4

                return moveAstrobee(points.get(area), quaternions.get(area));
            } else {
                Log.i("My Debug", "Final Target marker not detected");
                return false;
            }
        } catch(Exception e) {
            Log.i("My Debug", "Something went wrong when going to final target: " + e.getMessage());
            return false;
        }
    }

    public String detectObject(int target_num) {
        try {
            Mat originalImg = postImage.get(target_num);
            if (originalImg == null || originalImg.empty()) {
                Log.i("My Debug", "Image not found for target " + target_num);
                return "";
            }

            // Load and run model
            if (yoloNet.empty()) {
                Log.i("My Debug", "ONNX model failed to load");
                return "";
            }

            Net net = yoloNet;

            int IN_WIDTH = 320;
            int IN_HEIGHT = 320;

            double yScale = ((double) (originalImg.size().height)) / IN_HEIGHT;
            double xScale = ((double) originalImg.size().width) / IN_WIDTH;

            Mat imgRGB = new Mat();

            Imgproc.cvtColor(originalImg, imgRGB, Imgproc.COLOR_GRAY2RGB);

            Mat blob = Dnn.blobFromImage(imgRGB, 1.0 / 255.0, new Size(IN_WIDTH, IN_HEIGHT), new Scalar(0, 0, 0), true, false);

            net.setInput(blob);
            Mat outputs = net.forward();
//            Log.i("My Debug", "Outputs Dims: " + outputs.dims() + ", Size: " + outputs.size() + ", Channels: " + outputs.channels());
//            Log.i("My Debug", "Outputs rows: " + outputs.rows() + ", cols: " + outputs.cols()); // Often 1 row, N*(4+C) cols


            // Process detections
            Mat mask = outputs.reshape(0, 1).reshape(0, outputs.size(1));
//            Log.i("My Debug", "Mask Dims: " + mask.dims() + ", Size: " + mask.size() + ", Channels: " + mask.channels());
//            Log.i("My Debug", "Mask rows: " + mask.rows() + ", cols: " + mask.cols());

            List<Rect2d> rect2dList = new ArrayList<>();
            List<Float> scoreList = new ArrayList<>();
            List<Integer> classIdList = new ArrayList<>();

            float margin_in_per = 3f; // margin percentage
            float imageWidth = originalImg.width();
            float imageHeight = originalImg.height();

            for (int i = 0; i < mask.cols(); i++) {
                double[] x = mask.col(i).get(0, 0);
                double[] y = mask.col(i).get(1, 0);
                double[] w = mask.col(i).get(2, 0);
                double[] h = mask.col(i).get(3, 0);

                double cx = x[0];
                double cy = y[0];
                double width = w[0];
                double height = h[0];

                double x1 = (cx - width / 2) * xScale;
                double y1 = (cy - height / 2) * yScale;
                double scaledWidth = width * xScale;
                double scaledHeight = height * yScale;

                RectF rectF = new RectF((float)x1, (float)y1, (float)(x1 + scaledWidth), (float)(y1 + scaledHeight));
                if (isNearEdge(rectF, imageWidth, imageHeight, margin_in_per)) {
//                    Log.i("My Debug", "Skipping box " + i + " near edge.");
                    continue;
                }

                Rect2d rect = new Rect2d(x1, y1, scaledWidth, scaledHeight);
                Mat score = mask.col(i).submat(4, outputs.size(1), 0, 1);
                Core.MinMaxLocResult mmr = Core.minMaxLoc(score);

                rect2dList.add(rect);
                scoreList.add((float) mmr.maxVal);
                classIdList.add((int) mmr.maxLoc.y);
            }

            MatOfRect2d bboxes = new MatOfRect2d();
            bboxes.fromList(rect2dList);

            float[] scoref = new float[scoreList.size()];
            for (int i = 0; i < scoreList.size(); i++) scoref[i] = scoreList.get(i);

            int[] classid = new int[classIdList.size()];
            for (int i = 0; i < classIdList.size(); i++) classid[i] = classIdList.get(i);

            MatOfFloat scores = new MatOfFloat(scoref);
            MatOfInt indices = new MatOfInt();

            Dnn.NMSBoxes(bboxes, scores, 0.4f, 0.6f, indices);
            Log.i("My Debug", "Remaining after edge filtering + NMS: " + indices.total());


            List<Integer> result = indices.total() > 0 ? indices.toList() : new ArrayList<Integer>();

            String treasureItem = "";
            String landmarkItemName = "";
            int landmarkItemCount = 0;

            for (Integer idx : result) {
                if(debugMode) imgRGB = drawBoundingBox(imgRGB, classid[idx], scoref[idx], rect2dList.get(idx));

                String label = classNames[classid[idx]];
                if (treasureItems.contains(label)) {
                    treasureItem = label;  // last treasure item found will be returned
                } else if (landmarkItems.contains(label)) {
                    if (landmarkItemName.isEmpty() || landmarkItemName.equals(label)) {
                        landmarkItemName = label;
                        landmarkItemCount++;
                    }
                }
            }

            // Save image with annotations
            if(debugMode) api.saveMatImage(imgRGB, "recognitionTesting" + target_num + ".png");

            api.setAreaInfo(target_num, landmarkItemName, landmarkItemCount);
            Log.i("My Debug", "Landmark: " + landmarkItemName + ", Count: " + landmarkItemCount);

            Log.i("My Debug", "Treasure Item: " + treasureItem);
            return treasureItem;
        } catch (Exception e) {
            Log.e("My Debug", "Exception in detectObject: " + e.getMessage());
            return "";
        }
    }

    private static boolean isNearEdge(RectF rect, float imageWidth, float imageHeight, float margin_in_per) {
        float margin_width = (margin_in_per/100)*imageWidth;
        float margin_height = (margin_in_per/100)*imageHeight;
//        Log.i("My Debug", String.valueOf(rect.left) + " " + margin_width);
//        Log.i("My Debug", String.valueOf(rect.top) + " " + margin_height);
//        Log.i("My Debug", rect.right + " " + (imageWidth - margin_width));
//        Log.i("My Debug", rect.bottom + " " + (imageHeight - margin_height));

        return rect.left < margin_width || rect.top < margin_height || rect.right > imageWidth - margin_width || rect.bottom > imageHeight - margin_height;
    }

    private File getModelFile() {
        String modelName = "krpc-aug-small-yolo11m-batch4-v3.onnx";
        return convertModelFileFromAssetsToTempFile(modelName);
    }

    private File convertModelFileFromAssetsToTempFile(String modelFileName) {
        try {
            // Open the TFLite model file from the assets directory
            InputStream inputStream = getAssets().open(modelFileName);

            // Create a temporary file
            File tempFile = File.createTempFile(modelFileName, null);
            tempFile.deleteOnExit(); // Delete the temporary file when the JVM exits

            // Write the model data to the temporary file
            try (FileOutputStream outputStream = new FileOutputStream(tempFile)) {
                byte[] buffer = new byte[4 * 1024]; // 4K buffer
                int bytesRead;
                while ((bytesRead = inputStream.read(buffer)) != -1) {
                    outputStream.write(buffer, 0, bytesRead);
                }
            }

            return tempFile;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public String[] classNames = {"coin", "compass", "coral", "crystal", "diamond", "emerald", "fossil", "key", "letter", "shell", "treasure_box"};
    private static final List<String> treasureItems = Arrays.asList("crystal", "diamond", "emerald");
    private static final List<String> landmarkItems = Arrays.asList("treasure_box", "coin", "compass", "coral", "fossil", "key", "letter", "shell");


    public Mat drawBoundingBox(Mat in, int classId, double confidence, int x, int y, int x1, int y1) {
        String label = String.format("%s, %.2f", classNames[classId], confidence);
        Log.i("RecognitionDrawingBoundingBox", String.format("Label: %s, x: %d, y: %d", label, x, y));
        Rect box = new Rect(new Point(x, y), new Point(x1, y1));
        Imgproc.rectangle(in, box, new Scalar(255, 0, 0));
        Imgproc.putText(in, label, new Point(x - 10, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(255, 0, 0), 1);

        return in;
    }

    public Mat drawBoundingBox(Mat in, int classId, double confidence, Rect2d rect) {
        return drawBoundingBox(in, classId, confidence, (int) rect.x, (int) rect.y, (int) (rect.x + rect.width), (int)(rect.y + rect.height));
    }
}