/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1.robot;

import org.usfirst.frc.team1.robot.GripPipeline;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

/**
 * This example shows a stripped down robot program with vision processing for a 2018 vision target using
 * a GRIP Pipeline. The code shows how to use the GRIP Pipeline to do the primary processing, then check 
 * the resulting contours for a target that matches the parameters of the 2018 Vision Target.
 * This code has been stripped of many of the typical init and periodic methods from the usual robot code
 * and only includes robotInit (to set up the camera and pipeline) and autoPeriodic (to print values)
 */

public class Robot extends IterativeRobot {
	
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private static final double TARGET_HEIGHT = 15.3;	//The height of the target in inches
	private static final double CAMERA_FOV_VERT = 41;  //The camera vertical field of view in degrees. This is the number for the MS Lifecam
	// "Average" score of 75 needed to be seen as target. Note that the LV example also checks that no single score is under 15
	private static final int SCORE_THRESHOLD = 75 * 6;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private double distance = 0.0;
	
	private final Object imgLock = new Object();
	
	/**
	 * Helper class to compute the outer dimensions of the rectangle that contains 2 OpenCV Rects
	 * This has been placed inside the robot class for simplicity in this example, but should likely 
	 * be made it's own class file in a real project.
	 */
	public class boundingRect
	{
		public int top;
		public int bottom;
		public int left;
		public int right;
		
		public boundingRect (Rect rectangle1, Rect rectangle2)
		{
			top = Math.max(rectangle1.y, rectangle2.y);
			bottom = Math.max(rectangle1.y + rectangle1.height, rectangle2.y + rectangle2.height);
			left = Math.max(rectangle1.x, rectangle2.x);
			right = Math.max(rectangle1.x + rectangle1.width, rectangle2.x + rectangle2.width);
		}
		
		public boundingRect()
		{
			top = bottom = left = right = 0;
		}
	}

	@Override
	public void robotInit() {
	    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	    	//This code is called each time the pipeline completes. Here we process the results of the pipeline
	    	
	    	//If we have at least 2 contours, we might have a target
	        if (pipeline.filterContoursOutput().size() > 1) 
	        {
	        	double highScore = 0;
	        	boundingRect target = new boundingRect();
	        	
	        	//Iterate through list of found contours
	        	for(int i=0; i < pipeline.filterContoursOutput().size(); i++)
	        	{
	        		Rect rectangle1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
	        		
	        		//For each contour, iterate through the list of remaining contours to try all pairs
	        		for(int j=i+1; j < pipeline.filterContoursOutput().size(); j++)
	        		{
	        			Rect rectangle2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(j));
	        			
	        			//Calculate a total score across all 6 measurements
	        			double scoreTotal = 0;
	        			scoreTotal += boundingRatioScore(rectangle1, rectangle2);
	        			scoreTotal += contourWidthScore(rectangle1, rectangle2);
	        			scoreTotal += topEdgeScore(rectangle1, rectangle2);
	        			scoreTotal += leftSpacingScore(rectangle1, rectangle2);
	        			scoreTotal += widthRatioScore(rectangle1, rectangle2);
	        			scoreTotal += heightRatioScore(rectangle1, rectangle2);
	        			
	        			//If the score is the highest found so far, and is above the threshold, mark it as the target
	        			if (scoreTotal > highScore && scoreTotal > SCORE_THRESHOLD)
	        			{
	        				highScore = scoreTotal;
	        				target = new boundingRect(rectangle1, rectangle2);
	        			}
	        		}
	        	}
	        	
	        	/**  
	        	 * The ratio of the target height in inches/target height in pixels = The same ratio for the full camera view
	        	 * Solving this equation for the full camera view height in inches gives us the view height formula below.
	        	 * We can then draw a right triangle with the height being 1/2 * viewHeight, the length being distance, and the angle
	        	 * being 1/2 * the vertical FOV of the camera. We can then use trigonometry to come up with the equation for distance below
	        	 */
        		double viewHeight = TARGET_HEIGHT * IMG_HEIGHT / (target.bottom - target.top);
        		double localDistance = .5*viewHeight/Math.tan(Math.toRadians(CAMERA_FOV_VERT/2));

	        	//Save off the center of the target and distance for use in auto/teleop code
	            synchronized (imgLock) {
	                centerX = (target.left+target.right)/2;
	                distance = localDistance;
	            }
	        }
	    });
	    visionThread.start();
	}
	
	
	//The typical autonomous periodic method used in the Iterative or Timed Robot classes
	@Override
	public void autonomousPeriodic()
	{
		synchronized(imgLock)
		{
			//Use the distance and/or center measurements here. Check to make sure they are not 0 to make sure you have a target
			System.out.println("Center: " + centerX);
			System.out.println("Distance: " + distance);
		}
	}
	
	//The height of the bounding box around both rectangles should be approximately double the width
	double boundingRatioScore(Rect rectangle1, Rect rectangle2)
	{
		boundingRect bounding = new boundingRect(rectangle1, rectangle2);
		
		return ratioToScore((bounding.top-bounding.bottom)/(2*(bounding.left-bounding.right)));
	}
	
	//The width of either contour should be approximately 1/4 of the total bounding box width
	double contourWidthScore(Rect rectangle1, Rect rectangle2)
	{
		boundingRect bounding = new boundingRect(rectangle1, rectangle2);
		
		return ratioToScore(rectangle1.width*4/(bounding.right-bounding.left));
	}
	
	//The top edges should be very close together. Find the difference, then scale it by the bounding box height. 
	//This results in an ideal 0 instead of an ideal 1, so add 1
	double topEdgeScore(Rect rectangle1, Rect rectangle2)
	{
		boundingRect bounding = new boundingRect(rectangle1, rectangle2);
		
		return ratioToScore(1 + (rectangle1.y - rectangle2.y)/(bounding.top-bounding.bottom));
	}
	
	//The spacing between the left edges should be 3/4 of the target width
	double leftSpacingScore(Rect rectangle1, Rect rectangle2)
	{
		boundingRect bounding = new boundingRect(rectangle1, rectangle2);
		
		return ratioToScore(Math.abs(rectangle2.x - rectangle1.x)*3/(4*bounding.right-bounding.left));
	}
	
	//The width of the two contours should match
	double widthRatioScore(Rect rectangle1, Rect rectangle2)
	{
		return ratioToScore(rectangle1.width/rectangle2.width);
	}
	
	//The height of the two contours should match
	double heightRatioScore(Rect rectangle1, Rect rectangle2)
	{
		return ratioToScore(rectangle1.height/rectangle2.height);
	}
	
	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}
}
