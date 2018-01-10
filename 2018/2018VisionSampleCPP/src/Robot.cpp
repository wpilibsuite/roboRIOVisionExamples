/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "GripPipeline.h"

#include <iostream>
#include <cmath>
#include <string>

#include <opencv2/core.hpp>

#include <CameraServer.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <vision/VisionRunner.h>

class Robot : public frc::IterativeRobot {
public:
	/**
	 * Helper class to compute the outer dimensions of the rectangle that contains 2 OpenCV Rects
	 * This has been placed inside the robot class for simplicity in this example, but should likely
	 * be made it's own class file in a real project.
	 */
	class boundingRect
	{
		public:
			int top;
			int bottom;
			int left;
			int right;

		boundingRect (cv::Rect rectangle1, cv::Rect rectangle2)
		{
			top = std::max(rectangle1.y, rectangle2.y);
			bottom = std::max(rectangle1.y + rectangle1.height, rectangle2.y + rectangle2.height);
			left = std::max(rectangle1.x, rectangle2.x);
			right = std::max(rectangle1.x + rectangle1.width, rectangle2.x + rectangle2.width);
		}

		boundingRect()
		{
			top = bottom = left = right = 0;
		}
	};

	frc::VisionRunner<grip::GripPipeline>* m_vision;
	std::thread* m_visionThread;
	std::mutex* m_lock;
	double centerX = 0.0;
	double distance = 0.0;

	void RobotInit() {
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);

		m_vision = new frc::VisionRunner<grip::GripPipeline>(
						camera,
						new grip::GripPipeline(),
						[&](grip::GripPipeline& pipeline)
						{
							//This code is called each time the pipeline completes. Here we process the results of the pipeline

							//If we have at least 2 contours, we might have a target
							if (pipeline.GetFilterContoursOutput()->size() > 1)
							{
								double highScore = 0;
								boundingRect* target;
								target = new boundingRect();

								//Iterate through list of found contours
								for(int i=0; i < pipeline.GetFilterContoursOutput()->size(); i++)
								{
									cv::Rect rectangle1 = cv::boundingRect(cv::Mat(pipeline.GetFilterContoursOutput()[i]));

									//For each contour, iterate through the list of remaining contours to try all pairs
									for(int j=i+1; j < pipeline.GetFilterContoursOutput()->size(); j++)
									{
										cv::Rect rectangle2 = cv::boundingRect(cv::Mat(pipeline.GetFilterContoursOutput()[j]));

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
								double viewHeight = TARGET_HEIGHT * IMG_HEIGHT / (target->bottom - target->top);
								double localDistance = .5*viewHeight/std::tan(CAMERA_FOV_VERT*M_PI/(2*180));

								//Save off the center of the target and distance for use in auto/teleop code
								m_lock->lock();
								centerX = (target->left+target->right)/2;
								distance = localDistance;
								m_lock->unlock();
							}
						});

		m_lock = new std::mutex();
		m_visionThread = new std::thread(&Robot::visionExecuter, this);
	}

	void visionExecuter()
	{
		m_vision->RunForever();
	}

	void AutonomousPeriodic() {
		m_lock->lock();
		std::cout << "Center: " << centerX << std::endl;
		std::cout << "Distance: " << distance << std::endl;
		m_lock->unlock();
	}

private:
	const int IMG_WIDTH = 320;
	const int IMG_HEIGHT = 240;
	const double TARGET_HEIGHT = 15.3;	//The height of the target in inches
	const double CAMERA_FOV_VERT = 41;  //The camera vertical field of view in degrees. This is the number for the MS Lifecam
	// "Average" score of 75 needed to be seen as target. Note that the LV example also checks that no single score is under 15
	const int SCORE_THRESHOLD = 75 * 6;

	//The height of the bounding box around both rectangles should be approximately double the width
	double boundingRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

		return ratioToScore((bounding->top-bounding->bottom)/(2*(bounding->left-bounding->right)));
	}

	//The width of either contour should be approximately 1/4 of the total bounding box width
	double contourWidthScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

		return ratioToScore(rectangle1.width*4/(bounding->right-bounding->left));
	}

	//The top edges should be very close together. Find the difference, then scale it by the bounding box height.
	//This results in an ideal 0 instead of an ideal 1, so add 1
	double topEdgeScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

		return ratioToScore(1 + (rectangle1.y - rectangle2.y)/(bounding->top-bounding->bottom));
	}

	//The spacing between the left edges should be 3/4 of the target width
	double leftSpacingScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

		return ratioToScore(std::abs(rectangle2.x - rectangle1.x)*3/(4*bounding->right-bounding->left));
	}

	//The width of the two contours should match
	double widthRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		return ratioToScore(rectangle1.width/rectangle2.width);
	}

	//The height of the two contours should match
	double heightRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
	{
		return ratioToScore(rectangle1.height/rectangle2.height);
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (std::max(0.0, std::min(100*(1-std::abs(1-ratio)), 100.0)));
	}


};

START_ROBOT_CLASS(Robot)
