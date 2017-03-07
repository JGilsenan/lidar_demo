/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Joe Gilsenan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "lidar_demo/lidar_demo.h"
#include "sensor_msgs/LaserScan.h"
#include <list>
#include <queue>
#include <deque>
#include <cstdlib>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <math.h>

static int numSects = 360;
static float matchDelta = 0.05;
ros::Duration maxCollectDur(12.0);
static double referenceEstimate = 9.0;
static double throwOutDelta = 1.0;
static int sampleSizeMax = 200;

struct scan_msg {
	ros::Time timeReceived;
	std::vector<float> scan;
};

std::deque<scan_msg> scans;
ros::Duration freq;

std::queue<double> sampleDurations;
double sampleSum = 0;

bool freqKnown = false;
bool queueFilled = false;


static int countNeeded = 5;
int currentCount = 0;
bool indexSet = false;
scan_msg bestIndexScan;
int bestMatchScore = 0;

ros::Duration rotationTimeEstimate;
double thetaEst = 0;


int numMatchesFound(std::vector<float> scanA, std::vector<float> scanB){
	int score = 0;
	for(int i = 0; i < scanA.size(); i++){
		if(std::fabs(scanA[i]-scanB[i]) <= matchDelta) score++;
	}
	return score;
}

int getBestMatchIndex(const scan_msg& scan) {
	int bestScore = 0;
	int bestIdx = 0;
	for (int i = 0; i < scans.size() - 50; i++) {
		int score = numMatchesFound(scan.scan, scans.at(i).scan);
		if (score > bestScore) {
			bestScore = score;
			bestIdx = i;
		}
	}

	if(queueFilled && !indexSet) {
		if(bestScore > bestMatchScore) {
			std::cout << "possible index scan found!\n";
			bestIndexScan = scan;
			bestMatchScore = bestScore;
			currentCount = 0;
		} else {
			currentCount++;
			if (currentCount >= countNeeded) {
				indexSet = true;
			}
		}
	}

	return bestIdx;
}

bool isMatchTimeDeltaOutOfRange(const ros::Duration& bestMatchTimeDelta) {
	bool deltaOutOfRange = false;
	double refDeltDelt = referenceEstimate - bestMatchTimeDelta.toSec();
	if (refDeltDelt < 0)
		refDeltDelt = refDeltDelt * -1.0;

	if (refDeltDelt > throwOutDelta)
		deltaOutOfRange = true;

	return deltaOutOfRange;
}

double updateRotationTimeEstimate(const ros::Duration& bestMatchTimeDelta) {
	sampleDurations.push(bestMatchTimeDelta.toSec());
	sampleSum += bestMatchTimeDelta.toSec();
	if (sampleDurations.size() > sampleSizeMax) {
		double frontVal = sampleDurations.front();
		sampleDurations.pop();
		sampleSum -= frontVal;
	}
	return sampleSum / sampleDurations.size();
}

void cleanupQueue(const scan_msg& scan) {
	queueFilled = false;
	while (scan.timeReceived - scans.front().timeReceived > maxCollectDur) {
		scans.pop_front();
		queueFilled = true;
	}
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan_msg scan;
	for(int i = 0; i < numSects; i++){
		if(std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])){
			scan.scan.push_back(0);
		} else {
			scan.scan.push_back(msg->ranges[i]);
		}
	}
	scan.timeReceived = ros::Time::now();
	scans.push_back(scan);

	if(!freqKnown && scans.size() > 1){
		freqKnown = true;
		freq = scan.timeReceived - scans.front().timeReceived;
	}

	cleanupQueue(scan);
	if(!queueFilled) return;

	int bestIdx = getBestMatchIndex(scan);
	ros::Duration bestMatchTimeDelta = (scan.timeReceived - scans.at(bestIdx).timeReceived);
	if(isMatchTimeDeltaOutOfRange(bestMatchTimeDelta)) return;


	double dRotationTimeEstimate = updateRotationTimeEstimate(bestMatchTimeDelta);
	rotationTimeEstimate = ros::Duration(dRotationTimeEstimate);

	std::cout << "estimated rotation duration: " << rotationTimeEstimate.toSec() << std::endl;

	if(indexSet) {
		double delt = (ros::Time::now() - bestIndexScan.timeReceived).toSec();
		while(delt >= rotationTimeEstimate.toSec()){
			delt -= rotationTimeEstimate.toSec();
		}
		thetaEst = (delt/rotationTimeEstimate.toSec())*(2*M_PI);
		std::cout << "est time since index: " << delt << std::endl;
		std::cout << "est theta since index: " << thetaEst << std::endl;
	}
}


double yawTemp = 0;
void timerCallback(const ros::TimerEvent& e){

	tf::TransformBroadcaster baseToLaser;
	tf::TransformBroadcaster worldToBase;

	baseToLaser.sendTransform(
			tf::StampedTransform(
					tf::Transform(tf::createQuaternionFromRPY(0,0,M_PI/2),tf::Vector3(0,0,0)),
							ros::Time::now(),
							"base_link",
							"laser"));

	worldToBase.sendTransform(
			tf::StampedTransform(
					tf::Transform(tf::createQuaternionFromYaw(yawTemp),tf::Vector3(0,0,0)),
							ros::Time::now(),
							"world",
							"base_link"));

	double adj = (lidar_demo::node_rate/1000.0)*(2*M_PI);
	yawTemp += adj;
	if(yawTemp > (2*M_PI)){
		yawTemp = 0;
	}
	std::cout << "Test yaw: " << yawTemp << std::endl;

}

void lidar_demo::readParamsAndSetup(ros::NodeHandle *n){
  if (n->hasParam("node_rate")){
    n->getParam("node_rate", node_rate);
    n->deleteParam("node_rate");
    std::cout << "Parameter read: node_rate \n";
  } else {
    std::cout << "Parameter not found: node_rate \n";
  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "demo");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

//  lidar_demo::readParamsAndSetup(&n);
//  ros::Subscriber laser_sub = n.subscribe("/scan", 1, laserScanCallback);
//  ros::Timer timer = n.createTimer(ros::Duration(lidar_demo::node_rate/1000.0), timerCallback);
//  ros::spin();

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while(n.ok()){
      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
          ros::Time::now(),"world", "base_link"));
      r.sleep();
    }

  return 0;
}
