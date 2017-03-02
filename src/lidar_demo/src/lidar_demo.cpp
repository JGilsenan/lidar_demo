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

void timerCallback(const ros::TimerEvent& e){

}

int numMatchesFound(std::vector<float> scanA, std::vector<float> scanB){
	int score = 0;
	for(int i = 0; i < scanA.size(); i++){
		float delta = scanA[i]-scanB[i];
		if(delta < 0) delta = delta * -1.000;
		if(delta <= matchDelta) score++;
	}
	return score;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan_msg scan;
	for(int i = 0; i < numSects; i++){
		if(isnan(msg->ranges[i]) || isinf(msg->ranges[i])){
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
		std::cout << "freq: " << freq.toSec() << std::endl;
		return;
	}

	while(scan.timeReceived - scans.front().timeReceived > maxCollectDur){
		scans.pop_front();
		queueFilled = true;
	}
	if(!queueFilled) return;


	int bestScore = 0;
	int bestIdx = 0;
	for(int i = 0; i < scans.size()-50; i++){
		int score = numMatchesFound(scan.scan, scans.at(i).scan);
		if(score > bestScore) {
			bestScore = score;
			bestIdx = i;
		}
	}
	ros::Duration delt = scan.timeReceived - scans.at(bestIdx).timeReceived;
	double refDeltDelt = referenceEstimate - delt.toSec();
	if(refDeltDelt < 0) refDeltDelt = refDeltDelt * -1.0;
	if(refDeltDelt > throwOutDelta) return;

	sampleDurations.push(delt.toSec());
	sampleSum += delt.toSec();

	if(sampleDurations.size() > sampleSizeMax){
		double frontVal = sampleDurations.front();
		sampleDurations.pop();
		sampleSum -= frontVal;
	}
	double avgRpm = sampleSum / sampleDurations.size();

	std::cout << "best match idx: " << bestIdx << std::endl;
	std::cout << "best match score: " << bestScore << std::endl;
	std::cout << "best match time delta: " << delt.toSec() << std::endl;
	std::cout << "avg rotation duration: " << avgRpm << std::endl;
	std::cout << "avg rotation sample size: " << sampleDurations.size() << std::endl;

}


void lidar_demo::readParamsAndSetup(ros::NodeHandle *n){

  if (n->hasParam("node_rate")){
    n->getParam("node_rate", node_rate);
    n->deleteParam("node_rate");
  } else {
    std::cout << "Parameter not found: node_rate \n";
  }

}


int main(int argc, char **argv){

  ros::init(argc, argv, "demo");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  lidar_demo::readParamsAndSetup(&n);

  ros::Subscriber laser_sub = n.subscribe("/scan", 1, laserScanCallback);

  ros::Timer timer = n.createTimer(ros::Duration(lidar_demo::node_rate/1000), timerCallback);

  ros::spin();

  return 0;
}