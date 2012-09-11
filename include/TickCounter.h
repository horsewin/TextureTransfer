/*
 * TickCounter.h
 *
 *  Created on: 2012/09/11
 *      Author: umakatsu
 */

#ifndef TICKCOUNTER_H_
#define TICKCOUNTER_H_

#include <deque>
#include <iostream>
#include "OpenCV.h"

/////////////////////
// To calculate FPS//
/////////////////////
double previous_time = 0.0;
int    count_frame   = 0;
std::deque<double> fps;
double time_spent    = 0.0;
/////////////////////
namespace{
	double cal_mean() {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += fps[i];
		}
		sum /= fps.size();
		return sum;
	}

	double cal_std(double mean) {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += (fps[i] - mean) * (fps[i] - mean);
		}
		sum /= fps.size();
		return ( sqrt(sum) );
	}

	void TickCountAverageBegin()
	{
		previous_time = static_cast<double>(cv::getTickCount());
	}

	bool TickCountAverageEnd()
	{
		count_frame++;
		double current_time = static_cast<double>(cv::getTickCount());
		time_spent += ( current_time - previous_time ) / cv::getTickFrequency();
		if( count_frame == 1){ // you can change the frame count if you want
			if( fps.size() < 100){
				//fps.push_back(count_frame/time_spent);
				fps.push_back(1000*time_spent/count_frame);
			}else{
				fps.pop_front();
				fps.push_back(1000*time_spent/count_frame);
				//fps.push_back(count_frame/time_spent);
			}
			count_frame = 0;
			time_spent = 0.0;
			double mean = cal_mean();
			std::cout << "MEAN = " << mean << "ms  " << "SIGMA = " << cal_std(mean) << std::endl;
			std::cout << 1000/mean << std::endl;
			previous_time = current_time;
			return true;
		}
		return false;
	}
}



#endif /* TICKCOUNTER_H_ */
