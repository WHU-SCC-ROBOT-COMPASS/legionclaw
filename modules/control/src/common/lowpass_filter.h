/**
* The First Order Digital Lowpass Filter class.
* For IMU digital data,filter high frequency data.
* @author: luoxiaoliang@indrv.cn;
           yuanmu@indrv.cn;
* @date: 2018.12.28
*/

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <cmath>
#include <fstream>

class lowpass_filter {
 public:
  /*
      @parma q:time constant related to cut-off frequency
          f=q/(2*pi*SampleTime)
      @parma filtered_data_origin:origin output data
  */
  lowpass_filter(double q, double filtered_data_origin);
  virtual ~lowpass_filter();

  /*
      @parma sample_data:sample data
      @output :filtered data
  */
  double lowpass_filter_update(double sample_data);

 protected:
 private:
  // time constant related to cut-off frequency
  double q_;
  // previous filter output data value
  double previous_filter_ouput;
};

#endif  // LOWPASS_FILTER_H
