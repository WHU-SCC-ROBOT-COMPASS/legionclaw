#include "lowpass_filter.h"

/*
   @parma q:time constant related to cut-off frequency
       f=q/(2*pi*SampleTime)
   @parma filtered_data_origin:origin output data
*/
lowpass_filter::lowpass_filter(double q, double filtered_data_origin) {
  // ctor
  q_ = q;
  previous_filter_ouput = filtered_data_origin;
}

lowpass_filter::~lowpass_filter() {
  // dtor
}

/*
    @parma sample_data:sample data
    @output :filtered data
*/
double lowpass_filter::lowpass_filter_update(double sample_data) {
  // Z transform state differential equation
  // Y(n) = q*X(n)+(1-q)*Y(n-1)
  double filtered_ouput = q_ * sample_data + (1 - q_) * previous_filter_ouput;
  previous_filter_ouput = filtered_ouput;
  return filtered_ouput;
}
