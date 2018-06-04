/*
 * SmoothRect.h
 *
 *  Created on: 21.06.2016
 *      Author: chris
 */

#ifndef OHM_PERCEPTION_OHM_MOTIONDETECTION_SRC_SMOOTHRECT_H_
#define OHM_PERCEPTION_OHM_MOTIONDETECTION_SRC_SMOOTHRECT_H_

#include <numeric>
#include <deque>

class SmoothRect
{
public:
   void addRect(ohm_perception_msgs::Rect input)
   {
      _u.push_back(input.u);
      _v.push_back(input.v);

      _width.push_back( input.width);
      _height.push_back(input.height);


      _smoothed = input;

      // calc mean value
      _smoothed.u       = std::accumulate(_u.begin(), _u.end(), 0)            / _u.size();
      _smoothed.v       = std::accumulate(_v.begin(), _v.end(), 0)            / _v.size();
      _smoothed.width   = std::accumulate(_width.begin(), _width.end(), 0)    / _width.size();
      _smoothed.height  = std::accumulate(_height.begin(), _height .end(), 0) / _height .size();

      if(_u.size() > 10) {
         _u.pop_front();
         _v.pop_front();
         _width.pop_front();
         _height.pop_front();
      }



   }


   ohm_perception_msgs::Rect getSmoothedRect(void) const
   {
      return _smoothed;
   }




private:
  // ohm_perception_msgs::Rect _smoothed;

   std::deque<int>  _u;
   std::deque<int>  _v;
   std::deque<int>  _width;
   std::deque<int>  _height;
};

namespace smoothing{
   static inline
   ohm_perception_msgs::Rect smooth(ohm_perception_msgs::Rect input){
      SmoothRect sr;
      sr.addRect(input);
      return sr.getSmoothedRect();
   }
}

#endif /* OHM_PERCEPTION_OHM_MOTIONDETECTION_SRC_SMOOTHRECT_H_ */
