#ifndef __PWM_CONVERTER__H__
#define __PWM_CONVERTER__H__

#include <boost/algorithm/clamp.hpp>

class PWMConverter
{
  public:
    PWMConverter(double jnt_min, double jnt_max, double pwm_min, double pwm_max, double pwm_center, int direction)
      : jnt_min_(jnt_min)
      , jnt_max_(jnt_max)
      , pwm_min_(pwm_min)
      , pwm_max_(pwm_max)
      , pwm_center_(pwm_center)
      , direction_(direction)
    {
        // multiplier: arg_max(abs(jnt_min), jnt_max), pwm_max(or pwm_min) - center / jnt_max (or jnt_min)
        bool choose_min = std::abs(jnt_min) > jnt_max;
        if(choose_min)
          multiplier_ = (pwm_min + pwm_center)/std::abs(jnt_min);
        else
          multiplier_ = (pwm_max + pwm_center)/jnt_max;
//        ROS_WARN_STREAM("Added " << vel_joint_names[i] << " with multiplier and center of "
//                        << multipliers_[i] << ", " << centers_[i]);

        multiplier_ *= direction;
    }

    double operator()(double jnt_value)
    {


     // ROS_WARN_STREAM(jnt_value);
      jnt_value = boost::algorithm::clamp(jnt_value, jnt_min_, jnt_max_);

      const double converted = jnt_value * multiplier_ + pwm_center_;
      //ROS_WARN(converted);
      return boost::algorithm::clamp(converted, pwm_min_, pwm_max_);
    }

    double center()
    {
      return pwm_center_;
    }

    int getDirection(){ return direction_; }

  private:
    double jnt_min_;
    double jnt_max_;
    double pwm_min_;
    double pwm_max_;
    double pwm_center_;
    int direction_;

    double multiplier_;
};

#endif
