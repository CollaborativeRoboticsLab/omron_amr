#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace amr_core
{

class DriveRatioMapping
{
public:
  struct Calibration
  {
    double forward_mm_s;
    double reverse_mm_s;
    double rotation_stopped_deg_s;
    double rotation_forward_deg_s;
    double rotation_reverse_deg_s;
    double translation_dead_zone_pct;
    double rotation_dead_zone_pct;
    double feedback_timeout_sec;
  };

  struct Feedback
  {
    double linear_mps{ 0.0 };
    double age_sec{ 0.0 };
    bool valid{ false };
  };

  enum class Error
  {
    None,
    InvalidInput,
    ZeroThrottle,
    StaleFeedback,
    InvalidFeedback,
    InvalidRotationGain,
    UnrepresentableTranslation,
    UnrepresentableRotation
  };

  struct Result
  {
    double translation_pct{ 0.0 };
    double rotation_pct{ 0.0 };
    Error error{ Error::None };
  };

  explicit DriveRatioMapping(Calibration calibration) : calibration_(calibration)
  {
    for (const auto value :
         { calibration.forward_mm_s, calibration.reverse_mm_s, calibration.rotation_stopped_deg_s,
           calibration.rotation_forward_deg_s, calibration.rotation_reverse_deg_s, calibration.feedback_timeout_sec })
    {
      if (!std::isfinite(value) || value <= 0.0)
      {
        throw std::invalid_argument("Resolved ratio calibration speeds and feedback timeout must be positive");
      }
    }
    for (const auto value : { calibration.translation_dead_zone_pct, calibration.rotation_dead_zone_pct })
    {
      if (!std::isfinite(value) || value < 0.0 || value >= 100.0)
      {
        throw std::invalid_argument("Ratio dead zones must be finite percentages in [0, 100)");
      }
    }
  }

  Result map(double linear_mps, double angular_rad_s, double throttle_pct, Feedback feedback) const
  {
    if (!std::isfinite(linear_mps) || !std::isfinite(angular_rad_s) || !std::isfinite(throttle_pct) ||
        throttle_pct < 0.0 || throttle_pct > 100.0)
    {
      return failure(Error::InvalidInput);
    }
    if (linear_mps == 0.0 && angular_rad_s == 0.0)
    {
      return {};
    }
    if (throttle_pct == 0.0)
    {
      return failure(Error::ZeroThrottle);
    }
    const auto throttle = throttle_pct / 100.0;
    const auto full_translation = linear_mps >= 0.0 ? calibration_.forward_mm_s : calibration_.reverse_mm_s;
    const auto translation = (linear_mps / full_translation) * 1000.0 / throttle * 100.0;
    if (!std::isfinite(translation) || std::fabs(translation) > 100.0 ||
        (linear_mps != 0.0 && std::fabs(wireRatio(translation)) <= calibration_.translation_dead_zone_pct))
    {
      return failure(Error::UnrepresentableTranslation);
    }

    auto rotation_gain = calibration_.rotation_stopped_deg_s;
    const auto moving_rotation_gain =
        linear_mps > 0.0 ? calibration_.rotation_forward_deg_s : calibration_.rotation_reverse_deg_s;
    if (linear_mps != 0.0 && angular_rad_s != 0.0 && moving_rotation_gain != rotation_gain)
    {
      if (!feedback.valid || !std::isfinite(feedback.age_sec) || feedback.age_sec < 0.0 ||
          feedback.age_sec >= calibration_.feedback_timeout_sec)
      {
        return failure(Error::StaleFeedback);
      }
      if (!std::isfinite(feedback.linear_mps) ||
          std::fabs(feedback.linear_mps) > std::max(calibration_.forward_mm_s, calibration_.reverse_mm_s) / 1000.0)
      {
        return failure(Error::InvalidFeedback);
      }
      const auto speed_ratio = std::fabs(feedback.linear_mps) / calibration_.forward_mm_s * 1000.0;
      rotation_gain += (moving_rotation_gain - rotation_gain) * speed_ratio;
    }
    if (!std::isfinite(rotation_gain) || rotation_gain <= 0.0)
    {
      return failure(Error::InvalidRotationGain);
    }
    constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
    const auto rotation = angular_rad_s / rotation_gain * radians_to_degrees / throttle * 100.0;
    if (!std::isfinite(rotation) || std::fabs(rotation) > 100.0 ||
        (angular_rad_s != 0.0 && std::fabs(wireRatio(rotation)) < calibration_.rotation_dead_zone_pct))
    {
      return failure(Error::UnrepresentableRotation);
    }
    return { translation, rotation, Error::None };
  }

  static const char* describe(Error error)
  {
    switch (error)
    {
      case Error::None:
        return "representable";
      case Error::InvalidInput:
        return "non-finite command or invalid throttle";
      case Error::ZeroThrottle:
        return "nonzero target with zero throttle";
      case Error::StaleFeedback:
        return "missing or stale body-speed feedback";
      case Error::InvalidFeedback:
        return "body-speed feedback outside the calibrated range";
      case Error::InvalidRotationGain:
        return "nonpositive or invalid calibrated turning gain";
      case Error::UnrepresentableTranslation:
        return "translation outside calibrated range or inside dead zone";
      case Error::UnrepresentableRotation:
        return "rotation outside calibrated range or inside dead zone";
    }
    return "unknown mapping error";
  }

private:
  static double wireRatio(double ratio)
  {
    char encoded[64];
    std::snprintf(encoded, sizeof(encoded), "%g", ratio);
    return std::strtod(encoded, nullptr);
  }

  static Result failure(Error error)
  {
    return { 0.0, 0.0, error };
  }

  Calibration calibration_;
};

}  // namespace amr_core