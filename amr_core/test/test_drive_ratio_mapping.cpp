#include <gtest/gtest.h>

#include <limits>

#include "amr_core/utils/drive_ratio_mapping.hpp"

using Mapping = amr_core::DriveRatioMapping;
using MappingError = Mapping::Error;

static Mapping::Calibration calibration()
{
  return { 1200.0, 300.0, 60.0, 30.0, 24.0, 10.0, 5.0, 0.2 };
}

TEST(DriveRatioMapping, SeparatesAsymmetricCalibrationFromLocalLimits)
{
  Mapping mapping(calibration());
  const auto forward = mapping.map(0.6, 0.0, 100.0, {});
  const auto reverse = mapping.map(-0.15, 0.0, 100.0, {});
  EXPECT_EQ(forward.error, MappingError::None);
  EXPECT_EQ(reverse.error, MappingError::None);
  EXPECT_DOUBLE_EQ(forward.translation_pct, 50.0);
  EXPECT_DOUBLE_EQ(reverse.translation_pct, -50.0);
}

TEST(DriveRatioMapping, InvertsThrottleOnBothAxes)
{
  Mapping mapping(calibration());
  const auto result = mapping.map(0.3, 3.14159265358979323846 / 12.0, 50.0, { 0.6, 0.0, true });
  ASSERT_EQ(result.error, MappingError::None);
  EXPECT_DOUBLE_EQ(result.translation_pct, 50.0);
  EXPECT_NEAR(result.rotation_pct, 100.0 * 15.0 / 22.5, 1e-12);
  EXPECT_EQ(mapping.map(0.3, 0.0, 0.0, {}).error, MappingError::ZeroThrottle);
  EXPECT_EQ(mapping.map(0.0, 0.0, 0.0, {}).error, MappingError::None);
}

TEST(DriveRatioMapping, ReverseTurningUsesForwardSpeedDenominator)
{
  Mapping mapping(calibration());
  const auto result = mapping.map(-0.15, -3.14159265358979323846 / 12.0, 100.0, { -0.3, 0.0, true });
  ASSERT_EQ(result.error, MappingError::None);
  EXPECT_NEAR(result.rotation_pct, -100.0 * 15.0 / 51.0, 1e-12);
}

TEST(DriveRatioMapping, PreservesZerosAndDoesNotRequireUnusedFeedback)
{
  Mapping mapping(calibration());
  EXPECT_EQ(mapping.map(0.0, 0.0, 100.0, {}).error, MappingError::None);
  const auto spin = mapping.map(0.0, 3.14159265358979323846 / 6.0, 100.0, {});
  ASSERT_EQ(spin.error, MappingError::None);
  EXPECT_NEAR(spin.rotation_pct, 50.0, 1e-12);
  EXPECT_EQ(mapping.map(0.3, 0.0, 100.0, {}).error, MappingError::None);
}

TEST(DriveRatioMapping, RejectsDeadZonesWithoutBoostingSmallCommands)
{
  Mapping mapping(calibration());
  const auto small = mapping.map(0.01, 0.0, 100.0, {});
  EXPECT_EQ(small.error, MappingError::UnrepresentableTranslation);
  EXPECT_DOUBLE_EQ(small.translation_pct, 0.0);
  EXPECT_EQ(mapping.map(0.12, 0.0, 100.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(0.0, 0.001, 100.0, {}).error, MappingError::UnrepresentableRotation);
  auto no_dead_zones = calibration();
  no_dead_zones.translation_dead_zone_pct = 0.0;
  no_dead_zones.rotation_dead_zone_pct = 0.0;
  EXPECT_EQ(Mapping(no_dead_zones).map(0.001, 0.001, 100.0, { 0.0, 0.0, true }).error, MappingError::None);
}

TEST(DriveRatioMapping, RejectsSaturationInsteadOfClaimingRequestedVelocity)
{
  Mapping mapping(calibration());
  EXPECT_EQ(mapping.map(1.3, 0.0, 100.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(-0.4, 0.0, 100.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(0.7, 0.0, 50.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(0.0, 2.0, 100.0, {}).error, MappingError::UnrepresentableRotation);
}

TEST(DriveRatioMapping, RequiresFreshValidFeedbackOnlyForSpeedDependentTurns)
{
  Mapping mapping(calibration());
  EXPECT_EQ(mapping.map(0.3, 0.1, 100.0, {}).error, MappingError::StaleFeedback);
  EXPECT_EQ(mapping.map(0.3, 0.1, 100.0, { 0.1, 0.2, true }).error, MappingError::StaleFeedback);
  EXPECT_EQ(mapping.map(0.3, 0.1, 100.0, { 0.1, -0.1, true }).error, MappingError::StaleFeedback);
  EXPECT_EQ(mapping.map(0.3, 0.1, 100.0, { 2.0, 0.0, true }).error, MappingError::InvalidFeedback);
  EXPECT_EQ(mapping.map(0.3, 0.1, 100.0, { std::numeric_limits<double>::quiet_NaN(), 0.0, true }).error,
            MappingError::InvalidFeedback);
  auto constant_turning = calibration();
  constant_turning.rotation_forward_deg_s = constant_turning.rotation_stopped_deg_s;
  EXPECT_EQ(Mapping(constant_turning).map(0.3, 0.1, 100.0, {}).error, MappingError::None);
}

TEST(DriveRatioMapping, RejectsInvalidCalibrationAndCommands)
{
  auto invalid = calibration();
  invalid.forward_mm_s = 0.0;
  EXPECT_THROW(Mapping mapping(invalid), std::invalid_argument);
  invalid = calibration();
  invalid.rotation_dead_zone_pct = 100.0;
  EXPECT_THROW(Mapping mapping(invalid), std::invalid_argument);
  invalid = calibration();
  invalid.reverse_mm_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Mapping mapping(invalid), std::invalid_argument);
  Mapping mapping(calibration());
  EXPECT_EQ(mapping.map(std::numeric_limits<double>::quiet_NaN(), 0.0, 100.0, {}).error, MappingError::InvalidInput);
  EXPECT_EQ(mapping.map(0.0, 0.0, -1.0, {}).error, MappingError::InvalidInput);
  EXPECT_EQ(mapping.map(0.0, std::numeric_limits<double>::infinity(), 100.0, {}).error, MappingError::InvalidInput);
}

TEST(DriveRatioMapping, RejectsNonpositiveExtrapolatedTurningGain)
{
  auto config = calibration();
  config.forward_mm_s = 300.0;
  config.reverse_mm_s = 1200.0;
  Mapping mapping(config);
  const auto result = mapping.map(-0.6, 0.1, 100.0, { -1.2, 0.0, true });
  EXPECT_EQ(result.error, MappingError::InvalidRotationGain);
  EXPECT_DOUBLE_EQ(result.translation_pct, 0.0);
  EXPECT_DOUBLE_EQ(result.rotation_pct, 0.0);
}

TEST(DriveRatioMapping, RejectsTranslationRoundedBackIntoDeadZoneByWireFormat)
{
  Mapping mapping(calibration());
  EXPECT_EQ(mapping.map(0.120000001, 0.0, 100.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(-0.030000001, 0.0, 100.0, {}).error, MappingError::UnrepresentableTranslation);
  EXPECT_EQ(mapping.map(0.120001, 0.0, 100.0, {}).error, MappingError::None);
}

TEST(DriveRatioMapping, ReconstructsTargetsAcrossDirectionalThrottleAndSpeedGrid)
{
  auto config = calibration();
  config.translation_dead_zone_pct = 0.0;
  config.rotation_dead_zone_pct = 0.0;
  Mapping mapping(config);
  for (const auto throttle_pct : { 25.0, 50.0, 100.0 })
  {
    for (const auto linear : { -0.05, 0.0, 0.05 })
    {
      for (const auto angular : { -0.1, 0.0, 0.1 })
      {
        for (const auto measured : { -0.3, 0.0, 0.6, 1.2 })
        {
          const auto result = mapping.map(linear, angular, throttle_pct, { measured, 0.0, true });
          ASSERT_EQ(result.error, MappingError::None);
          const auto scale = throttle_pct / 100.0;
          const auto translation_gain = linear >= 0.0 ? config.forward_mm_s : config.reverse_mm_s;
          const auto moving_gain = linear > 0.0 ? config.rotation_forward_deg_s : config.rotation_reverse_deg_s;
          const auto rotation_gain = linear == 0.0 ? config.rotation_stopped_deg_s :
              config.rotation_stopped_deg_s + (moving_gain - config.rotation_stopped_deg_s) *
                  std::fabs(measured * 1000.0 / config.forward_mm_s);
          EXPECT_NEAR(result.translation_pct / 100.0 * translation_gain * scale / 1000.0, linear, 1e-12);
          EXPECT_NEAR(result.rotation_pct / 100.0 * rotation_gain * scale * 3.14159265358979323846 / 180.0,
                      angular, 1e-12);
        }
      }
    }
  }
}