#include <cmath>
#include <limits>

#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"
#include "lake_pose.h"

namespace {
namespace cc = cartographer::common;
namespace ct = cartographer::transform;
cc::Time at(int ms) { return cc::FromUniversal(123450000) + cc::FromMilliseconds(ms); }

TEST(Wt102, IntegratesPartialTrapezoidsAndExpires) {
  f1_mapping::HeadingHistory h;
  h.Add(at(0), 1, 0.);
  h.Add(at(10), 1, 2.);
  h.Add(at(20), 1, 4.);
  EXPECT_NEAR(*h.Delta(at(5), at(15)), .020, 1e-8);
  EXPECT_NEAR(*h.Delta(at(10), at(25)), .050, 1e-8);
  EXPECT_FALSE(h.Delta(at(-1), at(10)));
  EXPECT_FALSE(h.Delta(at(10), at(41)));
  h.Add(at(80), 1, 3.);
  EXPECT_FALSE(h.Delta(at(10), at(80)));
  h.Add(at(90), 1, -3.);
  EXPECT_NEAR(*h.Delta(at(80), at(90)), 0., 1e-9);
  h.Add(at(95), 2, 5.);
  EXPECT_FALSE(h.Delta(at(90), at(95)));
  h.Add(at(91), 2, 2.);
  EXPECT_FALSE(h.Delta(at(90), at(95)));
  h.Add(at(96), 2, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(h.Delta(at(91), at(96)));
}

TEST(Wt102, ActualCartographerPredictionUsesLastMatchedPoseAndFallsBack) {
  f1_mapping::HeadingHistory h;
  cartographer::mapping::PoseExtrapolator plain(cc::FromMilliseconds(1), 10.);
  cartographer::mapping::PoseExtrapolator aided(cc::FromMilliseconds(1), 10.);
  auto pose = [](double yaw) { return ct::Embed3D(ct::Rigid2d({1., 2.}, yaw)); };
  for (auto* ext : {&plain, &aided}) {
    ext->AddPose(at(0), pose(.7));
    ext->AddPose(at(20), pose(.7));
  }
  for (int ms = 10; ms <= 55; ms += 5) h.Add(at(ms), 7, 1.5);
  auto baseline = plain.ExtrapolatePose(at(40));
  f1_mapping::wt102_heading = &h;
  auto prediction = aided.ExtrapolatePose(at(40));
  EXPECT_NEAR(ct::GetYaw(prediction), .73, 1e-8);
  EXPECT_TRUE(prediction.translation().isApprox(baseline.translation()));
  EXPECT_NEAR(ct::GetYaw(baseline), .7, 1e-8);
  aided.AddPose(at(40), pose(-.4));
  EXPECT_NEAR(ct::GetYaw(aided.ExtrapolatePose(at(40))), -.4, 1e-8);
  EXPECT_NEAR(ct::GetYaw(aided.ExtrapolatePose(at(55))), -.3775, 1e-8);
  plain.AddPose(at(40), pose(-.4));
  auto fallback = aided.ExtrapolatePose(at(90));
  f1_mapping::wt102_heading = nullptr;
  EXPECT_NEAR(ct::GetYaw(fallback), ct::GetYaw(plain.ExtrapolatePose(at(90))), 1e-8);
}

TEST(Wt102, LongSegmentHasConstantHistoryAndDoesNotBridgeReconnect) {
  f1_mapping::HeadingHistory rates;
  f1_mapping::LakePose lake(rates);
  rates.Add(at(0), 1, .01);
  rates.Add(at(10), 1, .01);
  lake.Start(0, at(10), false);
  lake.Scan(at(10), ct::Embed3D(ct::Rigid2d({0., 0.}, .3)));
  for (int t = 20; t <= 1800010; t += 10) rates.Add(at(t), 1, .01);
  lake.Fix(at(1800010), {3., 400.}, 1.);
  auto pose = lake.Estimate(at(1800010));
  ASSERT_TRUE(pose);
  EXPECT_NEAR(std::remainder(ct::GetYaw(*pose) - 18.3, 2*M_PI), 0., 1e-8);
  EXPECT_DOUBLE_EQ(pose->translation().y(), 400.);
  EXPECT_FALSE(rates.Delta(at(10), at(1800010)));
  rates.Add(at(1800020), 2, .01);
  rates.Add(at(1800030), 2, .01);
  EXPECT_FALSE(lake.Estimate(at(1800030)));
  lake.Scan(at(1800030), ct::Rigid3d::Identity());
  EXPECT_TRUE(lake.Estimate(at(1800030)));
  lake.Start(1, at(1800030), true);
  EXPECT_FALSE(lake.Estimate(at(1800030)));
}
}  // namespace
