#include <aux_lib/aux_functions.h>
#include <aux_lib/struct_defs.h>

#include <aux_lib/clipper.hpp>

// Use Google Test
#include <gtest/gtest.h>

/**
 * @brief Test for overlap between rectangles
 */
TEST(ClipperWrapper, overlap) {
  ClipperLib::Clipper clipper;
  ClipperLib::Paths clip_union;
  ClipperLib::Paths clip_rect1(1);
  ClipperLib::Paths clip_rect2(1);
  bool overlap;

  {
    clip_rect1[0].clear();
    geometry::Point2D p_xy(0.0, 0.0);
    double width = 1.0, length = 1.0, theta = 0.0;
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect1[0].push_back(p_long);
      }
    }
  }

  /**
   * @test 1- No overlap origin
   */
  {
    clip_rect2[0].clear();
    geometry::Point2D p_xy(1.5, 0.0);
    double width = 1.0, length = 1.0, theta = 0.0;
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect2[0].push_back(p_long);
      }
    }
  }
  clipper.Clear();
  clip_union.clear();
  clipper.AddPaths(clip_rect1, ClipperLib::ptSubject, true);
  clipper.AddPaths(clip_rect2, ClipperLib::ptClip, true);
  clipper.Execute(ClipperLib::ctUnion, clip_union, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);
  overlap = clip_union.size() < 2;
  EXPECT_EQ(false, overlap) << "No overlap origin";

  /**
   * @test 2- Edge collision
   */
  {
    clip_rect2[0].clear();
    geometry::Point2D p_xy(1.0, 0.0);
    double width = 1.0, length = 1.0, theta = 0.0;
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect2[0].push_back(p_long);
      }
    }
  }
  clipper.Clear();
  clip_union.clear();
  clipper.AddPaths(clip_rect1, ClipperLib::ptSubject, true);
  clipper.AddPaths(clip_rect2, ClipperLib::ptClip, true);
  clipper.Execute(ClipperLib::ctUnion, clip_union, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);
  overlap = clip_union.size() < 2;
  EXPECT_EQ(true, overlap) << "Edge collision straight";

  /**
   * @test 3- No overlap diagonal
   */
  {
    clip_rect2[0].clear();
    double width = 3.0, length = 2.0, theta = M_PI / 4;
    double aux_val = width / 2.0 + 0.5;
    geometry::Point2D p_xy(0.5 + aux_val * sin(theta),
                           -0.5 - aux_val * cos(theta));
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect2[0].push_back(p_long);
      }
    }
  }
  clipper.Clear();
  clip_union.clear();
  clipper.AddPaths(clip_rect1, ClipperLib::ptSubject, true);
  clipper.AddPaths(clip_rect2, ClipperLib::ptClip, true);
  clipper.Execute(ClipperLib::ctUnion, clip_union, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);
  overlap = clip_union.size() < 2;
  EXPECT_EQ(false, overlap) << "No overlap diagonal";

  /**
   * @test 4- Point overlap diagonal
   */
  {
    clip_rect2[0].clear();
    double width = 2.0, length = 3.0, theta = M_PI / 4;
    geometry::Point2D p_xy(0.5 + width / 2.0 * sin(theta),
                           -0.5 - width / 2.0 * cos(theta));
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect2[0].push_back(p_long);
      }
    }
  }
  clipper.Clear();
  clip_union.clear();
  clipper.AddPaths(clip_rect1, ClipperLib::ptSubject, true);
  clipper.AddPaths(clip_rect2, ClipperLib::ptClip, true);
  clipper.Execute(ClipperLib::ctUnion, clip_union, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);

  overlap = clip_union.size() < 2;
  EXPECT_EQ(true, overlap) << "Edge collision diagonal";

  /**
   * @test 5- Same square
   */
  {
    clip_rect2[0].clear();
    double width = 1.0, length = 1.0, theta = 0.0;
    geometry::Point2D p_xy(0.0, 0.0);
    std::vector<geometry::Point2D> pts;
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, -width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, width / 2.0)));
    pts.push_back(geometry::localToGlobal(
        p_xy, theta, geometry::Point2D(-length / 2.0, -width / 2.0)));
    for (uint16_t k = 0; k < pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (ai4ad::to_cInt(pts[k].x, p_long.X) &&
          ai4ad::to_cInt(pts[k].y, p_long.Y)) {
        clip_rect2[0].push_back(p_long);
      }
    }
  }
  clipper.Clear();
  clip_union.clear();
  clipper.AddPaths(clip_rect1, ClipperLib::ptSubject, true);
  clipper.AddPaths(clip_rect2, ClipperLib::ptClip, true);
  clipper.Execute(ClipperLib::ctUnion, clip_union, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);

  overlap = clip_union.size() < 2;
  EXPECT_EQ(true, overlap) << "Same square";
}

// Run all the tests that were declared with
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
