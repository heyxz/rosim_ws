// Tool-frame generation along an edge polyline: parallel-transport TANGENT
// frames (no Frenet flips on straight seams), FIXED orientation, and the
// welding lead-angle tilt used to lift the torch body off the workpiece.

#include "rosim_core/edge_frame_generator.h"

#include <stdexcept>

namespace rosim::core
{

Eigen::Vector3d EdgeFrameGenerator::tangentAt(const std::vector<Eigen::Vector3d>& polyline,
                                              size_t i)
{
  if (polyline.size() < 2)
    return Eigen::Vector3d::UnitZ();

  Eigen::Vector3d t;
  if (i == 0)
    t = polyline[1] - polyline[0];
  else if (i + 1 == polyline.size())
    t = polyline[i] - polyline[i - 1];
  else
    t = polyline[i + 1] - polyline[i - 1];

  if (t.norm() < 1e-12)
    return Eigen::Vector3d::UnitZ();
  return t.normalized();
}

Eigen::Isometry3d EdgeFrameGenerator::frameAt(const Eigen::Vector3d& position,
                                              const Eigen::Vector3d& tangent,
                                              const Eigen::Vector3d& ref_up)
{
  const Eigen::Vector3d z_axis = tangent.normalized();

  // Project the reference onto the plane normal to the tangent.
  Eigen::Vector3d ref = ref_up;
  if (std::abs(ref.dot(z_axis)) > 0.99)
    ref = Eigen::Vector3d::UnitX();  // fallback when ref is (anti)parallel to tangent
  Eigen::Vector3d x_axis = ref - z_axis * ref.dot(z_axis);
  x_axis.normalize();

  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
  frame.translation() = position;
  frame.linear().col(0) = x_axis;
  frame.linear().col(1) = y_axis;
  frame.linear().col(2) = z_axis;
  return frame;
}

std::vector<Eigen::Isometry3d> EdgeFrameGenerator::generate(
    const std::vector<Eigen::Vector3d>& polyline, const Config& config)
{
  std::vector<Eigen::Isometry3d> frames;
  if (polyline.size() < 2)
    return frames;

  switch (config.orientation_mode)
  {
    case ORIENTATION_FIXED:
    {
      frames.reserve(polyline.size());
      Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
      frame.linear() = config.fixed_orientation.normalized().toRotationMatrix();
      for (const auto& p : polyline)
      {
        frame.translation() = p;
        frames.push_back(frame);
      }
      break;
    }
    case ORIENTATION_TANGENT:
    default:
    {
      frames.reserve(polyline.size());
      const Eigen::AngleAxisd tilt(config.tangent_tilt, Eigen::Vector3d::UnitY());
      for (size_t i = 0; i < polyline.size(); ++i)
      {
        // NOTE: ref_up is constant (parallel-transport-like projection, not true
        // transport). A constant reference keeps the frame stable and flip-free
        // for the typical use case; true transport accumulates drift on curved edges.
        Eigen::Isometry3d frame = frameAt(polyline[i], tangentAt(polyline, i), config.ref_up);
        // Lead angle: rotate tool Z (local +Y-rotation) toward the frame up-axis
        // (+X, the projected ref_up). With 0 tilt the tool axis lies exactly on
        // the edge line and a straight torch body sweeps the workpiece; the tilt
        // keeps the tip on the edge while lifting the trailing body clear.
        if (config.tangent_tilt != 0.0)
          frame.linear() = frame.linear() * tilt.toRotationMatrix();
        frames.push_back(frame);
      }
      break;
    }
  }
  return frames;
}

}  // namespace rosim::core
