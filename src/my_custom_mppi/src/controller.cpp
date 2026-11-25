#include "kernel.hpp"
#include "motion.h"
#include "seed_stream.hpp"
#include "reference.hpp"
#include <cstring>
#include <map>
#include <mutex>
#include <random>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_map.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace my_custom_mppi
{
using Pose = geometry_msgs::msg::PoseStamped;
using Command = geometry_msgs::msg::TwistStamped;
using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;
using Node = rclcpp_lifecycle::LifecycleNode;
using Result = rcl_interfaces::msg::SetParametersResult;
using Params = std::map<std::string, rclcpp::Parameter>;
static const char * critics[] =
{"collision", "obstacle", "obstacle_speed", "obstacle_repulsion", "tracking",
  "follow_path_lateral", "follow_path_angle", "turn", "acceleration", "angular_acceleration",
  "velocity",
  "costmap_speed", "recovery", "terminal"};

struct Settings
{
  Motion motion{};
  double dt, temperature, near, near_goal_limit_percent, look, rejoin;
  int samples, horizon, short_h;
  int64_t seed;
  bool show;
  std::array<double, 14> weights;
  explicit Settings(const Params & p)
  {
    auto d = [&](const char * k) {return p.at(k).as_double();};
    auto integer = [&](const char * k, int min) {
        auto value = p.at(k).as_int();
        if (value < min || value > INT_MAX) {throw std::invalid_argument(k);}
        return int(value);
      };
    for (const auto & [key, v]:p) {
      if (v.get_type() == rclcpp::PARAMETER_DOUBLE && !std::isfinite(v.as_double())) {
        throw std::invalid_argument(key);
      }
    }
    dt = d("dt");
    temperature = d("temperature");
    near = d("near_goal_distance");
    near_goal_limit_percent = d("near_goal_motion_limit_percent");
    if (near_goal_limit_percent < 0. || near_goal_limit_percent > 100.) {
      throw std::invalid_argument("near_goal_motion_limit_percent must be between 0 and 100");
    }
    look = d(
      "reference_lookahead");
    rejoin = d("rejoin_distance_percent");
    seed = p.at("seed").as_int();
    samples = integer(
      "samples",
      3);
    horizon = integer("horizon", 1);
    short_h = integer("near_goal_horizon", 1);
    show = p.at(
      "show_rollouts").as_bool();
    motion = {d("v_min"), d("v_max"), d("v_std"), d("a_max"), d("a_min"), d("w_min"), d("w_max"),
      d("w_std"), d("alpha_max"), d("alpha_min"), 0, 0};
    if (dt <= 0 || temperature <= 0 || near < 0 || look < 0 || rejoin < 5 || rejoin > 100 ||
      seed < 0 ||
      motion.v_min > 0 || motion.v_max < 0 || motion.w_min > 0 || motion.w_max < 0 ||
      motion.a_min > 0 ||
      motion.a_max < 0 || motion.alpha_min > 0 || motion.alpha_max < 0 || motion.v_std < 0 ||
      motion.w_std < 0 ||
      int64_t(samples) * horizon > INT_MAX / 3)
    {
      throw std::invalid_argument("Invalid MPPI limits, dimensions or timing");
    }
    for (size_t i = 0; i < weights.size(); ++i) {
      auto key = std::string("critics.") + critics[i];
      auto w = p.at(key + ".weight").as_double();
      if (w < 0) {throw std::invalid_argument(key + ".weight must be nonnegative");}
      weights[i] = p.at(key + ".enabled").as_bool() ? w : 0.;
    }
  }
};

class Controller : public nav2_core::Controller
{
  Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  rclcpp_lifecycle::LifecyclePublisher<Markers>::SharedPtr markers_;
  rclcpp_lifecycle::LifecyclePublisher<Marker>::SharedPtr rollouts_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  std::vector<std::string> keys_;
  std::string prefix_;
  nav_msgs::msg::Path plan_;
  Grid grid_;
  std::vector<unsigned char> raw_;
  std::vector<XY> points_, cached_;
  std::vector<double> path_, controls_, refs_, chosen_, displayed_;
  bool reset_ = true, percentage_ = false;
  double limit_ = 0., previous_dt_ = 0.;
  int64_t seed_ = -1;
  SeedStream rng_;
  std::mutex state_;
  Params current() const
  {
    Params p;
    for (auto & v:node_.lock()->get_parameters(keys_)) {
      p.emplace(v.get_name().substr(prefix_.size()), v);
    }
    return p;
  }
  static geometry_msgs::msg::Point point(double x, double y, double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }
  void visualize(
    const Pose & pose, int h, int status, int active, bool show, int count,
    double lookahead)
  {
    auto marker = [&](const char * name, int type, double size, float r, float g, float b) {
        Marker m;
        m.ns = name;
        m.header = pose.header;
        m.type = type;
        m.pose.orientation.w = 1;
        m.scale.x = size;
        m.scale.y = size;
        m.scale.z = size;
        m.color.a = 1;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.lifetime = rclcpp::Duration::from_seconds(.5);
        return m;
      };
    if (markers_->get_subscription_count()) {
      auto target = marker("lookahead", Marker::SPHERE, .12, 1, .2, .6);
      auto arc = marker("reference_arc", Marker::LINE_STRIP, .035, 1, .65, 0);
      auto selected = marker("selected", Marker::LINE_STRIP, .05, .1, 1, .2);
      auto radius = marker("lookahead_radius", Marker::LINE_STRIP, .015, .65, .25, 1.);
      for (int j = 0; j <= 64; ++j) {
        double theta = (2. * M_PI / 64.) * j;
        radius.points.push_back(
          point(
            pose.pose.position.x + lookahead * std::cos(theta),
            pose.pose.position.y + lookahead * std::sin(theta), .08));
      }
      target.pose.position = point(refs_[0], refs_[1], .20);
      arc.points.push_back(point(pose.pose.position.x, pose.pose.position.y, .10));
      selected.points.push_back(point(pose.pose.position.x, pose.pose.position.y, .14));
      for (int i = 0; i < h; ++i) {
        arc.points.push_back(point(refs_[2 * i], refs_[2 * i + 1], .10));
      }
      if (status == 1 || status == 2) {
        for (int i = 0; i < active; ++i) {
          selected.points.push_back(point(chosen_[3 * i], chosen_[3 * i + 1], .14));
        }
      } else {selected.action = Marker::DELETE;}
      Markers out;
      out.markers = {target, radius, arc, selected};
      markers_->publish(out);
    }
    if (rollouts_->get_subscription_count()) {
      auto m = marker("rollouts", Marker::LINE_LIST, .012, .1, .4, 1);
      m.color.a = .35;
      if (show && status != 0) {
        for (int k = 0; k < count; ++k) {
          for (int t = 1; t < h; ++t) {
            for (int i:{t - 1, t}) {
              auto j = 3 * (k * h + i);
              m.points.push_back(point(displayed_[j], displayed_[j + 1], .06));
            }
          }
        }
      } else {m.action = Marker::DELETE;}
      rollouts_->publish(m);
    }
  }

public:
  void configure(
    const Node::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    node_ = parent;
    tf_ = tf;
    map_ = map;
    prefix_ = name + ".";
    auto node = parent.lock();
    auto file = ament_index_cpp::get_package_share_directory("my_custom_mppi") +
      "/config/my_mppi.yaml";
    auto schema = rclcpp::parameter_map_from_yaml_file(file);
    for (const auto & v:schema.at("/controller_server")) {
      auto key = v.get_name().substr(std::string("MyCustomMPPI.").size());
      if (key == "plugin") {
        continue;
      }
      auto full = prefix_ + key;
      keys_.push_back(full);
      if (!node->has_parameter(full)) {node->declare_parameter(full, v.get_parameter_value());}
    }
    (void)Settings(current());
    markers_ = node->create_publisher<Markers>("my_custom_mppi/navigation_markers", 1);
    rollouts_ = node->create_publisher<Marker>("my_custom_mppi/trajectories", 1);
    callback_ = node->add_on_set_parameters_callback(
      [this](const auto & changes) {
        Result r;
        r.successful = true;
        try {
          auto p = current();
          bool changed = false;
          for (const auto & v:changes) {
            if (v.get_name().rfind(prefix_, 0) == 0) {
              auto it = p.find(v.get_name().substr(prefix_.size()));
              if (it != p.end()) {it->second = v;changed = true;}
            }
          }
          if (changed) {(void)Settings(p);}
        } catch (const std::exception & e) {r.successful = false;r.reason = e.what();}
        return r;
      });
  }
  void activate() override {markers_->on_activate();rollouts_->on_activate();}
  void deactivate() override {markers_->on_deactivate();rollouts_->on_deactivate();reset_ = true;}
  void cleanup() override
  {
    callback_.reset();
    markers_.reset();
    rollouts_.reset();
    keys_.clear();
    cached_.clear();
    reset_ = true;
    seed_ = -1;
  }
  void setPlan(const nav_msgs::msg::Path & p) override
  {
    std::lock_guard<std::mutex> lock(state_);
    reset_ |= p.poses.empty() || plan_.poses.empty() ||
      p.header.frame_id != plan_.header.frame_id ||
      p.poses.back().pose != plan_.poses.back().pose;
    plan_ = p;
  }
  void setSpeedLimit(const double & v, const bool & pct) override
  {
    std::lock_guard<std::mutex> lock(state_);
    limit_ = v;
    percentage_ = pct;
  }
  Command computeVelocityCommands(
    const Pose & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker *) override
  {
    Settings s(current());
    std::lock_guard<std::mutex> lock(state_);
    try {
      if (plan_.poses.empty()) {throw std::runtime_error("Empty MPPI plan");}
      auto tf = tf_->lookupTransform(
        map_->getGlobalFrameID(), plan_.header.frame_id, tf2_ros::fromMsg(
          pose.header.stamp), tf2::durationFromSec(.2));
      points_.clear();
      path_.clear();
      Pose local;
      for (const auto & p:plan_.poses) {
        tf2::doTransform(p, local, tf);
        auto xy = XY{local.pose.position.x, local.pose.position.y};
        if (!std::isfinite(xy[0]) || !std::isfinite(xy[1])) {
          throw std::runtime_error("Nonfinite path pose");
        }
        points_.push_back(xy);
        path_.insert(path_.end(), xy.begin(), xy.end());
      }
      double goal_yaw = tf2::getYaw(local.pose.orientation);
      auto costmap = map_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock_grid(*costmap->getMutex());
      int w = costmap->getSizeInCellsX(), height = costmap->getSizeInCellsY();
      double x = costmap->getOriginX(), y = costmap->getOriginY(), res = costmap->getResolution();
      bool new_grid = w != grid_.w || height != grid_.h || x != grid_.x || y != grid_.y ||
        res != grid_.res;
      size_t cells = size_t(w) * height;
      raw_.assign(costmap->getCharMap(), costmap->getCharMap() + cells);
      lock_grid.unlock();
      grid_.w = w;
      grid_.h = height;
      grid_.x = x;
      grid_.y = y;
      grid_.res = res;
      grid_.data.resize(
        5 * cells);
      for (size_t i = 0; i < cells; ++i) {
        grid_.data[i] = raw_[i] == 255 ? -1. : raw_[i] >= 253 ? 100. : raw_[i] * (99. / 252.);
      }
      if (new_grid || cached_ != points_) {grid_.geometry(points_);cached_ = points_;}
      auto & m = s.motion;
      m.pv = velocity.linear.x;
      m.pw = velocity.angular.z;
      if (limit_ > 0) {
        if (percentage_) {double f = std::min(1., limit_ / 100.);m.v_max *= f;m.v_min *= f;} else {
          m.v_max = std::min(limit_, m.v_max);
          m.v_min = std::max(-limit_, m.v_min);
        }
      }
      XY robot{pose.pose.position.x, pose.pose.position.y};
      double yaw = tf2::getYaw(
        pose.pose.orientation);
      if (!std::isfinite(robot[0]) || !std::isfinite(robot[1]) || !std::isfinite(yaw) ||
        !std::isfinite(goal_yaw) || !std::isfinite(m.pv) || !std::isfinite(m.pw))
      {
        throw std::runtime_error("Nonfinite pose or velocity");
      }
      double dist = distance(robot, points_.back()), look = dist < s.look ? .2 : s.look;
      const bool near_goal = s.near > 0 && dist <= s.near;
      const int h = near_goal ? std::min(s.short_h, s.horizon) : s.horizon;
      if (near_goal) {
        const double scale = s.near_goal_limit_percent / 100.;
        m.v_min *= scale;
        m.v_max *= scale;
        m.w_min *= scale;
        m.w_max *= scale;
      }
      reference(
        points_, robot, yaw, grid_, h, s.dt, look, look * s.rejoin / 100., m.v_max, m.a_min,
        m.w_min, m.w_max, refs_);
      std::copy(s.weights.begin(), s.weights.end(), refs_.begin() + 3 * h + 2);
      if (reset_ || previous_dt_ != s.dt) {
        controls_.resize(2 * h);
        for (int t = 0; t < h; ++t) {
          controls_[2 * t] = std::min(m.v_max, m.a_max * s.dt * (t + 1));
          controls_[2 * t + 1] = 0.;
        }
      } else {
        auto n = controls_.size();
        controls_.resize(2 * h);
        for (size_t i = n; i < controls_.size();
          ++i)
        {
          controls_[i] = controls_[i - 2];
        }
      }
      previous_dt_ = s.dt;
      reset_ = false;
      if (s.seed != seed_) {seed_ = s.seed;rng_.seed(seed_);}
      int stride = std::max(1, s.samples / 45), count = (s.samples - 1) / stride + 1;
      bool display = s.show && rollouts_->get_subscription_count();
      if (display) {
        displayed_.resize(size_t(3) * count * h);
      }
      chosen_.resize(3 * h);
      double position[] = {robot[0], robot[1], yaw}, cmd[2]{}, weighted[2]{}, curve[2]{},
        info[2]{};
      std::array<double, 12> motion;
      std::memcpy(motion.data(), &m, sizeof(m));
      int status = mppi_compute(
        s.samples, h, s.dt, s.temperature, 0., 0., rng_(), position,
        grid_.data.data(), height, w, res, x, y,
        path_.data(), points_.size(), goal_yaw, controls_.data(), cmd,
        chosen_.data(), display ? displayed_.data() : nullptr,
        motion.data(), refs_.data(), nullptr, weighted, curve, info, nullptr, nullptr);
      if (status < 0) {throw std::runtime_error("Native MPPI kernel failed");}
      visualize(pose, h, status, int(info[0]), display, count, look);
      Command out;
      out.header = pose.header;
      out.header.frame_id = map_->getBaseFrameID();
      out.twist.linear.x = cmd[0];
      out.twist.angular.z = cmd[1];
      return out;
    } catch (const std::exception & e) {throw nav2_core::PlannerException(e.what());}
  }
};
}
PLUGINLIB_EXPORT_CLASS(my_custom_mppi::Controller, nav2_core::Controller)
