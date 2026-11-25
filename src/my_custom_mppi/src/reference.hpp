#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace my_custom_mppi
{
using XY = std::array<double, 2>;
inline XY delta(XY a, XY b) {return {a[0] - b[0], a[1] - b[1]};}
inline double dot(XY a, XY b) {return a[0] * b[0] + a[1] * b[1];}
inline double distance(XY a, XY b) {auto d = delta(a, b);return std::hypot(d[0], d[1]);}
inline XY lerp(XY a, XY b, double u) {return {a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])};}
inline double angle(double a) {return a - 2 * M_PI * std::floor((a + M_PI) / (2 * M_PI));}
inline double projection(XY p, XY a, XY e, double len2)
{
  return len2 <= 1e-12 ? 0. : std::clamp(dot(delta(p, a), e) / len2, 0., 1.);
}
// A balanced hierarchy of contiguous path segments. Bounds prune only segments
// that cannot win. Equal-distance ties use the earliest original segment.
class SegmentIndex
{
  struct Box { XY lo, hi; size_t begin, end, left = 0, right = 0; };
  const std::vector<XY> & path;
  std::vector<Box> boxes;
  size_t build(size_t a, size_t b)
  {
    size_t idx = boxes.size();
    boxes.push_back({path[a], path[a], a, b});
    if (b - a > 8) {
      size_t mid = a + (b - a) / 2;
      auto l = build(a, mid), r = build(mid, b);
      boxes[idx].left = l;
      boxes[idx].right = r;
      for (int j = 0; j < 2; ++j) {
        boxes[idx].lo[j] = std::min(boxes[l].lo[j], boxes[r].lo[j]);
        boxes[idx].hi[j] = std::max(
          boxes[l].hi[j], boxes[r].hi[j]);
      }
    } else {
      for (size_t k = a + 1; k <= b; ++k) {
        for (int j = 0; j < 2; ++j) {
          boxes[idx].lo[j] = std::min(boxes[idx].lo[j], path[k][j]);
          boxes[idx].hi[j] = std::max(boxes[idx].hi[j], path[k][j]);
        }
      }
    }
    return idx;
  }
  double bound(size_t i, XY p) const
  {
    const auto & b = boxes[i];
    XY gap;
    for (int j = 0; j < 2; ++j) {gap[j] = std::max({0., b.lo[j] - p[j], p[j] - b.hi[j]});}
    return dot(gap, gap);
  }
  void search(size_t i, XY p, double & best, size_t & winner) const
  {
    const auto & b = boxes[i];
    if (b.right) {
      size_t l = b.left, r = b.right;
      double dl = bound(l, p), dr = bound(r, p);
      if (dl > dr) {std::swap(l, r);std::swap(dl, dr);}
      if (dl <= best + 1e-12) {search(l, p, best, winner);}
      if (dr <= best + 1e-12) {search(r, p, best, winner);}
      return;
    }
    for (size_t k = b.begin; k < b.end; ++k) {
      auto e = delta(path[k + 1], path[k]);
      double len = dot(e, e);
      if (len <= 1e-12) {continue;}
      auto off = delta(p, path[k]);
      double u = std::clamp(dot(off, e) / len, 0., 1.);
      XY d{off[0] - u * e[0], off[1] - u * e[1]};
      double cost = dot(d, d);
      if (cost < best || (cost == best && k < winner)) {best = cost;winner = k;}
    }
  }

public:
  explicit SegmentIndex(const std::vector<XY> & points)
  : path(points) {build(0, path.size() - 1);}
  void endpoints(XY p, double * out) const
  {
    size_t k = path.size() - 1;
    double best = INFINITY;
    search(0, p, best, k);
    const auto & a = std::isfinite(best) ? path[k] : path.front();
    const auto & b = std::isfinite(best) ? path[k + 1] : path.front();
    std::copy(a.begin(), a.end(), out);
    std::copy(b.begin(), b.end(), out + 2);
  }
};
struct Grid
{
  int w = 0, h = 0;
  double x = 0, y = 0, res = 0;
  std::vector<double> data;
  double sample(XY p) const
  {
    int i = std::floor((p[0] - x) / res), j = std::floor((p[1] - y) / res);
    return i < 0 || i >= w || j < 0 || j >= h ? -1. : data[j * w + i];
  }
  void geometry(const std::vector<XY> & path)
  {
    size_t cells = size_t(w) * h;
    SegmentIndex index(path);
    #pragma omp parallel for schedule(static) num_threads(6) if(cells>=4096)
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        index.endpoints(
          {x + (i + .5) * res, y + (j + .5) * res},
          data.data() + cells + 4 * (j * w + i));
      }
    }
  }
};
template<class T> T interpolate(
  double s, const std::vector<double> & arc,
  const std::vector<T> & values)
{
  auto it = std::upper_bound(arc.begin(), arc.end(), s);
  if (it == arc.end()) {return values.back();}
  if (it == arc.begin()) {return values.front();}
  size_t k = it - arc.begin();
  double u = (s - arc[k - 1]) / (arc[k] - arc[k - 1]);
  if constexpr (std::is_same_v<T, XY>) {return lerp(values[k - 1], values[k], u);} else {
    return values[k - 1] + u * (values[k] - values[k - 1]);
  }
}
inline void reference(
  const std::vector<XY> & path, XY robot, double yaw, const Grid & grid,
  int h, double dt, double lookahead, double rejoin, double vmax, double amin, double wmin,
  double wmax,
  std::vector<double> & refs)
{
  double best = INFINITY;
  size_t next = 1;
  XY closest = path.front();
  for (size_t k = 1; k < path.size(); ++k) {
    auto e = delta(path[k], path[k - 1]);
    auto p = lerp(path[k - 1], path[k], projection(robot, path[k - 1], e, dot(e, e)));
    auto d = delta(p, robot);
    if (dot(d, d) < best) {best = dot(d, d);closest = p;next = k;}
  }
  std::vector<XY> rem{closest};
  std::vector<double> arc{0.}, headings;
  for (size_t k = next; k < path.size(); ++k) {
    auto e = delta(path[k], rem.back());
    double l = std::hypot(e[0], e[1]);
    if (l <= 1e-12) {continue;}
    rem.push_back(path[k]);
    arc.push_back(arc.back() + l);
    headings.push_back(std::atan2(e[1], e[0]));
  }
  refs.assign(3 * h + 2 + 14, 0.);
  refs[3 * h + 1] = rejoin;
  auto hold = [&](XY target, double speed) {
      for (int t = 0; t < h; ++t) {
        refs[2 * t] = target[0];
        refs[2 * t + 1] = target[1];
        refs[2 * h + t] = speed;
      }
    };
  if (rem.size() == 1) {hold(rem.front(), 0.);return;}
  double travel = std::min(lookahead, arc.back()), spacing = std::min(.05, grid.res / 2);
  bool off = distance(robot, closest) > rejoin;
  auto target = off ? closest : interpolate(travel, arc, rem);
  auto d = delta(target, robot);
  double dist = distance(target, robot);
  if (off || (dist > rejoin && std::abs(angle(std::atan2(d[1], d[0]) - yaw)) > 1.2)) {
    bool free = true;
    int n = std::max(2, int(dist / spacing) + 2);
    for (int i = 0; i < n; ++i) {
      double c = grid.sample(lerp(robot, target, double(i) / (n - 1)));
      free &= c >= 0 && c < 100;
    }
    if (free) {
      refs[3 * h] = 4.;
      hold(target, std::min({vmax, .5, dist}));
      return;
    }
  }
  std::vector<double> positions = arc, limits;
  for (size_t i = 0; i * spacing < arc.back(); ++i) {
    positions.push_back(i * spacing);
  }
  std::sort(positions.begin(), positions.end());
  positions.erase(std::unique(positions.begin(), positions.end()), positions.end());
  for (double s:positions) {
    double c = grid.sample(interpolate(s, arc, rem));
    limits.push_back(
      c < 0 || c >= 100 ? 0. : vmax * (1. - .70 * std::clamp(
        c / 99., 0.,
        1.)));
  }
  for (size_t k = 1; k < headings.size(); ++k) {
    double turn = angle(headings[k] - headings[k - 1]), a = std::abs(turn);
    auto begin = std::lower_bound(positions.begin(), positions.end(), arc[k] - .15);
    for (auto it = begin; it != positions.end() && *it <= arc[k] + .15; ++it) {
      auto & v = limits[it - positions.begin()];
      v /= 1. + 2 * a;
      if (a > 1e-6) {v = std::min(v, (turn > 0 ? wmax : -wmin) * .30 / a);}
    }
  }
  limits.back() = 0.;
  for (size_t j = limits.size() - 1; j > 0; --j) {
    limits[j -
      1] =
      std::min(
      limits[j - 1],
      std::sqrt(limits[j] * limits[j] - 2 * amin * (positions[j] - positions[j - 1])));
  }
  for (int t = 0; t < h; ++t) {
    auto p = interpolate(travel, arc, rem);
    refs[2 * t] = p[0];
    refs[2 * t + 1] = p[1];
    double v = interpolate(std::max(0., travel - lookahead), positions, limits);
    refs[2 * h + t] = v;
    travel = std::min(travel + v * dt, arc.back());
  }
}
}
