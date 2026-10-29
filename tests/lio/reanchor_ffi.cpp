// Test-only C ABI: Python orchestration exercises the compiled C++ candidate, not a second implementation.
#include "../../candidates/native_odom_reanchor/reanchor.hpp"
using namespace qazbot::candidate;
extern "C" {
void * ra_new() {return new Reanchor;}
void ra_delete(void * p) {delete static_cast<Reanchor *>(p);}
int ra_update(void * p, double t, double x, double y, double yaw, double now, double * out)
{
  auto v = static_cast<Reanchor *>(p)->update({t, {x, y, yaw}}, now);
  if (!v) {return 0;}
  out[0] = v->pose.x; out[1] = v->pose.y; out[2] = v->pose.yaw;
  return 1;
}
int ra_begin(void * p, unsigned long epoch, int transition, int inhibited)
{return static_cast<Reanchor *>(p)->begin_reset(epoch, transition, inhibited);}
int ra_confirm(void * p, unsigned long epoch, double t, double x, double y, double yaw,
  double ax, double ay, double ayaw, double now, int proof, int stand, int inhibited)
{
  return static_cast<Reanchor *>(p)->confirm_reset(epoch, {t, {x, y, yaw}},
    {t, {ax, ay, ayaw}}, now, proof, stand, inhibited);
}
int ra_release(void * p, unsigned long epoch, int trust, double now)
{return static_cast<Reanchor *>(p)->release(epoch, trust, now);}
int ra_motion(void * p, double now, int trust)
{return static_cast<Reanchor *>(p)->motion_allowed(now, trust);}
}
