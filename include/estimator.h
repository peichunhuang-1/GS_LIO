#ifndef GS_LIO_ESTIMATOR_H
#define GS_LIO_ESTIMATOR_H

#include "state.h"
#include <deque>

namespace gs_lio
{

class estimator
{
public:
  estimator() = default;
  virtual ~estimator() = default;
  virtual void zupt(const stamp_t &tailstamp);
  virtual bool forward(const stamp_t &tailstamp = -1) {return false;}
  virtual void optimize() {}
  virtual void reset(const state_t &state);
protected:
  static std::shared_mutex state_mtx;
  static state_t optimal_state;
  state_t get_state() const;
  void set_state(const state_t &state);
  std::deque<state_t> propagated_queue;
  std::string world_frame;
};

}

#endif