#include "estimator.h"

namespace gs_lio
{

std::shared_mutex estimator::state_mtx;

state_t estimator::optimal_state;

void estimator::zupt(const stamp_t &tailstamp)
{
  // TODO: implement zero-velocity update
}

void estimator::reset(const state_t &state)
{
  std::unique_lock lock(estimator::state_mtx);
  estimator::optimal_state = state;
  propagated_queue.clear();
}

state_t estimator::get_state() const
{
  std::shared_lock lock(estimator::state_mtx);
  return estimator::optimal_state;
}

void estimator::set_state(const state_t &state)
{
  std::unique_lock lock(estimator::state_mtx);
  estimator::optimal_state = state;
}

} // namespace gs_lio