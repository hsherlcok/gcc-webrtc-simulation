/******************************************************************************
 * Copyright 2016-2017 Cisco Systems, Inc.                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 *                                                                            *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 ******************************************************************************/

/**
 * @file
 * Dummy controller (CBR) implementation for rmcat ns3 module.
 *
 * @version 0.1.1
 * @author Jiantao Fu
 * @author Sergio Mena
 * @author Xiaoqing Zhu
 */
#include "gcc-controller.h"
#include <sstream>
#include <cassert>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include "rtc_base/numerics/safe_minmax.h"


namespace rmcat {

enum { kMinFramePeriodHistoryLength = 60 };
enum { kDeltaCounterMax = 1000 };

const double kMaxAdaptOffsetMs = 15.0;
const double kOverUsingTimeThreshold = 10;
const int kMinNumDeltas = 60;

GccController::GccController() :
    SenderBasedController{},
    m_lastTimeCalcUs{0},
    m_lastTimeCalcValid{false},
    m_QdelayUs{0},
    m_ploss{0},
    m_plr{0.f},
    m_RecvR{0.}, 

    num_of_deltas_(0),
    slope_(0),	//need initial value
    offset_(0),	//need initial value
    prev_offset_(0),	//need initial value
    E_(),
    process_noise_(),
    avg_noise_(0),	//need initial value
    var_noise_(0),	//need initial value
    ts_delta_hist_(),	
 
    k_up_(0.0087),
    k_down_(0.039),
    overusing_time_threshold_(100),
    threshold_(12.5),
    last_update_ms_(-1),
    D_prev_offset_(0.0),
    time_over_using_(-1),
    overuse_counter_(0),
    D_hypothesis_("Normal")   

{}

GccController::~GccController() {}

void GccController::setCurrentBw(float newBw) {
    m_initBw = newBw;
}

void GccController::reset() {
    m_lastTimeCalcUs = 0;
    m_lastTimeCalcValid = false;

    m_QdelayUs = 0;
    m_ploss = 0;
    m_plr = 0.f;
    m_RecvR = 0.;

    SenderBasedController::reset();
}

bool GccController::processFeedback(uint64_t nowUs,
                                      uint16_t sequence,
                                      uint64_t rxTimestampUs,
                                      uint8_t ecn,
                                      uint64_t l_inter_arrival,
                                      uint64_t l_inter_departure,
                                      uint64_t l_inter_delay_var) {
    // First of all, call the superclass
    const bool res = SenderBasedController::processFeedback(nowUs, sequence,
                                                            rxTimestampUs, ecn,
                                                            l_inter_arrival,
                                                            l_inter_departure,
                                                            l_inter_delay_var);
    
    const uint64_t calcIntervalUs = 200 * 1000;
    if (m_lastTimeCalcValid) {
        assert(lessThan(m_lastTimeCalcUs, nowUs + 1));
        if (nowUs - m_lastTimeCalcUs >= calcIntervalUs) {
            updateMetrics();
            logStats(nowUs);
            m_lastTimeCalcUs = nowUs;
        }
    } else {
        m_lastTimeCalcUs = nowUs;
        m_lastTimeCalcValid = true;
    }
    return res;
}

float GccController::getBandwidth(uint64_t nowUs) const {

    return m_initBw;
}

void GccController::updateMetrics() {
    uint64_t qdelayUs;
    bool qdelayOK = getCurrentQdelay(qdelayUs);
    if (qdelayOK) m_QdelayUs = qdelayUs;

    float rrate;
    bool rrateOK = getCurrentRecvRate(rrate);
    if (rrateOK) m_RecvR = rrate;

    uint32_t nLoss;
    float plr;
    bool plrOK = getPktLossInfo(nLoss, plr);
    if (plrOK) {
        m_ploss = nLoss;
        m_plr = plr;
    }
}

void GccController::logStats(uint64_t nowUs) const {

    std::ostringstream os;
    os << std::fixed;
    os.precision(RMCAT_LOG_PRINT_PRECISION);

    os  << " algo:dummy " << m_id
        << " ts: "     << (nowUs / 1000)
        << " loglen: " << m_packetHistory.size()
        << " qdel: "   << (m_QdelayUs / 1000)
        << " ploss: "  << m_ploss
        << " plr: "    << m_plr
        << " rrate: "  << m_RecvR
        << " srate: "  << m_initBw;
    logMessage(os.str());
}

void GccController::OveruseEstimatorUpdate(int64_t t_delta, double ts_delta, int size_delta, std::string current_hypothesis, int64_t now_ms){
	const double min_frame_period = UpdateMinFramePeriod(ts_delta);
	const double t_ts_delta = t_delta - ts_delta;
	double fs_delta = size_delta;
	
	++num_of_deltas;
	if (num_of_deltas_ > kDeltaCounterMax) {
   		num_of_deltas_ = kDeltaCounterMax;
  	}

  	// Update the Kalman filter.
  	E_[0][0] += process_noise_[0];
  	E_[1][1] += process_noise_[1];

  	if ((current_hypothesis.compare("Overusing") &&
       		offset_ < prev_offset_) ||							  (current_hypothesis.compare("Underusing") &&
       		offset_ > prev_offset_)) {
    		E_[1][1] += 10 * process_noise_[1];
  	}

  	const double h[2] = {fs_delta, 1.0};
  	const double Eh[2] = {E_[0][0] * h[0] + E_[0][1] * h[1],
                        E_[1][0] * h[0] + E_[1][1] * h[1]};

 	const double residual = t_ts_delta - slope_ * h[0] - offset_;

  	const bool in_stable_state =
      		(current_hypothesis.compare("Normal");
  	const double max_residual = 3.0 * sqrt(var_noise_);
  	// We try to filter out very late frames. For instance periodic key
  	// frames doesn't fit the Gaussian model well.
  	if (fabs(residual) < max_residual) {
    		UpdateNoiseEstimate(residual, min_frame_period, in_stable_state);
  	} else {
    		UpdateNoiseEstimate(residual < 0 ? -max_residual : max_residual,
                        min_frame_period, in_stable_state);
  	}

  	const double denom = var_noise_ + h[0] * Eh[0] + h[1] * Eh[1];

  	const double K[2] = {Eh[0] / denom, Eh[1] / denom};

  	const double IKh[2][2] = {{1.0 - K[0] * h[0], -K[0] * h[1]},
                            {-K[1] * h[0], 1.0 - K[1] * h[1]}};
  	const double e00 = E_[0][0];
	const double e01 = E_[0][1];
                              
	  // Update state.            
  	E_[0][0] = e00 * IKh[0][0] + E_[1][0] * IKh[0][1];
  	E_[0][1] = e01 * IKh[0][0] + E_[1][1] * IKh[0][1];
  	E_[1][0] = e00 * IKh[1][0] + E_[1][0] * IKh[1][1];
  	E_[1][1] = e01 * IKh[1][0] + E_[1][1] * IKh[1][1];
  
 	// The covariance matrix must be positive semi-definite.
  	bool positive_semi_definite =
      		E_[0][0] + E_[1][1] >= 0 &&
      		E_[0][0] * E_[1][1] - E_[0][1] * E_[1][0] >= 0 && E_[0][0] >= 0;
  	assert(positive_semi_definite);
  

  	slope_ = slope_ + K[0] * residual;
  	prev_offset_ = offset_;
  	offset_ = offset_ + K[1] * residual;

}

double GccController::UpdateMinFramePeriod(double ts_delta) {
  double min_frame_period = ts_delta;
  if (ts_delta_hist_.size() >= kMinFramePeriodHistoryLength) {
    ts_delta_hist_.pop_front();
  }
  for (const double old_ts_delta : ts_delta_hist_) {
    min_frame_period = std::min(old_ts_delta, min_frame_period);
  }
  ts_delta_hist_.push_back(ts_delta);
  return min_frame_period;
}

void GccController::UpdateNoiseEstimate(double residual,
                                           double ts_delta,
                                           bool stable_state) {
  if (!stable_state) {
    return;
  }
  // Faster filter during startup to faster adapt to the jitter level
  // of the network. |alpha| is tuned for 30 frames per second, but is scaled
  // according to |ts_delta|.
  double alpha = 0.01;
  if (num_of_deltas_ > 10 * 30) {
    alpha = 0.002;
 }
  // Only update the noise estimate if we're not over-using. |beta| is a
  // function of alpha and the time delta since the previous update.
  const double beta = pow(1 - alpha, ts_delta * 30.0 / 1000.0);
  avg_noise_ = beta * avg_noise_ + (1 - beta) * residual;
  var_noise_ = beta * var_noise_ +
               (1 - beta) * (avg_noise_ - residual) * (avg_noise_ - residual);
  if (var_noise_ < 1) {
    var_noise_ = 1;
  }
}

void UpdateThreshold(double modified_offset, int64_t now_ms){

  if (last_update_ms_ == -1)
    last_update_ms_ = now_ms;

  if (fabs(modified_offset) > threshold_ + kMaxAdaptOffsetMs) {
    // Avoid adapting the threshold to big latency spikes, caused e.g.,
    // by a sudden capacity drop.
    last_update_ms_ = now_ms;
    return;
  }

  const double k = fabs(modified_offset) < threshold_ ? k_down_ : k_up_;
  const int64_t kMaxTimeDeltaMs = 100;
  int64_t time_delta_ms = std::min(now_ms - last_update_ms_, kMaxTimeDeltaMs);
  threshold_ += k * (fabs(modified_offset) - threshold_) * time_delta_ms;
  threshold_ = rtc::SafeClamp(threshold_, 6.f, 600.f);
  last_update_ms_ = now_ms;
	
}

std::string State() const {
	return D_hypothesis_;
}

std::string OveruseDetectorDetect(double offset, double timestamp_delta, int num_of_deltas, int64_t now_ms){

  if (num_of_deltas < 2) {
    return "Normal";
  }
  const double T = std::min(num_of_deltas, kMinNumDeltas) * offset;
   
  if (T > threshold_) {
    if (time_over_using_ == -1) {
      // Initialize the timer. Assume that we've been
      // over-using half of the time since the previous
      // sample.
      time_over_using_ = ts_delta / 2;
    } else {
      // Increment timer
      time_over_using_ += ts_delta;
    }
    overuse_counter_++;
    if (time_over_using_ > overusing_time_threshold_ && overuse_counter_ > 1) {
      if (offset >= prev_offset_) {
        time_over_using_ = 0;
        overuse_counter_ = 0;
        D_hypothesis_ = "Overusing";
      }
    }
  } else if (T < -threshold_) {
    time_over_using_ = -1;
    overuse_counter_ = 0;
    D_hypothesis_ = "Underusing";
  } else {
    time_over_using_ = -1;
    overuse_counter_ = 0;
    D_hypothesis_ = "Normal";
  }
  D_prev_offset_ = offset;

  UpdateThreshold(T, now_ms);

  return D_hypothesis_;

}



}
