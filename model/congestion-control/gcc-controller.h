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
 * Dummy controller (CBR) interface for rmcat ns3 module.
 *
 * @version 0.1.1
 * @author Jiantao Fu
 * @author Sergio Mena
 * @author Xiaoqing Zhu
 */

#ifndef GCC_CONTROLLER_H
#define GCC_CONTROLLER_H

#include "sender-based-controller.h"
#include <sstream>
#include <cassert>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include "rtc_base/numerics/safe_minmax.h"

namespace rmcat {

/**
 * Simplistic implementation of a sender-based congestion controller. The
 * algorithm simply returns a constant, hard-coded bandwidth when queried.
 */
class GccController: public SenderBasedController
{
public:
    /** Class constructor */
    GccController();

    /** Class destructor */
    virtual ~GccController();

    /**
     * Set the current bandwidth estimation. This can be useful in test environments
     * to temporarily disrupt the current bandwidth estimation
     *
     * @param [in] newBw Bandwidth estimation to overwrite the current estimation
     */
    virtual void setCurrentBw(float newBw);

    /**
     * Reset the internal state of the congestion controller
     */
    virtual void reset();

    /**
     * Simplistic implementation of feedback packet processing. It simply
     * prints calculated metrics at regular intervals
     */
    virtual bool processFeedback(uint64_t nowUs,
                                 uint16_t sequence,
                                 uint64_t rxTimestampUs,
                                 uint8_t ecn=0,
                                 uint64_t l_inter_arrival,
                                 uint64_t l_inter_departure,
                                 uint64_t l_inter_delay_var);
    /**
     * Simplistic implementation of bandwidth getter. It returns a hard-coded
     * bandwidth value in bits per second
     */
    virtual float getBandwidth(uint64_t nowUs) const;

	
/*Overuse Estimator Function */
    virtual void OveruseEstimatorUpdate(int64_t t_delta, double ts_delta,
	int size_delta, std::string current_hypothesis, int64_t now_ms);
	
/*Overuse Detector Function */
    virtual std::string OveruseDetectorDetect(double offset, double timestamp_delta, int num_of_deltas, int64_t now_ms);

    virtual std::string State() const;


private:
/*Overuse Estimator Function */
    double UpdateMinFramePeriod(double ts_delta);
    void UpdateNoiseEstimate(double residual, double ts_delta, bool stable_state);

/*Overuse Detector Function */
    void UpdateThreshold(double modified_offset, int64_t now_ms);


    void updateMetrics();
    void logStats(uint64_t nowUs) const;

    uint64_t m_lastTimeCalcUs;
    bool m_lastTimeCalcValid;

    uint64_t m_QdelayUs; /**< estimated queuing delay in microseconds */
    uint32_t m_ploss;  /**< packet loss count within configured window */
    float m_plr;       /**< packet loss ratio within packet history window */
    float m_RecvR;     /**< updated receiving rate in bps */
	
/*Overuse Estimator variable*/
    uint16_t num_of_deltas_;
    double slope_;
    double offset_;
    double prev_offset_;
    double E_[2][2];
    double process_noise_[2];
    double avg_noise_;
    double var_noise_;
    std::deque<double> ts_delta_hist_;

/*Overuse Detector variable*/
    double k_up_;
    double k_down_;
    double overusing_time_threshold_;
    double threshold_;
    int64_t last_update_ms_;
    double D_prev_offset_;
    double time_over_using_;
    int overuse_counter_;
    std::string D_hypothesis_;



};

}

#endif /* DUMMY_CONTROLLER_H */
