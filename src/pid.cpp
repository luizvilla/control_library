/*
 * Copyright (c) 2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @date 2024
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 */

#include <zephyr/logging/log.h>
#include <errno.h>
#include "pid.h"

LOG_MODULE_DECLARE(ot_control);

int8_t Pid::setDerivativeParams(float32_t Td, float32_t N) {
    float32_t tau = 0.0;

    if (N != 0.0) {
        tau = Td / N;
    }

    if (tau < 0.0) {
        LOG_ERR("Td/N should be > 0");
        return -EINVAL;
    }

    _Td = Td;
    _N = N;
    _b1_filter = _Ts / (_Ts + tau);
    _a1_filter = -tau / (_Ts + tau);

    return 0;
}

int8_t Pid::init(PidParams p) {

    if (setTs(p.Ts) != 0) {
        return -EINVAL;
    }

    if (setKp(p.Kp) != 0) {
        return -EINVAL;
    }

    if (setTi(p.Ti) != 0) {
        return -EINVAL;
    }

    if (setDerivativeParams(p.Td, p.N) != 0) {
        return -EINVAL;
    }

    if (p.lower_bound > p.upper_bound) {
        LOG_ERR("lower bound > upper_bound");
        return -EINVAL;
    }
    _lower_bound = p.lower_bound;
    _upper_bound = p.upper_bound;


    _integral = 0.0;
    _previous_error = 0.0;
    _previous_f_deriv = 0.0;
    _output = 0.0;

    LOG_DBG("_Ts = %f\n", (double)_Ts);
    LOG_DBG("_Kp = %f\n", (double)_Kp);
    LOG_DBG("_Td = %f\n", (double)_Td);
    LOG_DBG("_Ti = %f\n", (double)_Ti);
    LOG_DBG("_N = %f\n", (double)_N);

    return 0;
}

void Pid::calculate(void) {
    float32_t error;
    float32_t deriv, filtered_deriv;
    float32_t tmp_output;
    error = _reference - _measure;

    _integral = _integral + _Ts * error;

    deriv = _inverse_Ts * (error - _previous_error);

    filtered_deriv = _b1_filter * deriv - _a1_filter * _previous_f_deriv; 

    tmp_output = _Kp * ( error + _inverse_Ti * _integral + _Td * filtered_deriv ) ; 

    _output = saturate(tmp_output);
    // re-compute integral to no have integral divergence during saturation
    if (_output != tmp_output)
        _integral = _Ti * (_inverse_Kp * _output - error - _Td * filtered_deriv);

    _previous_error = error;
    
    _previous_f_deriv = filtered_deriv;
}


void Pid::reset() {
    Pid::reset(0.0);
}

void Pid::reset(float32_t output=0.0) {
    _integral = _Ti * _inverse_Kp * output;
    _output = 0.0;
    _previous_f_deriv = 0.0;
    _previous_error = 0.0;
}

float32_t Pid::getTs() const {
    return _Ts;
}

int8_t Pid::setTs(float32_t value) {
    if (value <= 0.0) {
        LOG_ERR("Ts should be > 0");
        return -EINVAL;
    }

    _Ts = value;
    _inverse_Ts = 1.0 / value;

    return setDerivativeParams(_Td, _N);
}

float32_t Pid::getKp() const {
    return _Kp;
}

int8_t Pid::setKp(float32_t value) {
    if (value == 0.0) {
        LOG_ERR("Kp equal To 0");
        return -EINVAL;
    }

    _Kp = value;
    _inverse_Kp = 1.0 / value;
    return 0;
}

float32_t Pid::getTi() const {
    return _Ti;
}

int8_t Pid::setTi(float32_t value) {
    if (value == 0.0) {
        LOG_ERR("Ti can not be equal to 0.0\n");
        return -EINVAL;
    }

    _Ti = value;
    _inverse_Ti = 1.0 / value;
    return 0;
}

float32_t Pid::getTd() const {
    return _Td;
}

int8_t Pid::setTd(float32_t value) {
    return setDerivativeParams(value, _N);
}

float32_t Pid::getN() const {
    return _N;
}

int8_t Pid::setN(float32_t value) {
    return setDerivativeParams(_Td, value);
}

float32_t Pid::getLowerBound() const {
    return _lower_bound;
}

int8_t Pid::setLowerBound(float32_t value) {
    if (value > _upper_bound) {
        LOG_ERR("lower bound > upper_bound");
        return -EINVAL;
    }

    _lower_bound = value;
    return 0;
}

float32_t Pid::getUpperBound() const {
    return _upper_bound;
}

int8_t Pid::setUpperBound(float32_t value) {
    if (value < _lower_bound) {
        LOG_ERR("lower bound > upper_bound");
        return -EINVAL;
    }

    _upper_bound = value;
    return 0;
}
