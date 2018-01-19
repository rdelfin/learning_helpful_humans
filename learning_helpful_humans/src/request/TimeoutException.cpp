/*
 * Copyright 2017 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "learning_helpful_humans/request/TimeoutException.h"

#include <sstream>

TimeoutException::TimeoutException()
    : _time(0)
{
    
}

TimeoutException::TimeoutException(long int time)
    : _time(time)
{

}

long int TimeoutException::time()
{
    return _time;
}

std::string TimeoutException::what()
{
    std::stringstream ss;
    ss << "The request timeout with maximum timeout of " << _time;
    return ss.str();
}

TimeoutException::~TimeoutException()
{

}
