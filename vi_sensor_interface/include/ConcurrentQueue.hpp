/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 *
 * All rights reserved.
 *
 * Redistribution and non-commercial use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of the {organization} nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CONCURRENTQUEUE_HPP_
#define CONCURRENTQUEUE_HPP_

#include <queue>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

template<typename T>
class ConcurrentQueue {
 private:
  std::queue<T> q_;
  boost::mutex m_;
  boost::condition_variable c_;
  unsigned int cntr;

 public:
  ConcurrentQueue() {
    cntr = 0;
  };

  void push(const T& data) {
    boost::lock_guard<boost::mutex> l(m_);
    q_.push(data);
    c_.notify_one();
  }

  //if the queue is empty and we pop, we just wait for data!!
  T pop() {
    boost::mutex::scoped_lock l(m_);

    if (q_.size() == 0) {
      c_.wait(l);
    }

    T res = q_.front();
    q_.pop();
    return res;
  }

  //checks queue is empty
  bool empty() {
    boost::mutex::scoped_lock l(m_);
    return q_.size() == 0;
  }

  //return size of queue
  size_t size() {
    boost::mutex::scoped_lock l(m_);
    return q_.size();
  }

  void clear() {
    std::queue<int> empty;
    std::swap(q_, empty);
  }

  //returns current queue and clear it
  std::queue<T> pop_all() {
    boost::mutex::scoped_lock l(m_);

    std::queue<T> queue; //empty here
    std::swap(queue, q_);

    return queue;
  }

  //returns current queue and clear it
  std::queue<T> clone() {
    boost::mutex::scoped_lock l(m_);

    std::queue<T> queue = q_;
    return queue;
  }

};

#endif /* CONCURRENTQUEUE_HPP_ */
