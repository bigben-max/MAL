/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-22 02:51:45
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 01:58:25
 */

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <vector>
//
#include "base/common/log.h"
#include "base/common/macros.h"

namespace base {

template <typename MsgType>
class DataBuffer {
 public:
  POINTER_TYPEDEFS(DataBuffer);
  using TimeMsgPair = std::pair<uint64_t, MsgType>;
  DataBuffer() = delete;
  DataBuffer(int max_buffer_size) : max_buffer_size_(max_buffer_size) {}
  ~DataBuffer() = default;

  void lockBuffer() { dq_msg_buf_mtx_.lock(); }

  void tryLockBuffer() { dq_msg_buf_mtx_.try_lock(); }

  void unlockBuffer() { dq_msg_buf_mtx_.unlock(); }

  void addMsg(const uint64_t timestamp, const MsgType &msg) {
    if (static_cast<int>(dq_msg_buf_.size()) > max_buffer_size_) {
      dq_msg_buf_.pop_front();
    }
    dq_msg_buf_.emplace_back(TimeMsgPair(timestamp, msg));
  }

  void addMsgSafe(const uint64_t &timestamp, const MsgType &msg) {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    this->addMsg(timestamp, msg);
  }

  bool getMsg(MsgType *msg_ptr) {
    if (static_cast<int>(dq_msg_buf_.size()) == 0) {
      msg_ptr = nullptr;
      return false;
    } else {
      *msg_ptr = dq_msg_buf_.front().second;
      return true;
    }
  }

  bool getMsgSafe(MsgType *msg_ptr) {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->getMsg(msg_ptr);
  }

  int size() const { return static_cast<int>(dq_msg_buf_.size()); }

  bool empty() const { return dq_msg_buf_.empty(); }
  bool emptySafe() const {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->empty();
  }

  void clear() { dq_msg_buf_.clear(); }
  void clearSafe() {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    this->clear();
  }

  const TimeMsgPair *front() const {
    if (dq_msg_buf_.empty())
      return nullptr;
    else
      return &dq_msg_buf_.front();
  }
  const TimeMsgPair *frontSafe() const {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->front();
  }

  const TimeMsgPair *back() const {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    if (dq_msg_buf_.empty())
      return nullptr;
    else
      return &dq_msg_buf_.back();
  }
  const TimeMsgPair *backSafe() const {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->back();
  }

  void popFront() {
    if (!dq_msg_buf_.empty()) dq_msg_buf_.pop_front();
  }
  void popFrontSafe() {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    this->popFront();
  }
  void popBack() {
    if (!dq_msg_buf_.empty()) dq_msg_buf_.pop_back();
  }
  void popBackSafe() {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    this->popBack();
  }

  bool searchMsgByTime(const uint64_t &msg_time, const uint64_t &tolerance,
                       MsgType *msg_ptr) {
    int64_t time_diff = 0;
    for (size_t i = 0; i < dq_msg_buf_.size(); i++) {
      const auto &data = dq_msg_buf_.at(i);
      if (data.first == msg_time) {
        *msg_ptr = data.second;
        return true;
      }
      time_diff = std::abs(static_cast<int64_t>(msg_time) -
                           static_cast<int64_t>(data.first));
      if (time_diff <= static_cast<int64_t>(tolerance)) {
        *msg_ptr = data.second;
        return true;
      }
    }
    return false;
  }
  bool searchMsgByTimeSafe(const uint64_t &msg_time, const uint64_t &tolerance,
                           MsgType *msg_ptr) {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->searchMsgByTime(msg_time, tolerance, msg_ptr);
  }

  void searchMsgByRangeTime(const uint64_t &start_time,
                            const uint64_t &end_time,
                            std::vector<MsgType> *msg_vec_ptr) {
    for (size_t i = 0; i < dq_msg_buf_.size(); ++i) {
      const auto &data = dq_msg_buf_.at(i);
      if (data.first > start_time && data.first <= end_time) {
        (*msg_vec_ptr).push_back(data.second);
      }
    }
  }
  void searchMsgByRangeTimeSafe(const uint64_t &start_time,
                                const uint64_t &end_time,
                                std::vector<MsgType> *msg_vec_ptr) {
    std::lock_guard<std::mutex> lk(dq_msg_buf_mtx_);
    return this->searchMsgByRangeTime(start_time, end_time, msg_vec_ptr);
  }

 private:
  const int max_buffer_size_ = 10;
  int buffer_size_;
  mutable std::mutex dq_msg_buf_mtx_;
  std::deque<TimeMsgPair> dq_msg_buf_;
};
}  // namespace base