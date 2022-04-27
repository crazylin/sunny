// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>

#undef NDEBUG
#include <cassert>

#include <deque>
#include <future>
#include <iostream>
#include <vector>

bool _ok = true;

std::vector<int> _ret;

std::vector<int> _num;

/**
 * @brief Number of co-workers.
 *
 */
int _workers = 4;

/**
 * @brief Mutex to protect image queue.
 *
 */
std::mutex _images_mut;

/**
 * @brief Condition variable for image queue.
 *
 */
std::condition_variable _images_con;

/**
 * @brief Double end queue for images.
 *
 */
std::deque<int> _images;

/**
 * @brief Mutex to protect result queue.
 *
 */
std::mutex _futures_mut;

/**
 * @brief Condition variable for result queue.
 *
 */
std::condition_variable _futures_con;

/**
 * @brief Double end queue for results.
 *
 */
std::deque<std::future<int>> _futures;

/**
 * @brief Threads for workers and the manager.
 *
 */
std::vector<std::thread> _threads;

void _push_back_image(int ptr)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.emplace_back(ptr);
  // auto s = static_cast<int>(_images.size());
  // if (s > _workers + 1) {
  //   _images.pop_front();
  // }
  lk.unlock();
  _images_con.notify_all();
}

void _push_back_future(std::future<int> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
}

void _worker()
{
  int count = 0;
  while (_ok) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = _images.front();
      _images.pop_front();
      std::promise<int> prom;
      _push_back_future(prom.get_future());
      lk.unlock();
      sleep(1);
      ++count;
      prom.set_value(ptr);
    } else {
      _images_con.wait(lk);
    }
  }
  std::lock_guard<std::mutex> lk(_images_mut);
  _num.push_back(count);
}

void _manager()
{
  while (_ok) {
    std::unique_lock<std::mutex> lk(_futures_mut);
    if (_futures.empty() == false) {
      auto f = std::move(_futures.front());
      _futures.pop_front();
      lk.unlock();
      auto ptr = f.get();
      _ret.push_back(ptr);
    } else {
      _futures_con.wait(lk);
    }
  }
}

int main()
{
  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(_worker));
  }
  _threads.push_back(std::thread(_manager));

  sleep(1);
  
  for (auto i = 0; i < 50; ++i) {
    _push_back_image(i);
  }

  sleep(15);
  std::cout << "wake up" << std::endl;

  _ok = false;
  _images_con.notify_all();
  _futures_con.notify_one();

  for (auto & t : _threads) {
    t.join();
  }

  for (size_t i = 0; i < _ret.size(); ++i) {
    std::cout << _ret[i] << " ";
  }

  std::cout << "\n";

  for (size_t i = 0; i < _num.size(); ++i) {
    std::cout << _num[i] << " ";
  }

  std::cout << std::endl;
  return 0;
}
