#pragma once
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>

using namespace std;

template <typename T>

class BlockingQueue {
private:
    BlockingQueue(const BlockingQueue& rhs);
    BlockingQueue& operator=(const BlockingQueue& rhs);

    mutable mutex _mutex;
    condition_variable _condvar;
    deque<T> _queue;
    bool _is_shut_down = false;

public:
    BlockingQueue() : _mutex(), _condvar(), _queue() {}
    ~BlockingQueue() {}

    void ShutDown() {
        _is_shut_down = true;
        _condvar.notify_all();
        _queue.clear();
    }
    bool IsShutDown() { return _is_shut_down; }

    void Put(const T task) {
        lock_guard<mutex> lock(_mutex);
        if (!_is_shut_down) {
            { _queue.push_back(task); }
            _condvar.notify_all();
        }
    }

    T Take() {
        unique_lock<mutex> lock(_mutex);
        if (_queue.size() <= 0)
            _condvar.wait(lock);
        if (_is_shut_down || _queue.empty()) {
            throw exception();
        }
        T front(_queue.front());
        _queue.pop_front();
        return front;
    }

    size_t Size() const {
        lock_guard<mutex> lock(_mutex);
        return _queue.size();
    }
};
class _MyThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
    int num_thread;
    int num_thread_free;
public:
    _MyThreadPool(int num_threads) : stop(false) {
        num_thread = num_threads;
        num_thread_free = num_threads;
        for (int i = 0; i < num_threads; ++i) {
            workers.emplace_back(std::thread([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return !tasks.empty() || stop; });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task(); // 执行任务
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        num_thread_free++;
                    }
                }
                }));
        }
    }

    template<typename Func, typename... Args>
    bool addTask(Func&& func, Args&&... args) {
        {
            if (getNumIdleThreads() == 0) {
                return false;
            }
            std::lock_guard<std::mutex> lock(queue_mutex);
            auto task = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
            tasks.push(task);
            num_thread_free--;
        }
        condition.notify_one();

        return true;
    }

    // 判断线程池是否空闲
    void waitThreadPoolFree() {
        while (getNumIdleThreads() != num_thread);
    }

    // 获取空闲线程的数量
    int getNumIdleThreads() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return num_thread_free;
    }


    ~_MyThreadPool() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }
};