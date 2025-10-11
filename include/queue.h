//
// Created by anaple on 2024/4/25.
//

#ifndef UMS_SDK_QUEUE_H
#define UMS_SDK_QUEUE_H
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>              // 优化: 引入互斥锁以实现线程安全
#include <condition_variable> // 优化: 引入条件变量以实现高效的线程等待和唤醒
#include <atomic>             // 优化: 引入原子变量以安全地停止线程

class CircularQueue {
private:
    std::vector<std::string> buffer;
    int head;
    int tail;
    int maxSize;
    bool isFull;

    // --- 性能优化：添加线程同步机制 ---
    // 原因: 使用互斥锁来保护队列的共享数据（buffer, head, tail等），防止多线程访问时出现数据竞争。
    std::mutex mutex_;

    // 原因: 使用两个条件变量来替代旧的 sleep(1ms) 忙等待。
    // cond_not_full_ 用于在队列已满时阻塞生产者线程。
    // cond_not_empty_ 用于在队列为空时阻塞消费者线程。
    // 这样可以使线程在没有工作可做时进入深度睡眠，完全不消耗CPU。
    std::condition_variable cond_not_full_;
    std::condition_variable cond_not_empty_;

    // 原因: 使用一个原子布尔值作为停止标志，确保当我们需要停止队列时，
    // 所有正在等待的线程都能被安全地唤醒并退出。
    std::atomic<bool> stopped_{false};

public:
    CircularQueue(int size);

    // 优化: 修改函数签名以反映其可能阻塞的特性
    bool enqueue(const std::string& data); // 使用 const& 避免不必要的数据拷贝

    bool dequeue(std::string& data);

    bool isEmpty() const;

    int getSize() const;

    // 优化: 添加一个停止方法
    // 原因: 提供一个清晰的机制来通知所有正在等待的线程退出，从而实现程序的优雅关闭。
    void stop();
};
#endif //UMS_SDK_QUEUE_H
